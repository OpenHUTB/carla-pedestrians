import logging
from queue import Empty, Queue
from typing import Dict, List, Tuple, Union

import warnings
import numpy as np
import PIL
import torch
from pedestrians_scenarios.karma.renderers.renderer import Renderer

from pedestrians_video_2_carla.carla_utils.destroy import destroy_client_and_world
from pedestrians_video_2_carla.carla_utils.setup import *
from pedestrians_video_2_carla.walker_control.controlled_pedestrian import \
    ControlledPedestrian
from pedestrians_video_2_carla.walker_control.p3d_pose import P3dPose
from torch import Tensor
from pytorch_lightning.utilities import rank_zero_warn
from pytorch3d.transforms.rotation_conversions import matrix_to_euler_angles


try:
    import carla
except (ImportError, ModuleNotFoundError) as e:
    import pedestrians_video_2_carla.carla_utils.mock_carla as carla
    warnings.warn("Using mock carla.", category=ImportWarning)


class CarlaRenderer(Renderer):
    # TODO: reuse Karma and its CamerasManager
    def __init__(self, fps=30.0, fov=90.0, **kwargs) -> None:
        super().__init__(**kwargs)
        self.__fps = fps
        self.__fov = fov

    @torch.no_grad()
    def render(self,
               relative_pose_loc: Tensor,
               relative_pose_rot: Tensor,
               world_loc: Tensor,
               world_rot: Tensor,
               meta: List[Dict[str, Any]],
               **kwargs
               ) -> List[np.ndarray]:
        rendered_videos = len(relative_pose_loc)

        if relative_pose_loc is None and relative_pose_loc is None:
            rank_zero_warn(
                "Neither relative pose locations nor rotations are available. " +
                "This will result in rendering reference pose")
        elif relative_pose_loc is None:
            rank_zero_warn(
                "Relative pose locations are not available, falling back to reference locations. " +
                "Please note that this may result in weird rendering effects.")
        elif relative_pose_loc is None:
            rank_zero_warn(
                "Relative pose rotations are not available, falling back to reference rotations. " +
                "Please note that this may result in weird rendering effects.")

        # prepare connection to carla as needed - TODO: should this be in (logging) epoch start?
        client, world = setup_client_and_world(fps=self.__fps)

        clip_length = self.__guess_clip_length(
            relative_pose_loc,
            relative_pose_rot,
            world_loc,
            world_rot
        )

        for clip_idx in range(rendered_videos):
            video = self.render_clip(
                clip_length,
                relative_pose_loc[clip_idx] if relative_pose_loc is not None else None,
                relative_pose_rot[clip_idx] if relative_pose_rot is not None else None,
                world_loc[clip_idx],
                world_rot[clip_idx],
                meta['age'][clip_idx],
                meta['gender'][clip_idx],
                world,
                rendered_videos
            )
            yield video

        # close connection to carla as needed - TODO: should this be in (logging) epoch end?
        if (client is not None) and (world is not None):
            destroy_client_and_world(client, world)

    def __guess_clip_length(self,
                            *args
                            ) -> int:
        """Retrieves clip length from first available source. Assumes that all clips in batch are of the same length."""
        for arg in args:
            if arg is not None:
                return len(arg[0])
        return 1

    @torch.no_grad()
    def render_clip(self,
                    clip_length: int,
                    relative_pose_loc_clip: Union[Tensor, None],
                    relative_pose_rot_clip: Union[Tensor, None],
                    world_loc_clip: Tensor,
                    world_rot_clip: Tensor,
                    age: str,
                    gender: str,
                    world: 'carla.World',
                    rendered_videos: int
                    ):
        bound_pedestrian = ControlledPedestrian(
            world=world,
            age=age,
            gender=gender,
            reference_pose=P3dPose,
            max_spawn_tries=10+rendered_videos,
            device=relative_pose_loc_clip.device
        )
        camera_queue = Queue()
        camera_rgb = setup_camera(
            world, camera_queue, bound_pedestrian, self._image_size, self.__fov)

        if relative_pose_loc_clip is None or relative_pose_rot_clip is None:
            ref_abs_pose_loc, ref_abs_pose_rot = bound_pedestrian.current_pose.pose_to_tensors(
                bound_pedestrian.current_pose.relative)

            if relative_pose_loc_clip is None:
                relative_pose_loc_clip = [ref_abs_pose_loc] * clip_length

            if relative_pose_rot_clip is None:
                relative_pose_rot_clip = [ref_abs_pose_rot] * clip_length

        video = []
        for relative_pose_loc_frame, relative_pose_rot_frame, world_loc_frame, world_rot_frame in zip(relative_pose_loc_clip, relative_pose_rot_clip, world_loc_clip, world_rot_clip):
            frame = self.render_frame(relative_pose_loc_frame, relative_pose_rot_frame,
                                      world_loc_frame, world_rot_frame,
                                      world, bound_pedestrian, camera_queue)
            video.append(frame)

        camera_rgb.stop()
        camera_rgb.destroy()

        bound_pedestrian.walker.destroy()

        return np.stack(video, axis=0)

    @torch.no_grad()
    def render_frame(self,
                     relative_pose_loc_frame: Tensor,
                     relative_pose_rot_frame: Tensor,
                     world_loc_frame: Tensor,
                     world_rot_frame: Tensor,
                     world: 'carla.World',
                     bound_pedestrian: ControlledPedestrian,
                     camera_queue: Queue
                     ):
        abs_pose = bound_pedestrian.current_pose.tensors_to_pose(
            relative_pose_loc_frame, relative_pose_rot_frame)

        # TODO: get root transform so that the contact points are correct
        root_hips_transform = None

        bound_pedestrian.apply_pose(
            pose_snapshot=abs_pose,
            root_hips_transform=root_hips_transform,
        )

        world_loc = world_loc_frame.cpu().numpy().astype(float)
        world_rot = -np.rad2deg(matrix_to_euler_angles(world_rot_frame,
                                "XYZ").cpu().numpy()).astype(float)

        bound_pedestrian.teleport_by(
            carla.Transform(
                carla.Location(
                    x=world_loc[0],
                    y=world_loc[1],
                    z=-world_loc[2]
                ),
                carla.Rotation(
                    pitch=world_rot[1],
                    yaw=world_rot[2],
                    roll=world_rot[0]
                )
            ),
            from_initial=True
        )

        world_frame = world.tick()

        frames = []
        sensor_data = None

        carla_img = torch.zeros(
            (self._image_size[1], self._image_size[0], 3), dtype=torch.uint8)
        if world_frame:
            # drain the sensor queue
            try:
                while (sensor_data is None) or sensor_data.frame < world_frame:
                    sensor_data = camera_queue.get(True, 1.0)
                    frames.append(sensor_data)
            except Empty:
                logging.getLogger(__name__).warn(
                    "Sensor data skipped in frame {}".format(world_frame))

            if len(frames):
                data = frames[-1]
                data.convert(carla.ColorConverter.Raw)
                img = PIL.Image.frombuffer('RGBA', (data.width, data.height),
                                           data.raw_data, "raw", 'RGBA', 0, 1)  # load
                img = img.convert('RGB')  # drop alpha
                # the data is actually in BGR format, so switch channels
                carla_img = np.array(img, dtype=np.uint8)[..., ::-1].copy()

        return carla_img
