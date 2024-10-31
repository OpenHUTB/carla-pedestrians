import glob
import logging
import multiprocessing as mp
import os
import time
from typing import Any, Dict, Iterable, List, Tuple
from uuid import uuid4

from PIL import Image
import carla
import numpy as np
import pandas as pd
from pedestrians_scenarios.karma.cameras import CamerasManager
from pedestrians_scenarios.karma.karma import Karma, KarmaStage
from pedestrians_scenarios.karma.karma_data_provider import KarmaDataProvider
from pedestrians_scenarios.karma.pose.pose_dict import (
    convert_list_to_pose_dict, convert_pose_2d_dict_to_list,
    convert_pose_dict_to_list, get_pedestrian_pose_dicts)
from pedestrians_scenarios.karma.pose.projection import project_pose
from pedestrians_scenarios.karma.utils.conversions import (
    convert_transform_to_list, convert_vector3d_to_list)
from pedestrians_scenarios.karma.utils.deepcopy import deepcopy_transform
from pedestrians_scenarios.karma.walker import Walker
from srunner.scenariomanager.actorcontrols.pedestrian_control import \
    PedestrianControl
from tqdm.auto import trange

from .pedestrian_profile import PedestrianProfile, ExamplePedestrianProfiles
from .standard_distribution import StandardDistribution


class BatchGenerator(mp.Process):
    """
    Common class that generates single batch of clips/data.
    """

    def __init__(self,
                 outfile: str,  # csv output file
                 # so the one process writes to the same file (it is shared among all processes)
                 outfile_lock: mp.Lock,
                 outputs_dir: str,  # directory where the output video files will be stored
                 # queue to communicate with the main process (to know when the batch is done and how many clips were generated)
                 queue: mp.Queue,
                 batch_idx: int = 0,  # index of the batch
                 # how many videos to generate in a single world at the same time (the world is not reset between clips and the created only at the beginning of the batch)
                 batch_size: int = 1,
                 clip_length_in_frames: int = 900,
                 # there is problem with kids in real datasets, there are only few of them,
                 # in our case have equal number of adults and children which is less realistic but more useful
                 pedestrian_distributions: Iterable[Tuple[PedestrianProfile, float]] = (
                     (ExamplePedestrianProfiles.adult_female.value, 0.25),
                     (ExamplePedestrianProfiles.adult_male.value, 0.25),
                     (ExamplePedestrianProfiles.child_female.value, 0.25),
                     (ExamplePedestrianProfiles.child_male.value, 0.25)
                 ),
                 camera_distances_distributions: Iterable[Iterable[StandardDistribution]] = ((
                     StandardDistribution(-7.0, 2.0),
                     StandardDistribution(0.0, 0.25),
                     StandardDistribution(1.0, 0.25)
                 ),),
                 camera_fov: float = 90.0,
                 camera_image_size: Tuple[int, int] = (1600, 600),
                 waypoint_jitter_scale: float = 1.0,
                 **kwargs) -> None:
        super().__init__(
            group=kwargs.get('group', None),
            target=kwargs.get('target', None),
            name=kwargs.get('name', None),
            daemon=kwargs.get('daemon', None)
        )

        self._outfile = outfile
        self._outfile_lock = outfile_lock
        self._outputs_dir = outputs_dir
        self._queue = queue

        self._batch_idx = batch_idx
        self._batch_size = batch_size
        self._camera_distances_distributions = camera_distances_distributions
        self._pedestrian_distributions = pedestrian_distributions

        self._clip_length_in_frames = clip_length_in_frames
        self._camera_fov = camera_fov
        self._camera_image_size = camera_image_size
        self._waypoint_jitter_scale = waypoint_jitter_scale

        self._karma = None
        self._kwargs = kwargs

    def run(self) -> None:
        no_of_generated_clips = 0
        with Karma(**self._kwargs) as karma:
            self._karma = karma
            map_name = self.get_map_for_batch()
            karma.reset_world(map_name)

            batch_data, no_of_generated_clips = self.generate_batch(map_name)
            if no_of_generated_clips > 0:
                with self._outfile_lock:
                    header = False
                    if not os.path.exists(self._outfile):
                        header = True
                    else:
                        df = pd.read_csv(self._outfile, nrows=1)
                        header = len(df) == 0

                    pd.DataFrame(batch_data).to_csv(self._outfile, mode='a',
                                                    header=header,
                                                    index=False)

            self._karma = None

        logging.getLogger(__name__).debug(
            f'Generated {no_of_generated_clips} clips.')

        self._queue.put(no_of_generated_clips)

    def get_profiles(self) -> Iterable[Iterable[PedestrianProfile]]:
        """
        Get the pedestrian profiles for each clip.
        By default, this will return a random, single pedestrian profile per clip.
        """
        profiles, weights = zip(*self._pedestrian_distributions)
        indices = KarmaDataProvider.get_rng().choice(np.arange(len(profiles)), size=(
            self._batch_size,), replace=True, p=weights)
        return [(profiles[i],) for i in indices]

    def get_spawn_points(self, profiles: Iterable[Iterable[PedestrianProfile]]) -> Iterable[Iterable[carla.Transform]]:
        """
        Get the spawn points for each pedestrian.
        By default, spawn points will be randomly chosen from list of allowed spawn points.

        :param profiles: List of pedestrian profiles that will be used.
        :type profiles: Iterable[Iterable[PedestrianProfile]]
        """
        spawn_points = []
        for clip_profiles in profiles:
            in_clip = []
            for _ in clip_profiles:
                in_clip.append(KarmaDataProvider.get_pedestrian_spawn_point())
            spawn_points.append(in_clip)
        return spawn_points

    def get_models(self, profiles: Iterable[Iterable[PedestrianProfile]]) -> Iterable[Iterable[str]]:
        """
        Get the models (blueprint names) for each pedestrian.
        In addition to sex and age we need blueprint (appearance+) of pedestrian.

        :param profiles: List of pedestrian profiles that will be used.
        :type profiles: Iterable[Iterable[PedestrianProfile]]
        """
        return [
            [
                Walker.get_model_by_age_and_gender(profile.age, profile.gender)
                for profile in clip_profiles
            ] for clip_profiles in profiles
        ]

    def get_pedestrians(self, models: Iterable[Iterable[str]], spawn_points: Iterable[Iterable[carla.Transform]]) -> Iterable[Iterable[Walker]]:
        """
        Generate pedestrians for a batch.
        In general, this should not tick the world, but rather just create the pedestrians.
        Karma.tick() will be called after this to actually spawn the pedestrians.

        :param models: [description]
        :type models: Iterable[Iterable[str]]
        :param spawn_points: [description]
        :type spawn_points: Iterable[Iterable[carla.Transform]]
        :return: [description]
        :rtype: Iterable[Iterable[Walker]]
        """
        pedestrians = []
        for clip_idx, (clip_models, clip_spawn_points) in enumerate(zip(models, spawn_points)):
            in_clip = []
            try:
                for model, spawn_point in zip(clip_models, clip_spawn_points):
                    in_clip.append(
                        Walker(model=model, spawn_point=spawn_point, tick=False))
            except RuntimeError:
                logging.getLogger(__name__).info(
                    f'Failed to create pedestrians for clip {clip_idx} in batch {self._batch_idx}.')
            pedestrians.append(in_clip)
        return pedestrians

    def _rotate_pedestrian_towards_location(self, pedestrian: Walker, location: carla.Location) -> None:
        """
        Helper method in setting up a single pedestrian.
        It does not tick the world.
        """

        direction_unit = (location -
                          pedestrian.get_transform().location)
        direction_unit.z = 0  # ignore height
        direction_unit = direction_unit.make_unit_vector()

        # shortcut, since we're ignoring elevation
        # compute how we need to rotate the pedestrian to face the waypoint
        pedestrian_transform = deepcopy_transform(pedestrian.get_transform())
        delta = np.rad2deg(np.arctan2(direction_unit.y, direction_unit.x))
        pedestrian_transform.rotation.yaw = pedestrian_transform.rotation.yaw + delta
        pedestrian.set_transform(pedestrian_transform)

    def get_camera_distances(self) -> Iterable[Iterable[float]]:
        """
        Get the camera distances for each clip.
        By default, this will return a random, single camera position per clip.
        """
        distances = []
        total_cameras = len(self._camera_distances_distributions)
        for camera_idx in range(total_cameras):
            camera_distances = []
            for i in range(3):  # x,y,z
                camera_distances.append(KarmaDataProvider.get_rng().normal(
                    loc=self._camera_distances_distributions[camera_idx][i].mean,
                    scale=self._camera_distances_distributions[camera_idx][i].std,
                    size=(self._batch_size,)
                ))
            distances.append(list(zip(*camera_distances)))
        return list(zip(*distances))

    def get_camera_look_at(self, pedestrians: Iterable[Iterable[Walker]], camera_distances: Iterable[Iterable[float]]) -> Iterable[Iterable[carla.Transform]]:
        """
        Get the camera look at points for each clip.
        By default, this will return a random, single camera position per clip.
        """
        return [
            self.get_clip_camera_look_at(clip_idx, clip_pedestrians, clip_camera_distances
                                         ) if len(clip_pedestrians) > 0 else []
            for clip_idx, (clip_pedestrians, clip_camera_distances) in enumerate(zip(pedestrians, camera_distances))
        ]

    def setup_pedestrians(self, pedestrians: Iterable[Iterable[Walker]], profiles: Iterable[Iterable[PedestrianProfile]], camera_look_at: Iterable[Iterable[carla.Transform]]) -> None:
        """
        Setup the pedestrians for a batch. It is called after the pedestrians are spawned in the world.
        """
        for clip_idx, (clip_pedestrians, clip_profiles, clip_look_at) in enumerate(zip(pedestrians, profiles, camera_look_at)):
            if len(clip_pedestrians) > 0:
                self.setup_clip_pedestrians(
                    clip_idx, clip_pedestrians, clip_profiles, clip_look_at)

    def get_pedestrians_control(self, pedestrians: Iterable[Iterable[Walker]], profiles: Iterable[Iterable[PedestrianProfile]], camera_look_at: Iterable[Iterable[carla.Transform]]) -> Iterable[Iterable[PedestrianControl]]:
        """
        Sets up controllers for all pedestrians.
        :param pedestrians: [description]
        :type pedestrians: Iterable[Iterable[Walker]]
        :param profiles: [description]
        :type profiles: Iterable[Iterable[PedestrianProfile]]
        :param camera_look_at: [description]
        :type camera_look_at: Iterable[Iterable[carla.Transform]]
        :return: [description]
        :rtype: Iterable[Iterable[PedestrianControl]]
        """
        return [
            self.get_clip_pedestrians_control(clip_idx, clip_pedestrians, clip_profiles, clip_look_at
                                              ) if len(clip_pedestrians) > 0 else []
            for clip_idx, (clip_pedestrians, clip_profiles, clip_look_at) in enumerate(zip(pedestrians, profiles, camera_look_at))
        ]

    def get_clip_pedestrians_control(self, clip_idx: int, pedestrians: Iterable[Walker], profiles: Iterable[PedestrianProfile], camera_look_at: Iterable[carla.Transform]) -> Iterable[PedestrianControl]:
        """
        Get the pedestrians controls for a single clip.
        """
        raise NotImplementedError()

    def setup_clip_pedestrians(self, clip_idx: int, pedestrians: Iterable[Walker], profiles: Iterable[PedestrianProfile], camera_look_at: Iterable[carla.Transform]) -> None:
        """
        Setup the pedestrians in a single clip (e.g. add props or other things).
        By default does nothing.
        """
        pass

    def get_clip_camera_look_at(self, clip_idx: int, pedestrians: Iterable[Walker], camera_distances: Iterable[float]) -> Iterable[carla.Transform]:
        """
        Get the camera look at points for a single clip.
        """
        raise NotImplementedError()

    def get_camera_managers(
        self,
        camera_look_at: Iterable[Iterable[carla.Transform]],
        camera_distances: Iterable[Iterable[float]],
        pedestrians: Iterable[Iterable[Walker]]
    ) -> Iterable[Iterable[CamerasManager]]:
        """
        Get camera managers for batch. By default this will create a single
        CameraManager per clip, merging images from all specified clip cameras.
        This method ticks the world with each CameraManager created,
        since cameras have to be spawned to attach event listeners.

        :param camera_look_at: [description]
        :type camera_look_at: Iterable[Iterable[carla.Transform]]
        :param camera_distances: [description]
        :type camera_distances: Iterable[Iterable[float]]
        """
        managers = []
        for clip_idx, (clip_look_at, clip_distances, clip_pedestrians) in enumerate(zip(camera_look_at, camera_distances, pedestrians)):
            managers.append(self.get_clip_camera_managers(clip_idx, clip_look_at, clip_distances, clip_pedestrians
                                                          ))
        return managers

    def get_clip_camera_managers(self, clip_idx: int, camera_look_at: Iterable[carla.Transform], camera_distances: Iterable[float], pedestrians: Iterable[Walker]) -> Iterable[CamerasManager]:
        """
        Get the camera managers for a single clip. By default all created cameras are FreeCamera.
        This method ticks the world since cameras have to be spawned to attach event listeners.
        """
        if len(camera_look_at):
            manager = CamerasManager(
                self._karma,
                outputs_dir=os.path.join(self._outputs_dir, 'clips'),
            )
            manager.create_free_cameras(
                cameras=zip(camera_look_at, camera_distances),
                image_size=self._camera_image_size,
                fov=self._camera_fov,
            )
            return [manager, ]
        else:
            return []

    def get_map_for_batch(self) -> str:
        """
        Get the map for a batch. All pedestrians in batch will be spawned in the same world at the same time.
        """
        return KarmaDataProvider.get_rng().choice(KarmaDataProvider.get_available_maps())

    def generate_batch(self, map_name: str) -> Iterable[Dict]:
        """
        Generate a single batch.
        """
        profiles = self.get_profiles()
        spawn_points = self.get_spawn_points(profiles)
        models = self.get_models(profiles)
        pedestrians = self.get_pedestrians(models, spawn_points)

        # spawn pedestrians in world
        self._karma.tick()

        # if no pedestrians, skip batch
        if sum(len(pedestrians_per_clip) for pedestrians_per_clip in pedestrians) == 0:
            logging.getLogger(__name__).info(
                f'No pedestrians spawned in batch {self._batch_idx}, skipping.')
            return [], 0

        camera_distances = self.get_camera_distances()
        camera_look_at = self.get_camera_look_at(
            pedestrians, camera_distances)
        camera_managers = self.get_camera_managers(
            camera_look_at, camera_distances, pedestrians)
        # no need to tick, CamerasManager already did it

        self.setup_pedestrians(pedestrians, profiles, camera_look_at)

        # apply settings to pedestrians by ticking the world
        self._karma.tick()

        controllers = self.get_pedestrians_control(
            pedestrians, profiles, camera_look_at)

        # apply settings to pedestrians by ticking the world in case controllers needed to
        # update something (e.g. rotate pedestrians to face desired trajectory)
        self._karma.tick()

        for clip_controllers in controllers:
            for controller in clip_controllers:
                self._karma.register_controller(controller)

        # sleep for a little bit to apply everything before recording starts
        time.sleep(2)

        # start cameras
        recordings = []
        for managers in camera_managers:
            clip_id = str(uuid4())
            clip_recordings = []
            for manager_idx, manager in enumerate(managers):
                recording_name = f'{clip_id}-{manager_idx}'
                clip_recordings.append(recording_name)
                manager.start_recording(recording_name)
            recordings.append(clip_recordings)

        # prepare data capture callback
        frame_data = [[] for _ in range(self._batch_size)]
        capture_callback_id = self._karma.register_callback(
            KarmaStage.tick, lambda snapshot: self.capture_frame_data(snapshot, pedestrians, frame_data))

        # move simulation forward the required number of frames
        # and capture per-frame data
        map_basename = os.path.basename(map_name)
        for _ in trange(self._clip_length_in_frames, desc=f'Frame (in {map_basename})', position=1, leave=False):
            self._karma.tick()

        # sleep for a little bit to let cameras & data capture finish recording
        time.sleep(2)

        # stop data capture
        self._karma.unregister_callback(capture_callback_id)

        # stop cameras
        recorded_frames = []
        for managers in camera_managers:
            clip_recorded_frames = []
            for manager in managers:
                clip_recorded_frames.append(manager.stop_recording())
            recorded_frames.append(clip_recorded_frames)

        # unregister controllers
        reached_first_waypoint = []
        for clip_controllers in controllers:
            clip_reached_first_waypoint = []
            for controller in clip_controllers:
                self._karma.unregister_controller(controller)
                clip_reached_first_waypoint.append(
                    controller.check_reached_first_waypoint())
            reached_first_waypoint.append(
                len(clip_reached_first_waypoint) > 0 and all(clip_reached_first_waypoint))

        # collect batch data
        batch_data, clips_count = self.collect_batch_data(
            map_name, profiles, spawn_points, models, pedestrians, camera_managers, recordings, recorded_frames, frame_data, reached_first_waypoint)

        return batch_data, clips_count

    def capture_frame_data(self, snapshot: carla.WorldSnapshot, pedestrians: Iterable[Iterable[Walker]], out_frame_data: Iterable[Iterable[Iterable[Dict]]]):
        """
        Capture per-frame data.
        """
        for clip_pedestrians, clip_data in zip(pedestrians, out_frame_data):
            current_frame_data = []
            for pedestrian in clip_pedestrians:
                pedestrian_snapshot = snapshot.find(pedestrian.id)
                pedestrian_transform = pedestrian_snapshot.get_transform()
                pedestrian_velocity = pedestrian_snapshot.get_velocity()

                world_pose, component_pose, relative_pose = get_pedestrian_pose_dicts(
                    pedestrian)

                current_frame_data.append({
                    'world.frame': snapshot.frame,
                    'frame.pedestrian.id': pedestrian.id,
                    'frame.pedestrian.transform': convert_transform_to_list(pedestrian_transform),
                    'frame.pedestrian.velocity': convert_vector3d_to_list(pedestrian_velocity),
                    'frame.pedestrian.pose.world': convert_pose_dict_to_list(world_pose),
                    'frame.pedestrian.pose.component': convert_pose_dict_to_list(component_pose),
                    'frame.pedestrian.pose.relative': convert_pose_dict_to_list(relative_pose),
                    'frame.pedestrian.is_crossing': pedestrian.is_crossing,
                })

            clip_data.append(current_frame_data)

    def collect_batch_data(self, map_name, profiles, spawn_points, models, pedestrians, camera_managers, recordings, recorded_frames, captured_data, reached_first_waypoint) -> Tuple[List[Dict], int]:
        batch_data = []
        clips_count = 0
        for clip_idx in range(self._batch_size):
            clip_managers: Iterable[CamerasManager] = camera_managers[clip_idx]

            if not sum(len(c) for c in captured_data[clip_idx]):
                logging.getLogger(__name__).info(
                    f'No data was captured for clip {clip_idx}, skipping.')
                for manager_idx, manager in enumerate(clip_managers):
                    self.remove_clip_files(manager.outputs_dir,
                                           recordings[clip_idx][manager_idx])
                continue
            if not reached_first_waypoint[clip_idx]:
                logging.getLogger(__name__).info(
                    f'At least one of the pedestrians in {clip_idx} did not reach first waypoint, skipping.')
                for manager_idx, manager in enumerate(clip_managers):
                    self.remove_clip_files(manager.outputs_dir,
                                           recordings[clip_idx][manager_idx])
                continue

            clip_models: Iterable[str] = models[clip_idx]
            clip_profiles: Iterable[PedestrianProfile] = profiles[clip_idx]
            clip_spawn_points: Iterable[carla.Transform] = spawn_points[clip_idx]
            clip_pedestrians: Iterable[Walker] = pedestrians[clip_idx]

            clip_captured_data: Iterable[Dict] = sorted(
                captured_data[clip_idx], key=lambda x: x[0]['world.frame'])
            clip_captured_data_frames: Iterable[int] = [
                data[0]['world.frame']
                for data in clip_captured_data
            ]

            clip_data = []

            for manager_idx, manager in enumerate(clip_managers):
                clip_id: str = recordings[clip_idx][manager_idx] if len(
                    recordings[clip_idx]) else None

                if clip_id is None:
                    logging.getLogger(__name__).info(
                        f'No recording was created for clip {clip_idx} camera {manager_idx}, skipping.')
                    continue

                clip_recorded_frames: Iterable[int] = sorted(
                    recorded_frames[clip_idx][manager_idx])

                for camera_idx, synced_cameras in enumerate(manager.get_synchronized_cameras()):
                    first_camera = synced_cameras[0]

                    # get rgb camera
                    rgb_camera = next(
                        (x for x in synced_cameras if x.camera_type == 'rgb'), None)
                    if rgb_camera is not None:
                        rgb_camera_idx = synced_cameras.index(rgb_camera)

                    # if semantic segmentation camera is present, we can use it to check if the pedestrians are visible
                    semantic_camera = next(
                        (x for x in synced_cameras if x.camera_type == 'semantic_segmentation'), None)
                    if semantic_camera is not None:
                        semantic_camera_idx = synced_cameras.index(semantic_camera)

                    # TODO: Some of the cameras can move (e.g. attached to a vehicle), so camera position should be per frame
                    # but for now we assume only FreeCameras that are static
                    camera_transform = first_camera.get_transform()

                    min_frame = clip_recorded_frames[0]
                    max_frame = clip_recorded_frames[-1]

                    skip = 0
                    camera_data = []
                    has_pedestrian_data = False
                    for frame_idx, world_frame in enumerate(range(min_frame, max_frame + 1)):
                        # skip frame if it is not in the recorded frames - we do not want to have
                        # indexing differences between video and captured data
                        if world_frame not in clip_recorded_frames:
                            skip += 1
                            continue

                        # extract frame data or minimal dict if not available
                        if world_frame in clip_captured_data_frames:
                            frame = clip_captured_data[clip_captured_data_frames.index(
                                world_frame)]
                        else:
                            frame = [{
                                'frame.pedestrian.id': p.id,
                                'world.frame': world_frame
                            } for p in range(len(clip_pedestrians))]

                        for pedestrian_idx, (profile, model, spawn_point, pedestrian) in enumerate(zip(clip_profiles, clip_models, clip_spawn_points, clip_pedestrians)):
                            assert frame[pedestrian_idx]['frame.pedestrian.id'] == pedestrian.id, 'Pedestrian ID mismatch'
                            assert frame[pedestrian_idx]['world.frame'] == world_frame, 'World frame mismatch'

                            full_frame_data = {
                                'id': clip_id,
                                'world.map': map_name,
                                'camera.idx': camera_idx,
                                'camera.transform': convert_transform_to_list(camera_transform),
                                'camera.width': self._camera_image_size[0],
                                'camera.height': self._camera_image_size[1],
                                'pedestrian.idx': pedestrian_idx,
                                'pedestrian.model': model,
                                'pedestrian.age': profile.age,
                                'pedestrian.gender': profile.gender,
                                'pedestrian.spawn_point': convert_transform_to_list(spawn_point),
                                'frame.idx': frame_idx - skip,
                            }

                            if rgb_camera is not None:
                                full_frame_data['camera.recording'] = f'clips/{clip_id}-{rgb_camera_idx}.mp4'

                            if semantic_camera is not None:
                                full_frame_data[
                                    'camera.semantic_segmentation'] = f'clips/{clip_id}-{semantic_camera_idx}.apng'

                            full_frame_data.update(frame[pedestrian_idx])

                            if 'frame.pedestrian.pose.world' in frame[pedestrian_idx]:
                                camera_pose = project_pose(
                                    convert_list_to_pose_dict(
                                        frame[pedestrian_idx]['frame.pedestrian.pose.world']),
                                    camera_transform,
                                    first_camera
                                )
                                full_frame_data['frame.pedestrian.pose.camera'] = convert_pose_2d_dict_to_list(
                                    camera_pose)
                                np_camera_pose = np.array(
                                    full_frame_data['frame.pedestrian.pose.camera']).reshape((-1, 2))
                                is_camera_pose_in_frame = np.all(
                                    np.isfinite(np_camera_pose))
                                if is_camera_pose_in_frame:
                                    is_camera_pose_in_frame = np.any(
                                        np_camera_pose[:, 0] >= 0) & np.any(
                                        np_camera_pose[:, 1] >= 0) & np.any(
                                        np_camera_pose[:, 0] <= self._camera_image_size[0]) & np.any(
                                        np_camera_pose[:, 1] <= self._camera_image_size[1])
                                full_frame_data['frame.pedestrian.pose.in_frame'] = bool(
                                    is_camera_pose_in_frame)

                                has_pedestrian_data = has_pedestrian_data or is_camera_pose_in_frame

                            del full_frame_data['frame.pedestrian.id']

                            camera_data.append(full_frame_data)

                    if has_pedestrian_data:
                        if semantic_camera is None:
                            clip_data.extend(camera_data)
                        elif self.has_pedestrian_in_semantic_segmentation(
                            os.path.join(manager.outputs_dir,
                                         '..',
                                         camera_data[0]['camera.semantic_segmentation']),
                            camera_data
                        ):
                            clip_data.extend(camera_data)

            if len(clip_data) > 0:
                batch_data.extend(clip_data)
                clips_count += 1
            else:
                logging.getLogger(__name__).info(
                    f'No pedestrian is visible in {clip_idx}, removing files.')
                self.remove_clip_files(manager.outputs_dir, clip_id)

         # batch_data has been flattened to a list of dicts
        return batch_data, clips_count

    def remove_clip_files(self, outputs_dir, clip_id):
        if clip_id is not None:
            path = os.path.join(outputs_dir, '{}-*'.format(clip_id))
            exts = ['mp4', 'apng', 'npy']
            files = [f for ext in exts for f in glob.glob(f'{path}.{ext}')]
            for file in files:
                os.remove(file)

    def has_pedestrian_in_semantic_segmentation(self, semantic_segmentation_file: str, camera_data: List[Dict[str, Any]]) -> bool:
        """
        Checks if there is a pedestrian in the semantic segmentation image.
        https://carla.readthedocs.io/en/0.9.13/ref_sensors/#semantic-segmentation-camera
        Pedestrian palette index is 4.

        :param semantic_segmentation_file:
        :return:
        """
        has_pedestrian_in_clip = False
        with Image.open(semantic_segmentation_file) as img:
            for frame in range(img.n_frames):
                has_pedestrian_in_frame = False
                img.seek(frame)
                img_array = np.array(img)
                if np.any(img_array == 4):
                    has_pedestrian_in_frame = True
                camera_data[frame]['frame.pedestrian.pose.in_segmentation'] = has_pedestrian_in_frame
                has_pedestrian_in_clip = has_pedestrian_in_clip or has_pedestrian_in_frame

        return has_pedestrian_in_clip
