import os
import uuid
from typing import Dict, Iterable, List, Tuple, Union

import av
import carla
import numpy as np
from tqdm.auto import tqdm

from pedestrians_scenarios.karma.renderers.dvs_renderer import DVSRenderer
from pedestrians_scenarios.karma.renderers.segmentation_renderer import SegmentationRenderer

from ..karma import Karma, KarmaStage
from ..karma_data_provider import KarmaDataProvider
from .camera import Camera
from .free_camera import FreeCamera


class CamerasManager(object):
    def __init__(self,
                 karma: Karma,
                 outputs_dir: str,
                 ):
        self.__karma = karma

        self.__outputs_dir = outputs_dir
        os.makedirs(self.__outputs_dir, exist_ok=True)

        # managed cameras are those that Cameras Manager is responsible for (creation and deletion)
        self.__managed_cameras: Dict[int, FreeCamera] = {}
        # streamed cameras are all cameras that the Camera Manager should get data from
        # this includes managed cameras and e.g. vehicle cameras
        self.__streamed_cameras: Dict[int, Union[FreeCamera, Camera]] = {}
        # some cameras are synchronized, i.e. they show the same scene, but in different 'modes'
        self.__synchronized_cameras: List[List[int]] = []

        self.__mp4_containers = []
        self.__segmentation_containers = []
        self.__dvs_containers = []

        self.__tick_callback_id = None
        self.__close_callback_id = None

        self.__segmentation_renderer = SegmentationRenderer()
        self.__dvs_renderer = DVSRenderer()

        self.__captured_frames: List[int] = None

    @property
    def outputs_dir(self) -> str:
        return self.__outputs_dir

    @property
    def __mp4_cameras(self) -> List[Camera]:
        return [camera
                for camera in self.__streamed_cameras.values()
                if camera.camera_type in ['rgb', 'depth']]  # for now we're not saving raw depth data

    @property
    def __segmentation_cameras(self) -> List[Camera]:
        return [camera
                for camera in self.__streamed_cameras.values()
                if camera.camera_type in ['semantic_segmentation', 'instance_segmentation']]

    @property
    def __dvs_cameras(self) -> List[Camera]:
        return [camera
                for camera in self.__streamed_cameras.values()
                if camera.camera_type == 'dvs']

    def create_free_cameras(self,
                            cameras: Iterable[Tuple[carla.Transform, Iterable[float]]],
                            image_size: Tuple[int, int] = (1600, 600),
                            fov: float = 90.0,
                            camera_types: Iterable[str] = (
                                'rgb', 'semantic_segmentation', 'dvs'),
                            ):
        """
        Creates cameras at the given locations and registers them with the Camera Manager.
        This forcibly ticks the world, because cameras need to be spawned to attach listeners.

        :param cameras: List of carla.Transform look_at, (x, y, z) distance tuples
        :type cameras: List[Tuple[carla.Transform, Tuple[float]]]
        :param image_size: Size of the camera image (width, height).
            Currently all cameras need to have the same dimensions. Default: (1600, 600)
        :type image_size: Tuple[int, int]
        :param fov: Field of view of the camera in degrees. Default: 90.0
        :type fov: float
        :param camera_types: List of camera types to create. Default: ('rgb', 'semantic_segmentation', 'dvs')
        :type camera_types: Iterable[str]
        """
        for look_at, distance in cameras:
            synced = []
            for camera_type in camera_types:
                camera = FreeCamera(
                    look_at=look_at,
                    distance=distance,
                    image_size=image_size,
                    fov=fov,
                    tick=False,
                    register=False,
                    camera_type=camera_type,
                )
                self.__managed_cameras[camera.id] = camera
                synced.append(camera.id)
            if len(synced) > 1:
                self.__synchronized_cameras.append(synced)

        # spawn cameras
        self.__karma.tick()

        # register cameras with KarmaDataProvider (listeners)
        for camera in self.__managed_cameras.values():
            KarmaDataProvider.register_sensor_queue(camera.sensor)

        # register cameras for streaming
        for camera in self.__managed_cameras.values():
            self.register_streamed_camera(camera)

    def register_streamed_camera(self, camera: Union[FreeCamera, Camera, carla.Sensor]):
        """
        Adds a camera to the list of cameras that the Camera Manager should get data from.
        Assumption: camera life cycle is managed by the caller.

        If camera is a carla.Sensor, it will be wrapped in a Camera object,
        which will try to register the data queue for it with
        KarmaDataProvider.register_sensor_queue().

        Assumptions: camera already is spawned it the world, camers other than
        carla.Sensor are assumed to be already registered with KarmaDataProvider.

        :param camera: Karma.FreeCamera or CARLA camera-like Sensor
        :type camera: Union[FreeCamera, carla.Sensor]
        """
        if self.__mp4_containers is not None and len(self.__mp4_containers):
            raise RuntimeError(
                'Recording session is in progress, all cameras must be registered before recording starts.')

        if isinstance(camera, carla.Sensor):
            camera = Camera(sensor=camera)

        self.__streamed_cameras[camera.id] = camera
        self.__captured_frames = []

    def start_recording(self, session_id: str = None) -> Iterable[str]:
        """
        Starts recording stream for all cameras.
        """
        if session_id is None:
            session_id = str(uuid.uuid4())

        for ci, camera in enumerate(self.__mp4_cameras):
            name = '{}-{}'.format(session_id, ci)
            container = av.open(os.path.join(self.__outputs_dir,
                                             f'{name}.mp4'), mode="w")
            stream = container.add_stream('libx264', rate=30)
            stream.width = camera.image_size[0]
            stream.height = camera.image_size[1]
            stream.pix_fmt = "yuv420p"
            stream.options = {}

            self.__mp4_containers.append((name, container, stream))

        for si, camera in enumerate(self.__segmentation_cameras):
            name = '{}-{}'.format(session_id, si+len(self.__mp4_cameras))
            self.__segmentation_containers.append((name, [], []))

        for di, camera in enumerate(self.__dvs_cameras):
            name = '{}-{}'.format(session_id, di +
                                  len(self.__mp4_cameras)+len(self.__segmentation_cameras))
            self.__dvs_containers.append((name, []))

        self.__tick_callback_id = self.__karma.register_callback(
            KarmaStage.tick, self.__on_tick)
        self.__close_callback_id = self.__karma.register_callback(
            KarmaStage.close, self.stop_recording)

    def __on_tick(self, snapshot: carla.WorldSnapshot, *args, **kwargs):
        """
        Callback called during Karma.on_carla_tick.
        """

        for camera, (_, container, stream) in zip(self.__mp4_cameras, self.__mp4_containers):
            img = camera.get_rgb_data()

            frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            frame.pict_type = "NONE"

            try:
                for packet in stream.encode(frame):
                    container.mux(packet)

                self.__captured_frames.append(snapshot.frame)
            except av.error.EOFError:
                pass  # recording already finished

        for camera, (_, container, tags) in zip(self.__segmentation_cameras, self.__segmentation_containers):
            labels = camera.get_segmentation_data()
            container.append(labels[..., 0])

            if camera.camera_type == 'instance_segmentation':
                tags.append(labels[..., 1:2])

        for camera, (_, container) in zip(self.__dvs_cameras, self.__dvs_containers):
            dvs_data = camera.get_dvs_data()
            container.append(dvs_data)

    def stop_recording(self):
        """
        Finishes recording stream for all cameras.
        """
        total_clips = len(self.__mp4_containers) + \
            len(self.__segmentation_containers) + len(self.__dvs_containers)
        pbar = tqdm(total=total_clips, desc='Saving', leave=False)

        for (_, container, stream) in self.__mp4_containers:
            # Flush stream
            for packet in stream.encode():
                container.mux(packet)

            container.close()
            pbar.update(1)

        for camera, (name, container, tags) in zip(self.__segmentation_cameras, self.__segmentation_containers):
            labels = np.stack(container, axis=0)

            if len(tags) > 0:
                instances = np.stack(tags, axis=0).view(dtype=np.uint16)
                # TODO: save instance tags for interesting objects

            self.__segmentation_renderer.image_size = camera.image_size
            img_sequence = self.__segmentation_renderer.render_clip(labels)
            self.__segmentation_renderer.save(
                img_sequence, name, self.__outputs_dir)
            pbar.update(1)

        for camera, (name, container) in zip(self.__dvs_cameras, self.__dvs_containers):
            self.__dvs_renderer.image_size = camera.image_size
            img_sequence = self.__dvs_renderer.render_clip(container)
            self.__dvs_renderer.save(img_sequence, name, self.__outputs_dir)
            pbar.update(1)

        pbar.close()

        if self.__tick_callback_id is not None:
            self.__karma.unregister_callback(self.__tick_callback_id)

        if self.__close_callback_id is not None:
            self.__karma.unregister_callback(self.__close_callback_id)

        captured = self.__captured_frames

        self.__mp4_containers = []
        self.__segmentation_containers = []
        self.__dvs_containers = []
        self.__captured_frames = None

        return captured

    def get_streamed_cameras(self) -> List[Camera]:
        """
        Returns a list of cameras that are being streamed.
        """
        return list(self.__streamed_cameras.values())

    def get_synchronized_cameras(self) -> List[List[Camera]]:
        """
        Returns a list of cameras that are synchronized.
        If there is no synchronization, returns empty list.
        """
        return [
            [self.__streamed_cameras[camera_id] for camera_id in sync_group]
            for sync_group in self.__synchronized_cameras
        ]
