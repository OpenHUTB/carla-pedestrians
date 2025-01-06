import os
import warnings
from collections import OrderedDict
from typing import Tuple

import cameratransform as ct
import numpy as np
from pedestrians_video_2_carla.carla_utils.setup import get_camera_transform


try:
    import carla
except (ImportError, ModuleNotFoundError) as e:
    import pedestrians_video_2_carla.carla_utils.mock_carla as carla
    warnings.warn("Using mock carla.", category=ImportWarning)


class RGBCameraMock(object):
    """
    模拟默认的 CARLA 相机。
    """

    def __init__(self, pedestrian: 'ControlledPedestrian' = None, x: int = 800, y: int = 600, **kwargs):
        super().__init__()

        self.attributes = {
            'image_size_x': str(x),
            'image_size_y': str(y),
            'fov': '90.0',
            'lens_x_size': '0.08',
            'lens_y_size': '0.08'
        }
        if pedestrian is not None:
            self._transform = get_camera_transform(pedestrian, **kwargs)
        else:
            self._transform = carla.Transform()

    def get_transform(self):
        return self._transform


class PoseProjection(object):
    def __init__(self, pedestrian: 'ControlledPedestrian', camera_rgb: 'carla.Sensor' = None, *args, **kwargs) -> None:
        super().__init__()

        self._pedestrian = pedestrian

        if camera_rgb is None:
            camera_rgb = RGBCameraMock(pedestrian)

        self._image_size = (
            int(camera_rgb.attributes['image_size_x']),
            int(camera_rgb.attributes['image_size_y'])
        )
        self.camera = self._setup_camera(camera_rgb)

    @property
    def image_size(self) -> Tuple:
        """
        返回投影图像大小。

        :return: (width, height)
        :rtype: Tuple
        """
        return self._image_size

    def _calculate_distance_and_elevation(self, camera_rgb: 'carla.Sensor') -> Tuple[float, float]:
        distance = camera_rgb.get_transform().location.x - \
            self._pedestrian.world_transform.location.x + \
            self._pedestrian.spawn_shift.x
        elevation = camera_rgb.get_transform().location.z - \
            self._pedestrian.world_transform.location.z + \
            self._pedestrian.spawn_shift.z

        return distance, elevation

    def _setup_camera(self, camera_rgb: 'carla.Sensor'):
        # 基本变换是在 UE 世界坐标中，其轴是不同的，此外，我们需要纠正生成偏移错误
        distance, elevation = self._calculate_distance_and_elevation(camera_rgb)

        camera_ct = ct.Camera(
            ct.RectilinearProjection(
                image_width_px=self._image_size[0],
                image_height_px=self._image_size[1],
                view_x_deg=float(camera_rgb.attributes['fov']),
                sensor_width_mm=float(camera_rgb.attributes['lens_x_size'])*1000,
                sensor_height_mm=float(camera_rgb.attributes['lens_y_size'])*1000
            ),
            ct.SpatialOrientation(
                pos_y_m=distance,
                elevation_m=elevation,
                heading_deg=180,
                tilt_deg=90
            )
        )

        return camera_ct

    def update_camera(self, camera_position: Tuple[float, float, float]):
        """
        更新相机位置。

        :param camera_position: new camera position (x, y, z)
        :type camera_position: List[Tuple[float, float, float]]
        """

        (x, y, z) = camera_position

        self.camera.orientation = ct.SpatialOrientation(
            pos_x_m=y,
            pos_y_m=x,
            elevation_m=z
        )

    def current_pose_to_points(self):
        # 从 UE 世界坐标切换，其轴不同
        root_transform = carla.Transform(location=carla.Location(
            x=self._pedestrian.transform.location.y,
            y=self._pedestrian.transform.location.x,
            z=self._pedestrian.transform.location.z
        ), rotation=carla.Rotation(
            yaw=-self._pedestrian.transform.rotation.yaw
        ))

        relativeBones = [
            root_transform.transform(carla.Location(
                x=-bone.location.x,
                y=bone.location.y,
                z=bone.location.z
            ))
            for bone in self._pedestrian.current_pose.absolute.values()
        ]
        return self.camera.imageFromSpace([
            (bone.x, bone.y, bone.z)
            for bone in relativeBones
        ], hide_backpoints=False)
