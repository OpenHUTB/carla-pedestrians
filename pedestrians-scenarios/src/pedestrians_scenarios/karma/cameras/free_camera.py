from typing import Tuple, Union

import carla

from ..actor import Actor
from ..karma_data_provider import KarmaDataProvider
from ..utils.rotations import look_at as carla_look_at
from .camera import Camera


class FreeCamera(Actor, Camera):
    def __init__(self,
                 look_at: carla.Transform,
                 distance: Tuple[float] = (-10.0, 0.0, 0.0),
                 image_size: Tuple[int] = (800, 600),
                 fov: float = 90.0,
                 camera_type: str = 'rgb',
                 **kwargs
                 ):
        if camera_type not in ['rgb', 'depth', 'semantic_segmentation', 'instance_segmentation', 'dvs']:
            raise ValueError(
                'Camera type must be one of "rgb", "depth", "semantic_segmentation", "instance_segmentation" or "dvs".')

        camera_location = carla.Location(look_at.transform(carla.Location(*distance)))
        camera_transform = carla_look_at(look_at, camera_location)

        blueprint_library = KarmaDataProvider.get_blueprint_library()
        camera_bp = blueprint_library.find(f'sensor.camera.{camera_type}')

        camera_bp.set_attribute('image_size_x', str(image_size[0]))
        camera_bp.set_attribute('image_size_y', str(image_size[1]))
        camera_bp.set_attribute('fov', str(fov))

        camera = KarmaDataProvider.request_new_sensor(
            camera_bp, camera_transform, **kwargs)

        Actor.__init__(self, actor=camera)
        Camera.__init__(self, sensor=camera, **kwargs)

    def get_transform(self) -> carla.Transform:
        return Actor.get_transform(self)
