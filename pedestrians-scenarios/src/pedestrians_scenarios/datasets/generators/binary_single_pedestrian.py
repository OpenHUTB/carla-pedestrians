from typing import Iterable, List, Tuple

from pedestrians_scenarios.karma.karma_data_provider import KarmaDataProvider
from pedestrians_scenarios.karma.utils.deepcopy import deepcopy_rotation, deepcopy_transform
from pedestrians_scenarios.pedestrian_controls.basic_pedestrian_control import BasicPedestrianControl
from .batch_generator import BatchGenerator, PedestrianProfile
from .generator import Generator
from pedestrians_scenarios.karma.walker import Walker
import carla


class BinarySinglePedestrianBatch(BatchGenerator):
    """
    Creates dataset with randomized pedestrians crossing (or not) the street.
    Pedestrians are controlled by BasicPedestrianControl.
    Each clip should contain a single pedestrian crossing/not-crossing the street,
    HOWEVER, due to concurrent data generation, the pedestrians other
    than primary one may be visible in the clip.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def get_clip_camera_look_at(self, clip_idx: int, pedestrians: Iterable[Walker], camera_distances: Iterable[float]) -> Iterable[carla.Transform]:
        """
        Get the camera look at points for a single clip.
        In BasicSinglePedestrianCrossing there can be only one pedestrian in the clip,
        but potentially there can be more than one camera.
        All cameras are looking at the same point.
        """
        waypoints: List[carla.Waypoint] = [
            KarmaDataProvider.get_shifted_driving_lane_waypoint(
                pedestrian.get_transform().location,
                waypoint_jitter_scale=self._waypoint_jitter_scale)
            for pedestrian in pedestrians
        ]

        assert len(waypoints) < 2, "Only one pedestrian per clip is supported"

        if len(waypoints) == 0:
            return []

        waypoint = waypoints[0]
        camera_look_at: List[carla.Transform] = [
            waypoint.transform for _ in camera_distances
        ]

        return camera_look_at

    def get_clip_pedestrians_control(self, clip_idx: int, pedestrians: Iterable[Walker], profiles: Iterable[PedestrianProfile], camera_look_at: Iterable[carla.Transform]) -> Iterable[BasicPedestrianControl]:
        """
        Get the pedestrians controls for a single clip.
        """

        controllers = []
        for pedestrian, profile, look_at in zip(pedestrians, profiles, camera_look_at):
            controller = BasicPedestrianControl(pedestrian)
            controller.update_target_speed(KarmaDataProvider.get_rng().normal(
                profile.crossing_speed.mean, profile.crossing_speed.std))

            pedestrian_path = self.generate_path(pedestrian, look_at)
            self._rotate_pedestrian_towards_location(
                pedestrian, pedestrian_path[0].location)

            controller.update_waypoints(pedestrian_path)

            controllers.append(controller)

        return controllers

    def get_point_along_road(self, pedestrian: Walker, camera_look_at: carla.Transform, max_distance=5.0):
        spawn_point = pedestrian.get_transform()

        try:
            lane_waypoint = KarmaDataProvider.get_closest_driving_lane_waypoint(
                camera_look_at.location
            )

            distance = KarmaDataProvider.get_rng().uniform() * max_distance
            if KarmaDataProvider.get_rng().uniform() < 0.5:
                next_waypoint = lane_waypoint.next(distance)[0]
            else:
                next_waypoint = lane_waypoint.previous(distance)[0]

            direction_unit = next_waypoint.transform.location - camera_look_at.location
            direction_unit.z = 0  # ignore height
            direction_unit = direction_unit.make_unit_vector()

            next_point = carla.Transform(
                location=(spawn_point.location + direction_unit * distance),
            )

            return next_point

        except IndexError:

            return spawn_point

    def generate_path(self, pedestrian: Walker, camera_look_at: carla.Transform) -> Tuple[List[carla.Transform], int]:
        pr = KarmaDataProvider.get_rng().uniform()

        if pr < 0.5:
            path = [camera_look_at]
        else:
            path = [self.get_point_along_road(pedestrian, camera_look_at)]

        return path


class BinarySinglePedestrian(Generator):
    batch_generator = BinarySinglePedestrianBatch
