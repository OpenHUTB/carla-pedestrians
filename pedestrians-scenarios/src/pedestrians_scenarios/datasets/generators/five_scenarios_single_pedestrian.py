from typing import Iterable, List, Tuple

from pedestrians_scenarios.karma.karma_data_provider import KarmaDataProvider
from pedestrians_scenarios.pedestrian_controls.basic_pedestrian_control import BasicPedestrianControl
from .batch_generator import BatchGenerator, PedestrianProfile
from .generator import Generator
from pedestrians_scenarios.karma.walker import Walker
import carla


class FiveScenariosSinglePedestrianBatch(BatchGenerator):
    """
    Creates dataset with randomized pedestrians crossing (or not) the street.
    Pedestrians are controlled by BasicPedestrianControl.
    Each clip should contain a single pedestrian crossing the street,
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
        pedestrian = pedestrians[0]

        pedNextPos, roadpos2 = self.get_road_parallel_path(pedestrian, waypoint)
        try:
            previous_road_position = KarmaDataProvider.get_closest_driving_lane_waypoint(
                pedNextPos.location).previous(3)[0]
        except IndexError:
            previous_road_position = roadpos2

        camera_look_at: List[carla.Transform] = [
            previous_road_position.transform for _ in camera_distances
        ]

        return camera_look_at

    def get_clip_pedestrians_control(self, clip_idx: int, pedestrians: Iterable[Walker], profiles: Iterable[PedestrianProfile], camera_look_at: Iterable[carla.Transform]) -> Iterable[BasicPedestrianControl]:
        """
        Get the pedestrians controls for a single clip.
        """

        controllers = []
        for pedestrian, profile in zip(pedestrians, profiles):
            waypoint = KarmaDataProvider.get_shifted_driving_lane_waypoint(
                pedestrian.get_transform().location,
                waypoint_jitter_scale=self._waypoint_jitter_scale
            )
            controller = BasicPedestrianControl(pedestrian)
            controller.update_target_speed(KarmaDataProvider.get_rng().normal(
                profile.crossing_speed.mean, profile.crossing_speed.std))

            transforms_path, lane_waypoint_idx = self.generate_path(
                pedestrian, waypoint)
            self._rotate_pedestrian_towards_location(
                pedestrian, transforms_path[0].location)

            controller.update_waypoints(transforms_path)
            controller.set_lane_waypoint_idx(lane_waypoint_idx)

            controllers.append(controller)

        return controllers

    def get_road_parallel_path(self, pedestrian, waypoint, max_distance=5.0):
        spawn_point = pedestrian.get_transform()

        try:
            lane_waypoint = KarmaDataProvider.get_closest_driving_lane_waypoint(
                waypoint.transform.location
            )

            if KarmaDataProvider.get_rng().randn() < 0:
                next_waypoint = lane_waypoint.next(max_distance)[0]
            else:
                next_waypoint = lane_waypoint.previous(max_distance)[0]

            direction_unit = next_waypoint.transform.location - waypoint.transform.location
            direction_unit.z = 0  # ignore height
            direction_unit = direction_unit.make_unit_vector()

            parallel_location_shift = direction_unit * \
                max_distance * KarmaDataProvider.get_rng().uniform()

            parallel_waypoint = carla.Transform(
                location=(spawn_point.location + parallel_location_shift),
            )

            return [parallel_waypoint, next_waypoint]

        except IndexError:

            return [spawn_point, waypoint]

    def generate_path(self, pedestrian: Walker, waypoint: carla.Waypoint) -> Tuple[List[carla.Transform], int]:
        spawn_point = pedestrian.get_transform()
        pr = KarmaDataProvider.get_rng().uniform()
        pr_factor = 0.5

        if pr < 0.2 * pr_factor:
            # Case 0: Pedestrian directly wants to cross the street:
            path = [waypoint.transform]
            laneWaypointPos = 0

        elif pr >= 0.2 * pr_factor and pr < 0.25 * pr_factor:
            # Case 1: Pedestrian starts crossing the street and then regrets and goes back again:
            path = [waypoint.transform, spawn_point]
            laneWaypointPos = 0

        elif pr >= 0.25 * pr_factor and pr < 0.75 * pr_factor:
            # Case 2: Pedestrian walks to a point in the path and then decides crossing the street:
            pedNextPos, roadpos2 = self.get_road_parallel_path(pedestrian, waypoint)
            nextWaypoint = roadpos2 if KarmaDataProvider.get_rng().uniform(
            ) < 0.2 else KarmaDataProvider.get_closest_driving_lane_waypoint(pedNextPos.location).transform

            path = [pedNextPos, nextWaypoint]
            laneWaypointPos = 1

        elif pr >= 0.75 * pr_factor and pr < 0.8 * pr_factor:
            # Case 3: Pedestrian walks to a point in the path, then decides crossing the street, and finally regrets and goes back:
            pedNextPos, roadpos2 = self.get_road_parallel_path(pedestrian, waypoint)
            nextWaypoint = roadpos2 if KarmaDataProvider.get_rng().uniform(
            ) < 0.2 else KarmaDataProvider.get_closest_driving_lane_waypoint(pedNextPos.location).transform

            path = [pedNextPos, nextWaypoint, pedNextPos]
            laneWaypointPos = 1

        elif pr >= 0.8 * pr_factor:
            # Case 4: Pedestrian walks to a point in the path and never decides to cross the street:
            pedNextPos, roadpos2 = self.get_road_parallel_path(pedestrian, waypoint)

            path = [pedNextPos]
            laneWaypointPos = -1

        return [path, laneWaypointPos]


class FiveScenariosSinglePedestrian(Generator):
    batch_generator = FiveScenariosSinglePedestrianBatch
