from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.actorcontrols.pedestrian_control import PedestrianControl

from pedestrians_scenarios.karma.karma_data_provider import KarmaDataProvider


class BasicPedestrianControl(PedestrianControl):
    """
    This is a basic PedestrianControl from ScenarioRunner,
    just with karma.Walker allowed.
    """

    def __init__(self, actor, args=None):
        if not hasattr(actor, 'get_control') or not hasattr(actor, 'apply_control'):
            raise RuntimeError(
                "PedestrianControl: The to be controlled actor does not have the required methods.")

        # we need to skip the PedestrianControl init,
        # instead we call the BasicControl one directly
        BasicControl.__init__(self, actor)

        self._road_waypoint_idx = None
        self._reached_waypoints = 0

    def run_step(self, *args, **kwargs):
        old_waypoints = self._waypoints

        out = super().run_step()

        road_waypoint = KarmaDataProvider.get_map().get_waypoint(
            self._actor.get_location(), project_to_road=False)
        self._actor.is_crossing = road_waypoint is not None

        if len(old_waypoints) != len(self._waypoints):
            self._reached_waypoints = self._reached_waypoints + 1

        return out

    def check_reached_first_waypoint(self):
        # if walker didn't reach the first waypoint when other code asks,
        # it is probably stuck somewhere, and therefore useless
        return self._reached_waypoints > 0

    # TODO: remove when FiveScenarios is fixed
    def set_lane_waypoint_idx(self, waypoint_pos):
        self._road_waypoint_idx = waypoint_pos
