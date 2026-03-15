import copy
import logging
# from turtle import speed
import numpy as np
from roach_agents.cilrs.cilrs_agent import CilrsAgent
# from srunner.autoagents.autonomous_agent import AutonomousAgent
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track


from srunner.tests.carla_mocks import carla
import carla_gym.utils.transforms as trans_utils
import carla_gym.core.task_actor.common.navigation.route_manipulation as gps_util
from carla_gym.core.task_actor.common.navigation.map_utils import RoadOption


def get_entry_point():
    return 'CarlaRoachAgent'


class CarlaRoachAgent(CilrsAgent, AutonomousAgent):
    """
    CarlaRoachAgent is an autonomous agent that uses CILRS to drive a Roach.
    """
    def __init__(self, path_to_conf_file='config_agent.yaml'):
        self._logger = logging.getLogger(__name__)
        self._render_dict = None
        self._global_plan_gps = None
        AutonomousAgent.__init__(self, path_to_conf_file)
   
    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]
        """


        sensors = [
            {'type': 'sensor.camera.rgb', 'x': -1.5, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 900, 'height': 256, 'fov': 100, 'id': 'central_rgb'},
            {'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'id': 'gnss'},
            {'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'id': 'imu'},
            {'type': 'sensor.speedometer', 'reading_frequency': 25, 'id': 'forward_speed',}

        ]
        return sensors

    def _preprocess_rgb(self, carla_image):
        # np_img = np.frombuffer(carla_image.raw_data, dtype=np.dtype("uint8"))

        np_img = copy.deepcopy(carla_image)
        # print(np_img.shape)

        # np_img = np.reshape(np_img, (carla_image.height, carla_image.width, 4))
        np_img = np_img[:, :, :3]
        np_img = np_img[:, :, ::-1]
        return np_img

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        self._global_plan_gps = global_plan_gps
        return super().set_global_plan(global_plan_gps, global_plan_world_coord)

    def _get_gps_point_and_road_option(self, imu_data=None, gnss_data=None):
        self._idx = -1
        global_plan_gps = self._global_plan_gps

        next_gps, _ = global_plan_gps[self._idx+1]
        next_vec_in_global = gps_util.gps_to_location(tuple(next_gps.values())) - gps_util.gps_to_location(gnss_data)
        compass = 0.0 if np.isnan(imu_data[-1]) else imu_data[-1]
        ref_rot_in_global = carla.Rotation(yaw=np.rad2deg(compass)-90.0)
        loc_in_ev = trans_utils.vec_global_to_ref(next_vec_in_global, ref_rot_in_global)

        if np.sqrt(loc_in_ev.x**2+loc_in_ev.y**2) < 12.0 and loc_in_ev.x < 0.0:
            self._idx += 1

        self._idx = min(self._idx, len(global_plan_gps)-2)

        _, road_option_0 = global_plan_gps[max(0, self._idx)]
        gps_point, road_option_1 = global_plan_gps[self._idx+1]

        if (road_option_0 in [RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT]) \
                and (road_option_1 not in [RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT]):
            road_option = road_option_1
        else:
            road_option = road_option_0

        return tuple(gps_point.values()), road_option

    def run_step(self, input_data, timestamp):

        gnss_data = input_data['gnss'][1]
        imu_data = input_data['imu'][1]

        gps_point, road_option = self._get_gps_point_and_road_option(imu_data, gnss_data)

        gnss_data = {
            'gnss': gnss_data,
            'imu': imu_data,
            'target_gps': np.array(gps_point, dtype=np.float32),
            'command': np.array([road_option.value], dtype=np.int8)
            }

        speed_data = {
            'forward_speed': np.array([ input_data['forward_speed'][1]['speed'] ], dtype=np.float32)
        }
    
        converted_data = {
            'central_rgb':  {'frame': input_data['central_rgb'][0], 'data': self._preprocess_rgb(input_data['central_rgb'][1])},
            'gnss': gnss_data,
            'speed': speed_data
        }
        return super().run_step(converted_data, timestamp)