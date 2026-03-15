import carla
import copy
import logging
import numpy as np
from roach_agents.rl_birdview.rl_birdview_agent import RlBirdviewAgent
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

from srunner.tests.carla_mocks import carla
import carla_gym.utils.transforms as trans_utils
import carla_gym.core.task_actor.common.navigation.route_manipulation as gps_util
from carla_gym.core.task_actor.common.navigation.map_utils import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from carla_gym.core.task_actor.common.task_vehicle import TaskVehicle
from carla_gym.core.obs_manager.birdview.chauffeurnet import ObsManager

def get_entry_point():
    return 'LeaderboardPPOAgent'

class LeaderboardPPOAgent(RlBirdviewAgent, AutonomousAgent):
    def __init__(self, path_to_conf_file='config_agent.yaml'):
        self._logger = logging.getLogger(__name__)
        self._render_dict = None
        self._global_plan_gps = None
        RlBirdviewAgent.__init__(self, path_to_conf_file)
        AutonomousAgent.__init__(self, path_to_conf_file)

        self._idx = -1
        self._agent = None
        self.ego_vehicle = None
        self.obs_manager = ObsManager(self._obs_configs['birdview'])
   
    def sensors(self):
        sensors = [
            {'type': 'sensor.camera.rgb', 'x': -1.5, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 900, 'height': 256, 'fov': 100, 'id': 'central_rgb'},
            {'type': 'sensor.other.gnss', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'id': 'gnss'},
            {'type': 'sensor.other.imu', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'id': 'imu'},
            {'type': 'sensor.speedometer', 'reading_frequency': 25, 'id': 'forward_speed',}

        ]
        return sensors

    def _preprocess_rgb(self, carla_image):
        np_img = copy.deepcopy(carla_image)

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

    def set_ego_vehicle(self, ego_vehicle):
        route = [carla.Transform(location=gps_util.gps_to_location(tuple(next_gps.values())), rotation=carla.Rotation(pitch=5)) for next_gps, _ in self._global_plan_gps]
        
        self.ego_vehicle = TaskVehicle(ego_vehicle, route, None, False)
        self.obs_manager.attach_ego_vehicle(self.ego_vehicle)

    def run_step(self, input_data, timestamp):
        if self.ego_vehicle is None:
            # Search for the ego actor
            for actor in CarlaDataProvider.get_world().get_actors():
                if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                    self.set_ego_vehicle(actor)
                    break

        input_data = copy.deepcopy(input_data)

        vehicle_control = self.ego_vehicle.vehicle.get_control()
        input_data['control'] = {}
        input_data['control']['throttle'] = np.array([vehicle_control.throttle], dtype=np.float32)
        input_data['control']['steer'] = np.array([vehicle_control.steer], dtype=np.float32)
        input_data['control']['brake'] = np.array([vehicle_control.brake], dtype=np.float32)
        input_data['control']['gear'] = np.array([vehicle_control.gear], dtype=np.float32)

        obs_dict = self.obs_manager.get_observation()
        input_data['birdview'] = obs_dict

        vehicle_transform = self.ego_vehicle.vehicle.get_transform()
        acc = trans_utils.vec_global_to_ref(self.ego_vehicle.vehicle.get_acceleration(), vehicle_transform.rotation)
        vel = trans_utils.vec_global_to_ref(self.ego_vehicle.vehicle.get_velocity(), vehicle_transform.rotation)
        ang = self.ego_vehicle.vehicle.get_angular_velocity()

        input_data['velocity'] = {}
        input_data['velocity']['acc_xy'] = np.array([acc.x, acc.y], dtype=np.float32)
        input_data['velocity']['vel_xy'] = np.array([vel.x, vel.y], dtype=np.float32)
        input_data['velocity']['vel_ang_z'] = np.array([ang.z], dtype=np.float32)

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
    
        input_data['central_rgb'] =  {'frame': input_data['central_rgb'][0], 'data': self._preprocess_rgb(input_data['central_rgb'][1])}
        input_data['gnss'] = gnss_data
        input_data['speed'] = speed_data
        input_data['route_plan'] = self._global_plan_gps
           
        return super().run_step(input_data, timestamp)