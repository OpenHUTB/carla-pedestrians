#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Corporation.
# authors: German Ros (german.ros@intel.com), Felipe Codevilla (felipe.alcm@gmail.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Challenge 评估器的路线

评估 CARLA Autonomous Driving challenge 自动驾驶代理的临时代码
"""
from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
from distutils.version import LooseVersion
import importlib
import os
import pkg_resources
import sys
import carla
import signal
import time
import json
import random
import numpy as np
import subprocess

import sys
sys.path.append(r"/home/d/workspace/CILv2_multiview/scenario_runner")
sys.path.append(r"/home/d/workspace/CILv2_multiview/CARLA_0.9.13/PythonAPI/carla")
sys.path.append(r"/home/d/workspace/CILv2_multiview/run_CARLA_driving/driving/autoagents")
sys.path.append(r"/home/d/workspace/CILv2_multiview/run_CARLA_driving")
# /home/d/workspace/CILv2_multiview/run_CARLA_driving/driving/__init__.py

# --debug=0 --scenarios=/home/d/workspace/CILv2_multiview/run_CARLA_driving/data/leaderboard/leaderboard_Town05.json --routes=/home/d/workspace/CILv2_multiview/run_CARLA_driving/data/leaderboard --repetitions=1 --resume=True --track=SENSORS --agent=${DRIVING_TEST_ROOT}/driving/autoagents/CILv2_agent.py --checkpoint=/home/d/workspace/CILv2_multiview/run_CARLA_driving/results/leaderboard   --agent-config=/home/d/workspace/CILv2_multiview/_results/_results/Ours/Town12346_5/config40.json  --docker=carlasim/carla:0.9.13 --gpus=0     --fps=20  --PedestriansSeed=0 --trafficManagerSeed=0 --save-driving-vision

from srunner.scenariomanager.carla_data_provider import *
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from driving.scenarios.scenario_manager import ScenarioManager
from driving.scenarios.route_scenario import RouteScenario
from driving.envs.sensor_interface import SensorConfigurationInvalid
from driving.autoagents.agent_wrapper import AgentWrapper, AgentError
from driving.utils.statistics_manager import StatisticsManager
from driving.utils.route_indexer import RouteIndexer
from driving.utils.server_manager import ServerManagerDocker, find_free_port

sensors_to_icons = {
    'sensor.camera.rgb':        'carla_camera',
    'sensor.camera.depth':      'carla_depth',
    'sensor.camera.semantic_segmentation':  'carla_ss',
    'sensor.lidar.ray_cast':    'carla_lidar',
    'sensor.other.radar':       'carla_radar',
    'sensor.other.gnss':        'carla_gnss',
    'sensor.other.imu':         'carla_imu',
    'sensor.opendrive_map':     'carla_opendrive_map',
    'sensor.speedometer':       'carla_speedometer',
    'sensor.can_bus':           'carla_canbus'
}


# 将系统中所有地方都设置为相同的随机数种子
def seed_everything(seed=0):
    random.seed(seed)
    np.random.seed(seed)


class Evaluator(object):

    """
    TODO: document me!
    """

    ego_vehicles = []

    # 可调制的参数
    client_timeout = 10.0  # 客户端超时时间（以秒为单位）
    wait_for_world = 20.0  # 等待时间的时间（以秒为单位）

    def __init__(self, args, statistics_manager, ServerDocker=None):
        """
        设置 CARLA 客户端和世界
        设置场景管理器 ScenarioManager
        """
        self.ServerDocker = ServerDocker
        self.statistics_manager = statistics_manager
        self.sensors = None
        self.sensor_icons = []
        self._vehicle_lights = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam

        self.frame_rate = float(args.fps)  # in Hz

        # 首先，我们需要创建向模拟器发送请求的客户端。
        # 这里我们假设模拟器接受本地机器 localhost 的 2000 端口的请求。
        # 如果使用 docker，会随机分配未使用的端口
        if self.ServerDocker is not None:
            args.port = find_free_port()
            self.ServerDocker.reset(args.host, args.port)
            args.trafficManagerPort = find_free_port()

        self.client = carla.Client(args.host, int(args.port))
        if args.timeout:
            self.client_timeout = float(args.timeout)
        self.client.set_timeout(self.client_timeout)

        self.traffic_manager = self.client.get_trafficmanager(int(args.trafficManagerPort))

        dist = pkg_resources.get_distribution("carla")
        if dist.version != 'leaderboard':
            if LooseVersion(dist.version) < LooseVersion('0.9.10'):
                raise ImportError("CARLA version 0.9.10.1 or newer required. CARLA version found: {}".format(dist))

        # 加载代理
        module_name = os.path.basename(args.agent).split('.')[0]
        sys.path.insert(0, os.path.dirname(args.agent))
        self.module_agent = importlib.import_module(module_name)

        # 创建场景管理器 ScenarioManager
        self.manager = ScenarioManager(args.timeout, args.debug > 1)

        # 为了总结的目的进行时间控制
        self._start_time = GameTime.get_time()
        self._end_time = None

        # 创建代理计时器
        self._agent_watchdog = Watchdog(int(float(args.timeout)))
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """
        当接收到中断信号时，就终止场景的节拍
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError("Timeout: Agent took too long to setup")
        elif self.manager:
            self.manager.signal_handler(signum, frame)

    def __del__(self):
        """
        清除并删除参与者、场景管理器 ScenarioManager 和 CARLA 世界
        """

        self._cleanup()
        if hasattr(self, 'manager') and self.manager:
            del self.manager
        if hasattr(self, 'world') and self.world:
            del self.world
        if hasattr(self, 'client') and self.client:
            del self.client
        if hasattr(self, 'traffic_manager') and self.traffic_manager:
            del self.traffic_manager

    def _cleanup(self):
        """
        移除并销毁所有参与者
        """

        # 模拟仍然运行并处于同步模式？
        if self.manager and self.manager.get_running_status() \
                and hasattr(self, 'world') and self.world:
            # 重置为异步模式
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            self.traffic_manager.set_synchronous_mode(False)

        if self.manager:
            self.manager.cleanup()

        CarlaDataProvider.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if self._agent_watchdog:
            self._agent_watchdog.stop()

        if hasattr(self, 'agent_instance') and self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None

        if hasattr(self, 'statistics_manager') and self.statistics_manager:
            self.statistics_manager.scenario = None

    def _load_and_wait_for_world(self, args, config):
        """
        加载一个新的 CARLA 世界并为CarlaDataProvider提供数据
        """

        town = config.town
        print('  port:', args.port)
        self.world = self.client.load_world(town)
        self.world.set_weather(config.weather)
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 1.0 / self.frame_rate
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

        self.world.set_pedestrians_seed(int(args.PedestriansSeed))
        print('Set seed for pedestrians:', str(int(args.PedestriansSeed)))

        self.world.reset_all_traffic_lights()
        if hasattr(config, 'scenarios'):
            if 'background_activity' in list(config.scenarios.keys()):
                if 'cross_factor' in list(config.scenarios['background_activity'].keys()):
                    self.world.set_pedestrians_cross_factor(config.scenarios['background_activity']['cross_factor'])
                else:
                    self.world.set_pedestrians_cross_factor(1.0)

        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_traffic_manager_port(int(args.trafficManagerPort))

        self.traffic_manager.set_synchronous_mode(True)
        self.traffic_manager.set_global_distance_to_leading_vehicle(5.0)
        self.traffic_manager.set_random_device_seed(int(args.trafficManagerSeed))
        print('Set seed for traffic manager:', str(int(args.trafficManagerSeed)))
        seed_everything(seed=int(args.trafficManagerSeed))
        print('Set seed for numpy:', str(int(args.trafficManagerSeed)))
        CarlaDataProvider.set_random_state_seed(int(args.trafficManagerSeed))
        print('Set seed for random state:', str(int(args.trafficManagerSeed)))

        # 等待世界准备好
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

        if CarlaDataProvider.get_map().name.split('/')[-1] != town:
            raise Exception("The CARLA server uses the wrong map!"
                            "This scenario requires to use map {}".format(town))

    def _register_statistics(self, config, checkpoint, entry_status, crash_message=""):
        """
        计算并保存模拟统计数据
        """
        # 注册统计
        # 系统运行时间 vs 游戏运行时间 区别？
        current_stats_record = self.statistics_manager.compute_route_statistics(
            config,
            self.manager.scenario_duration_system,
            self.manager.scenario_duration_game,
            crash_message
        )

        print("\033[1m> Registering the route statistics\033[0m")
        self.statistics_manager.save_record(current_stats_record, config.index, checkpoint)
        self.statistics_manager.save_entry_status(entry_status, False, checkpoint)

    def _load_and_run_scenario(self, args, config):
        """
        加载并运行配置给出的场景。

        根据失败的代码，模拟将停止路线运行并从下一个路线继续，或报告崩溃并停止。
        """
        crash_message = ""
        entry_status = "Started"

        print("\n\033[1m========= Preparing {} (repetition {}) =========".format(config.name, config.repetition_index))
        print("> Setting up the agent\033[0m")

        # 准备路线统计数据
        self.statistics_manager.set_route(config.name, config.index)

        # 设置用户代理和计时器以避免冻结模拟
        try:
            self._agent_watchdog.start()
            agent_class_name = getattr(self.module_agent, 'get_entry_point')()

            # 用于数据收集
            if args.data_collection:
                vision_save_path = os.path.join(os.environ['DATASET_PATH'], config.package_name, config.name)
                self.agent_instance = getattr(self.module_agent, agent_class_name) \
                    (args.agent_config, save_driving_vision=vision_save_path)

            # 用于保存基准驾驶事件
            else:
                vision_save_path = os.path.join(os.environ['SENSOR_SAVE_PATH'], config.package_name,
                                                args.checkpoint.split('/')[-1].split('.')[-2], config.name,
                                                str(config.repetition_index)) if args.save_driving_vision else False
                self.agent_instance = getattr(self.module_agent, agent_class_name) \
                    (args.agent_config, save_driving_vision=vision_save_path,
                     save_driving_measurement=args.save_driving_measurement)

            config.agent = self.agent_instance

            # 检查并保存传感器
            if not self.sensors:
                self.sensors = self.agent_instance.sensors()
                track = self.agent_instance.track

                AgentWrapper.validate_sensor_configuration(self.sensors, track, args.track)

                self.sensor_icons = [sensors_to_icons[sensor['type']] for sensor in self.sensors]
                self.statistics_manager.save_sensors(self.sensor_icons, args.checkpoint)

            self._agent_watchdog.stop()

        except SensorConfigurationInvalid as e:
            # 传感器无效 -> 将执行设置为拒绝并停止
            print("\n\033[91mThe sensor's configuration used is invalid:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Agent's sensors were invalid"
            entry_status = "Rejected"
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)
            self._cleanup()
            sys.exit(-1)

        except Exception as e:
            # 代理设置失败 -> 将执行设置为拒绝并停止
            print("\n\033[91mCould not set up the required agent:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Agent couldn't be set up"
            entry_status = "Rejected"
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)
            self._cleanup()
            sys.exit(-1)

        print("\033[1m> Loading the world\033[0m")

        # 加载世界和场景
        try:
            self._load_and_wait_for_world(args, config)
            self.agent_instance.set_world(self.world)
            scenario = RouteScenario(world=self.world, config=config, debug_mode=args.debug)
            self.statistics_manager.set_scenario(scenario.scenario)

            self.agent_instance.set_ego_vehicle(scenario._ego_vehicle)

            # 夜间模式
            if config.weather.sun_altitude_angle < 0.0:
                for vehicle in scenario.ego_vehicles:
                    vehicle.set_light_state(carla.VehicleLightState(self._vehicle_lights))

            # 加载场景并运行它
            if args.record:
                self.client.start_recorder("{}/{}_rep{}.log".format(args.record, config.name, config.repetition_index))
            self.manager.load_scenario(scenario, self.agent_instance, config.repetition_index)

        except Exception as e:
            # 场景错误 -> 将执行设置为崩溃并停止
            print("\n\033[91mThe scenario could not be loaded:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Simulation crashed"
            entry_status = "Crashed"
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)

            if args.record:
                self.client.stop_recorder()

            self._cleanup()
            sys.exit(-1)

        print("\033[1m> Running the route\033[0m")

        # 运行场景
        try:
            self.manager.run_scenario()

        except AgentError as e:
            # 代理失败 -> 将执行设置为崩溃并停止
            print("\n\033[91mStopping the route, the agent has crashed:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Agent crashed"
            entry_status = "Crashed"
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)
            self._cleanup()
            sys.exit(-1)

        except Exception as e:
            print("\n\033[91mError during the simulation:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Simulation crashed"
            entry_status = "Crashed"
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)
            self._cleanup()
            sys.exit(-1)

        # 停止场景
        try:
            print("\033[1m> Stopping the route\033[0m")
            self.manager.stop_scenario()
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)

            if args.record:
                self.client.stop_recorder()

            # 移除所有参与者
            scenario.remove_all_actors()

            self._cleanup()

        except Exception as e:
            print("\n\033[91mFailed to stop the scenario, the statistics might be empty:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Simulation crashed"
            entry_status = "Crashed"
            self._register_statistics(config, args.checkpoint, entry_status, crash_message)
            self._cleanup()
            sys.exit(-1)

    def run(self, args):
        """
        运行挑战模式
        """
        route_indexer = RouteIndexer(args.routes, args.scenarios, args.repetitions)

        if args.resume:
            route_indexer.resume(args.checkpoint)
            self.statistics_manager.resume(args.checkpoint)
        else:
            self.statistics_manager.clear_record(args.checkpoint)
            route_indexer.save_state(args.checkpoint)

        while route_indexer.peek():
            # 设置
            config = route_indexer.next()
            # 运行
            self._load_and_run_scenario(args, config)

            route_indexer.save_state(args.checkpoint)

        # 保存全局统计数据
        print("\033[1m> Registering the global statistics\033[0m")
        # 计算全局的性能指标，生成论文表格中的数据
        global_stats_record = self.statistics_manager.compute_global_statistics(route_indexer.total)
        StatisticsManager.save_global_record(global_stats_record, self.sensor_icons, route_indexer.total, args.checkpoint)
        if self.ServerDocker is not None:
            self.ServerDocker.stop()


def main():
    description = "CARLA Evaluation: evaluate your Agent in CARLA simulator\n"

    # 常规参数
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host', default='localhost',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default='2000', help='TCP port to listen to (default: 2000)')
    parser.add_argument('--trafficManagerPort', default='8000',
                        help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--trafficManagerSeed', default='0',
                        help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--PedestriansSeed', default='0',
                        help='Seed used by the Pedestrians setting (default: 0)')
    parser.add_argument('--debug', type=int, help='Run with debug output', default=0)
    parser.add_argument('--record', type=str, default='',
                        help='Use CARLA recording feature to create a recording of the scenario')
    parser.add_argument('--timeout', default="180.0",
                        help='Set the CARLA client timeout value in seconds')

    # 模拟设置
    # 要执行路线的名称。指向要执行的route_xml_file。
    parser.add_argument('--routes',
                        help='Name of the route to be executed. Point to the route_xml_file to be executed.',
                        required=True)
    # 与路线混合的场景注释文件的名称。
    parser.add_argument('--scenarios',
                        help='Name of the scenario annotation file to be mixed with the route.', required=True)
    # 每条路线的重复次数。
    parser.add_argument('--repetitions', type=int, default=1, help='Number of repetitions per route.')

    # 代理相关选项
    parser.add_argument("-a", "--agent", type=str, help="Path to Agent's py file to evaluate", required=True)
    parser.add_argument("--agent-config", type=str, help="Path to Agent's configuration file", default="")

    parser.add_argument("--track", type=str, default='SENSORS', help="Participation track: SENSORS, MAP")
    parser.add_argument('--resume', type=bool, default=False, help='Resume execution from last checkpoint?')
    parser.add_argument("--checkpoint", type=str, default='./simulation_results.json',
                        help="Path to checkpoint used for saving statistics and resuming")

    parser.add_argument('--docker', type=str, default='',
                        help='Use docker to run CARLA off-screen, this is typically for running CARLA on server')
    parser.add_argument('--gpus', nargs='+', dest='gpus', type=str, default=0,
                        help='The GPUs used for running the agent model. '
                             'The firtst one will be used for running docker')

    parser.add_argument('--save-driving-vision', action="store_true", help=' to save the driving visualization')
    parser.add_argument('--save-driving-measurement', action="store_true", help=' to save the driving measurements')
    parser.add_argument('--data-collection', action="store_true", help=' to collect dataset')
    parser.add_argument('--fps', default=10, help='The frame rate of CARLA world')

    arguments = parser.parse_args()

    gpus = []
    if arguments.gpus:
        # 检查传递的 GPU 向量是否有效。
        for gpu in arguments.gpus[0].split(','):
            try:
                int(gpu)
                gpus.append(int(gpu))
            except ValueError:  # 重新抛出有意义的错误。
                raise ValueError("GPU is not a valid int number")
        os.environ["CUDA_VISIBLE_DEVICES"] = ','.join(arguments.gpus)  # 这必须先于整个执行
        arguments.gpus = gpus
    else:
        raise ValueError('You need to define the ids of GPU you want to use by adding: --gpus')

    if arguments.save_driving_vision or arguments.save_driving_measurement:
        if not os.environ['SENSOR_SAVE_PATH']:
            raise RuntimeError('environemnt argument SENSOR_SAVE_PATH need to be setup for saving data')

    if not os.path.exists(os.path.join(arguments.checkpoint, arguments.scenarios.split('/')[-1].split('.')[-2])):
        os.makedirs(os.path.join(arguments.checkpoint, arguments.scenarios.split('/')[-1].split('.')[-2]))

    f = open(arguments.agent_config, 'r')
    _json = json.loads(f.read())
    arguments.checkpoint = '/'.join([arguments.checkpoint, arguments.scenarios.split('/')[-1].split('.')[-2], '_'.join([arguments.agent_config.split('/')[-3],
                                                                                                                        arguments.agent_config.split('/')[-2],
                                                                                                                        str(_json['checkpoint']), 'Seed'+str(arguments.PedestriansSeed),
                                                                                                                        arguments.fps+'FPS.json'])])

    statistics_manager = StatisticsManager()

    ServerDocker = None
    if arguments.docker:
        docker_params = {'docker_name': arguments.docker, 'gpu': arguments.gpus[0], 'quality_level': 'Epic'}
        ServerDocker = ServerManagerDocker(docker_params)

    try:
        evaluator = Evaluator(arguments, statistics_manager, ServerDocker)
        evaluator.run(arguments)
    except Exception as e:
        traceback.print_exc()
        sys.exit(-1)
    finally:
        del evaluator


if __name__ == '__main__':
    main()
    print('Finished all episode. Goodbye!')
