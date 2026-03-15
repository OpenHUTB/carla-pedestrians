#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
用于跟踪和检查所用传感器的自主代理的包装器
"""

from __future__ import print_function
import math

import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from driving.envs.sensor_interface import CallBack, OpenDriveMapReader, SpeedometerReader, SensorConfigurationInvalid, CanbusReader
from driving.autoagents.autonomous_agent import Track

MAX_ALLOWED_RADIUS_SENSOR = 100.0

SENSORS_LIMITS = {
    'sensor.camera.rgb': 7,
    'sensor.camera.depth': 7,
    'sensor.lidar.ray_cast': 1,
    'sensor.other.radar': 2,
    'sensor.other.gnss': 1,
    'sensor.other.imu': 1,
    'sensor.opendrive_map': 1,
    'sensor.speedometer': 1
}


class AgentError(Exception):
    """
    当代理在模拟过程中返回错误时抛出的异常
    """

    def __init__(self, message):
        super(AgentError, self).__init__(message)


class AgentWrapper(object):

    """
    用于跟踪和检查所用传感器的自主代理的包装器
    """

    allowed_sensors = [
        'sensor.opendrive_map',
        'sensor.camera.semantic_segmentation',
        'sensor.speedometer',
        'sensor.camera.rgb',
        'sensor.camera.depth',
        'sensor.camera',
        'sensor.lidar.ray_cast',
        'sensor.other.radar',
        'sensor.other.gnss',
        'sensor.other.imu',
        'sensor.can_bus'
    ]

    _agent = None
    _sensors_list = []

    def __init__(self, agent):
        """
        设置自主代理
        """
        self._agent = agent

    def __call__(self, timestamp):
        """
        Pass the call directly to the agent
        """
        return self._agent(timestamp)

    def setup_sensors(self, vehicle, other_actors_dict=None, route=None):
        """
        创建用户定义的传感器并将其连接到自主车辆
        :param vehicle: 自主车辆
        :return:
        """
        bp_library = CarlaDataProvider.get_world().get_blueprint_library()
        for sensor_spec in self._agent.sensors():
            # 这些是伪传感器（未生成）
            if sensor_spec['type'].startswith('sensor.opendrive_map'):
                # HDMap 伪传感器直接在这里创建
                delta_time = CarlaDataProvider.get_world().get_settings().fixed_delta_seconds
                frame_rate = 1 / delta_time
                sensor = OpenDriveMapReader(vehicle, frame_rate)
            elif sensor_spec['type'].startswith('sensor.speedometer'):
                delta_time = CarlaDataProvider.get_world().get_settings().fixed_delta_seconds
                frame_rate = 1 / delta_time
                sensor = SpeedometerReader(vehicle, frame_rate, other_actors_dict)
            elif sensor_spec['type'].startswith('sensor.can_bus'):
                # 速度表伪传感器直接在此处创建
                delta_time = CarlaDataProvider.get_world().get_settings().fixed_delta_seconds
                frame_rate = 1 / delta_time
                sensor = CanbusReader(vehicle, frame_rate, other_actors_dict, route)
            # 这些是 Carla 世界中产生的传感器
            else:
                bp = bp_library.find(str(sensor_spec['type']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    if sensor_spec['lens_circle_setting']:
                        bp.set_attribute('lens_circle_multiplier', str(3.0))
                        bp.set_attribute('lens_circle_falloff', str(3.0))
                        bp.set_attribute('chromatic_aberration_intensity', str(0.5))
                        bp.set_attribute('chromatic_aberration_offset', str(0))
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    bp.set_attribute('range', str(85))
                    bp.set_attribute('rotation_frequency', str(10))
                    bp.set_attribute('channels', str(64))
                    bp.set_attribute('upper_fov', str(10))
                    bp.set_attribute('lower_fov', str(-30))
                    bp.set_attribute('points_per_second', str(600000))
                    bp.set_attribute('atmosphere_attenuation_rate', str(0.004))
                    bp.set_attribute('dropoff_general_rate', str(0.45))
                    bp.set_attribute('dropoff_intensity_limit', str(0.8))
                    bp.set_attribute('dropoff_zero_intensity', str(0.4))
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.radar'):
                    bp.set_attribute('horizontal_fov', str(sensor_spec['fov']))  # degrees
                    bp.set_attribute('vertical_fov', str(sensor_spec['fov']))  # degrees
                    bp.set_attribute('points_per_second', '1500')
                    bp.set_attribute('range', '100')  # meters

                    sensor_location = carla.Location(x=sensor_spec['x'],
                                                     y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    bp.set_attribute('noise_alt_stddev', str(0.000005))
                    bp.set_attribute('noise_lat_stddev', str(0.000005))
                    bp.set_attribute('noise_lon_stddev', str(0.000005))
                    bp.set_attribute('noise_alt_bias', str(0.0))
                    bp.set_attribute('noise_lat_bias', str(0.0))
                    bp.set_attribute('noise_lon_bias', str(0.0))

                    sensor_location = carla.Location()
                    sensor_rotation = carla.Rotation()
                elif sensor_spec['type'].startswith('sensor.other.imu'):
                    bp.set_attribute('noise_accel_stddev_x', str(0.001))
                    bp.set_attribute('noise_accel_stddev_y', str(0.001))
                    bp.set_attribute('noise_accel_stddev_z', str(0.015))
                    bp.set_attribute('noise_gyro_stddev_x', str(0.001))
                    bp.set_attribute('noise_gyro_stddev_y', str(0.001))
                    bp.set_attribute('noise_gyro_stddev_z', str(0.001))

                    sensor_location = carla.Location()
                    sensor_rotation = carla.Rotation()
                # 创建传感器
                sensor_transform = carla.Transform(sensor_location, sensor_rotation)
                sensor = CarlaDataProvider.get_world().spawn_actor(bp, sensor_transform, vehicle)
            # 设置回调
            sensor.listen(CallBack(sensor_spec['id'], sensor_spec['type'], sensor, self._agent.sensor_interface))
            self._sensors_list.append(sensor)

        # 生成一次节拍信号可生成传感器
        CarlaDataProvider.get_world().tick()

    @staticmethod
    def validate_sensor_configuration(sensors, agent_track, selected_track):
        """
        如果使用挑战模式，请确保传感器配置有效
        有效配置时返回true，否则返回false
        """

        #if Track(selected_track) != agent_track:
        #    raise SensorConfigurationInvalid("You are submitting to the wrong track [{}]!".format(Track(selected_track)))

        sensor_count = {}
        sensor_ids = []

        for sensor in sensors:

            # 检查是否已使用
            sensor_id = sensor['id']
            if sensor_id in sensor_ids:
                raise SensorConfigurationInvalid("Duplicated sensor tag [{}]".format(sensor_id))
            else:
                sensor_ids.append(sensor_id)

            # 检查传感器是否有效
            if agent_track == Track.SENSORS:
                if sensor['type'].startswith('sensor.opendrive_map'):
                    raise SensorConfigurationInvalid("Illegal sensor used for Track [{}]!".format(agent_track))

            # 检查传感器的有效性
            if sensor['type'] not in AgentWrapper.allowed_sensors:
                raise SensorConfigurationInvalid("Illegal sensor used. {} are not allowed!".format(sensor['type']))

            # 检查传感器的外部特性
            if 'x' in sensor and 'y' in sensor and 'z' in sensor:
                if math.sqrt(sensor['x']**2 + sensor['y']**2 + sensor['z']**2) > MAX_ALLOWED_RADIUS_SENSOR:
                    raise SensorConfigurationInvalid(
                        "Illegal sensor extrinsics used for Track [{}]!".format(agent_track))

            # 检查传感器数量
            if sensor['type'] in sensor_count:
                sensor_count[sensor['type']] += 1
            else:
                sensor_count[sensor['type']] = 1

        for sensor_type, max_instances_allowed in SENSORS_LIMITS.items():
            if sensor_type in sensor_count and sensor_count[sensor_type] > max_instances_allowed:
                raise SensorConfigurationInvalid(
                    "Too many {} used! "
                    "Maximum number allowed is {}, but {} were requested.".format(sensor_type,
                                                                                  max_instances_allowed,
                                                                                  sensor_count[sensor_type]))

    def cleanup(self):
        """
        移除并销毁所有传感器
        """
        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                self._sensors_list[i].stop()
                self._sensors_list[i].destroy()
                self._sensors_list[i] = None
        self._sensors_list = []
