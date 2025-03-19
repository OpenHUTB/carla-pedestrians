#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
该模块包含 CARLA AD 排行榜的统计管理器
"""

from __future__ import print_function

from dictor import dictor
import math
import sys

from srunner.scenariomanager.traffic_events import TrafficEventType

from driving.utils.checkpoint_tools import fetch_dict, save_dict, create_default_json_msg

PENALTY_COLLISION_PEDESTRIAN = 0.50
PENALTY_COLLISION_VEHICLE = 0.60
PENALTY_COLLISION_STATIC = 0.65
PENALTY_TRAFFIC_LIGHT = 0.70
PENALTY_STOP = 0.80
PENALTY_ROUTE_DEV = 0.90


class RouteRecord():
    def __init__(self):
        self.route_id = None
        self.index = None
        self.status = 'Started'
        self.infractions = {
            'collisions_pedestrian': [],
            'collisions_vehicle': [],
            'collisions_layout': [],
            'red_light': [],
            'stop_infraction': [],
            'outside_route_lanes': [],
            'route_dev': [],
            'route_timeout': [],
            'vehicle_blocked': []
        }

        self.scores = {
            'score_route': 0,
            'score_penalty': 0,
            'score_composed': 0
        }

        self.meta = {}


def to_route_record(record_dict):
    record = RouteRecord()
    for key, value in record_dict.items():
        setattr(record, key, value)

    return record


def compute_route_length(config):
    trajectory = config.trajectory

    route_length = 0.0
    previous_location = None
    for location in trajectory:
        if previous_location:
            dist = math.sqrt((location.x-previous_location.x)*(location.x-previous_location.x) +
                             (location.y-previous_location.y)*(location.y-previous_location.y) +
                             (location.z - previous_location.z) * (location.z - previous_location.z))
            route_length += dist
        previous_location = location

    return route_length


class StatisticsManager(object):

    """
    这是 CARLA 排行榜的统计管理器。
    它通过场景评估标准在运行时收集数据。
    """

    def __init__(self):
        self._master_scenario = None
        self._registry_route_records = []  # 注册的路线记录

    def resume(self, endpoint):
        data = fetch_dict(endpoint)

        if data and dictor(data, '_checkpoint.records'):
            records = data['_checkpoint']['records']

            for record in records:
                self._registry_route_records.append(to_route_record(record))

    def set_route(self, route_id, index):

        self._master_scenario = None
        route_record = RouteRecord()
        route_record.route_id = route_id
        route_record.index = index

        if index < len(self._registry_route_records):
            # 该元素已经存在，因此我们更新它
            self._registry_route_records[index] = route_record
        else:
            self._registry_route_records.append(route_record)

    def set_scenario(self, scenario):
        """
        设置进行统计的场景。
        
        这与 set_route 配合使用，以便变量仅在模拟处于活动状态时才处于活动状态，以避免在模拟之间发生故障时出现统计错误
        """
        self._master_scenario = scenario

    def compute_route_statistics(self, config, duration_time_system=-1, duration_time_game=-1, failure=""):
        """
        通过评估所有相关场景标准来计算当前统计数据
        """
        index = config.index

        if not self._registry_route_records or index >= len(self._registry_route_records):
            raise Exception('Critical error with the route registry.')

        # 获取最新记录以填写
        route_record = self._registry_route_records[index]

        target_reached = False
        score_penalty = 1.0
        score_route = 0.0

        route_record.meta['duration_system'] = duration_time_system
        route_record.meta['duration_game'] = duration_time_game
        route_record.meta['route_length'] = compute_route_length(config)

        if self._master_scenario:
            if self._master_scenario.timeout_node.timeout:
                route_record.infractions['route_timeout'].append('Route timeout.')
                failure = "Agent timed out"

            for node in self._master_scenario.get_criteria():
                if node.list_traffic_events:
                    # 分析所有交通事件
                    for event in node.list_traffic_events:
                        if event.get_type() == TrafficEventType.COLLISION_STATIC:  # 碰撞静止的东西
                            score_penalty *= PENALTY_COLLISION_STATIC
                            route_record.infractions['collisions_layout'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.COLLISION_PEDESTRIAN:  # 碰撞行人
                            score_penalty *= PENALTY_COLLISION_PEDESTRIAN
                            route_record.infractions['collisions_pedestrian'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.COLLISION_VEHICLE:  # 碰撞车辆
                            score_penalty *= PENALTY_COLLISION_VEHICLE
                            route_record.infractions['collisions_vehicle'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.OUTSIDE_ROUTE_LANES_INFRACTION:  # 到车道外违规
                            score_penalty *= (1 - event.get_dict()['percentage'] / 100)
                            route_record.infractions['outside_route_lanes'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.TRAFFIC_LIGHT_INFRACTION:  # 红绿灯违规
                            score_penalty *= PENALTY_TRAFFIC_LIGHT
                            route_record.infractions['red_light'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.ROUTE_DEVIATION:  # 路线偏移
                            score_penalty *= PENALTY_ROUTE_DEV
                            route_record.infractions['route_dev'].append(event.get_message())
                            # failure = "Agent deviated from the route"

                        elif event.get_type() == TrafficEventType.STOP_INFRACTION:  # 停止标志违规
                            score_penalty *= PENALTY_STOP
                            route_record.infractions['stop_infraction'].append(event.get_message())

                        elif event.get_type() == TrafficEventType.VEHICLE_BLOCKED:  # 车辆阻塞
                            route_record.infractions['vehicle_blocked'].append(event.get_message())
                            failure = "Agent got blocked"

                        elif event.get_type() == TrafficEventType.ROUTE_COMPLETED:
                            score_route = 100.0
                            target_reached = True
                        elif event.get_type() == TrafficEventType.ROUTE_COMPLETION:
                            if not target_reached:
                                if event.get_dict():
                                    score_route = event.get_dict()['route_completed']
                                else:
                                    score_route = 0

        # 更新路线得分
        route_record.scores['score_route'] = score_route
        route_record.scores['score_penalty'] = score_penalty
        route_record.scores['score_composed'] = max(score_route*score_penalty, 0.0)

        # 更新状态
        if target_reached:
            route_record.status = 'Completed'
        else:
            route_record.status = 'Failed'
            if failure:
                route_record.status += ' - ' + failure

        return route_record

    def compute_global_statistics(self, total_routes):
        global_record = RouteRecord()
        global_record.scores['success_rate'] = 0  # 成功率
        global_record.scores['success_rate_strict'] = 0
        global_record.route_id = -1
        global_record.index = -1
        global_record.status = 'Completed'
        global_record.scores_std_dev = RouteRecord().scores

        if self._registry_route_records:
            for route_record in self._registry_route_records:
                global_record.scores['score_route'] += route_record.scores['score_route']
                global_record.scores['score_penalty'] += route_record.scores['score_penalty']
                global_record.scores['score_composed'] += route_record.scores['score_composed']
                global_record.scores['success_rate'] += int(route_record.scores['score_route']==100.0)
                global_record.scores['success_rate_strict'] += int(route_record.scores['score_composed'] == 100.0)

                for key in global_record.infractions.keys():
                    route_length_kms = max(route_record.scores['score_route'] / 100 * route_record.meta['route_length'] / 1000.0, 0.001)
                    if isinstance(global_record.infractions[key], list):
                        global_record.infractions[key] = len(route_record.infractions[key]) / route_length_kms
                    else:
                        global_record.infractions[key] += len(route_record.infractions[key]) / route_length_kms

                if route_record.status is not 'Completed':
                    global_record.status = 'Failed'
                    if 'exceptions' not in global_record.meta:
                        global_record.meta['exceptions'] = []
                    global_record.meta['exceptions'].append((route_record.route_id,
                                                             route_record.index,
                                                             route_record.status))

            for key in global_record.scores.keys():
                global_record.scores[key] /= float(total_routes)

            if total_routes == 1:
                for key in global_record.scores_std_dev.keys():
                    global_record.scores_std_dev[key] = 'NaN'
            else:
                for route_record in self._registry_route_records:
                    for key in global_record.scores_std_dev.keys():
                        global_record.scores_std_dev[key] += math.pow(route_record.scores[key] - global_record.scores[key], 2)

                for key in global_record.scores_std_dev.keys():
                    global_record.scores_std_dev[key] = math.sqrt(global_record.scores_std_dev[key] / float(total_routes - 1))

        return global_record

    @staticmethod
    def save_record(route_record, index, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        stats_dict = route_record.__dict__
        record_list = data['_checkpoint']['records']
        if index > len(record_list):
            print('Error! No enough entries in the list')
            sys.exit(-1)
        elif index == len(record_list):
            record_list.append(stats_dict)
        else:
            record_list[index] = stats_dict

        save_dict(endpoint, data)

    @staticmethod
    def save_global_record(route_record, sensors, total_routes, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        stats_dict = route_record.__dict__
        data['_checkpoint']['global_record'] = stats_dict
        # 保存的指标包括：成功分数、严格成功分数、分数构成、路线分数、处罚分数
        data['values'] = ['{:.3f}'.format(stats_dict['scores']['success_rate']),
                          '{:.3f}'.format(stats_dict['scores']['success_rate_strict']),
                          '{:.3f}'.format(stats_dict['scores']['score_composed']),
                          '{:.3f}'.format(stats_dict['scores']['score_route']),
                          '{:.3f}'.format(stats_dict['scores']['score_penalty']),
                          # 违规
                          '{:.3f}'.format(stats_dict['infractions']['collisions_pedestrian']),
                          '{:.3f}'.format(stats_dict['infractions']['collisions_vehicle']),
                          '{:.3f}'.format(stats_dict['infractions']['collisions_layout']),
                          '{:.3f}'.format(stats_dict['infractions']['red_light']),
                          '{:.3f}'.format(stats_dict['infractions']['stop_infraction']),
                          '{:.3f}'.format(stats_dict['infractions']['outside_route_lanes']),
                          '{:.3f}'.format(stats_dict['infractions']['route_dev']),
                          '{:.3f}'.format(stats_dict['infractions']['route_timeout']),
                          '{:.3f}'.format(stats_dict['infractions']['vehicle_blocked'])
                          ]

        data['labels'] = ['Success rate',                   # SR
                          'Success rate (strict)',          # S.SR
                          'Avg. driving score',             # Avg.DS
                          'Avg. route completion',          # Avg.RC,   Route compl.
                          'Avg. infraction penalty',        # Infrac penalty
                          'Collisions with pedestrians',
                          'Collisions with vehicles',
                          'Collisions with layout',
                          'Red lights infractions',
                          'Stop sign infractions',
                          'Off-road infractions',
                          'Route deviations',
                          'Route timeouts',
                          'Agent blocked'
                          ]

        entry_status = "Finished"
        eligible = True

        route_records = data["_checkpoint"]["records"]
        progress = data["_checkpoint"]["progress"]

        if progress[1] != total_routes:
            raise Exception('Critical error with the route registry.')

        if len(route_records) != total_routes or progress[0] != progress[1]:
            entry_status = "Finished with missing data"
            eligible = False
        else:
            for route in route_records:
                route_status = route["status"]
                if "Agent" in route_status:
                    entry_status = "Finished with agent errors"
                    break

        data['entry_status'] = entry_status
        data['eligible'] = eligible

        save_dict(endpoint, data)

    @staticmethod
    def save_sensors(sensors, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        if not data['sensors']:
            data['sensors'] = sensors

            save_dict(endpoint, data)

    @staticmethod
    def save_entry_status(entry_status, eligible, endpoint):
        data = fetch_dict(endpoint)
        if not data:
            data = create_default_json_msg()

        data['entry_status'] = entry_status
        data['eligible'] = eligible
        save_dict(endpoint, data)

    @staticmethod
    def clear_record(endpoint):
        if not endpoint.startswith(('http:', 'https:', 'ftp:')):
            with open(endpoint, 'w') as fd:
                fd.truncate(0)
