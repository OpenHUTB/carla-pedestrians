#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
用于解析所有路线和场景配置参数的模块。
"""
from collections import OrderedDict
import json
import math
import xml.etree.ElementTree as ET
import os

import carla
from agents.navigation.local_planner import RoadOption
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration

# TODO  检查这个阈值，它可能有点大，但不会大到让我们聚类场景。
TRIGGER_THRESHOLD = 2.0  # 判断触发位置是新的还是重复的阈值，适用于匹配位置
TRIGGER_ANGLE_THRESHOLD = 10  # 在匹配变换时，用于表示两个角度是否可以考虑匹配的阈值。


class RouteParser(object):

    """
    纯静态类，用于解析所有路线和场景配置参数。
    """

    @staticmethod
    def parse_annotations_file(annotation_filename):
        """
        返回场景将要发生位置的注释。
        :param annotation_filename: 注释文件的文件名
        :return:
        """
        with open(annotation_filename, 'r') as f:
            annotation_dict = json.loads(f.read(), object_pairs_hook=OrderedDict)

        final_dict = OrderedDict()

        for env_name, env_dict in annotation_dict['envs'].items():   # annotation_dict['available_scenarios']
            final_dict.update({env_name: env_dict})

        package_name = annotation_dict['package_name']

        return final_dict, package_name  # 该文件具有当前地图名称，该名称是一个元素向量

    @staticmethod
    def parse_routes_file(route_filename, scenario_file, single_route=None):
        """
        返回路线元素的列表。
        :param route_filename: 一组路线的路径。
        :param single_route: 如果设置，则只返回此路线
        :return: 包含路线的航点、ID 和城镇的字典列表
        """

        list_route_descriptions = []
        tree = ET.parse(route_filename)
        for route in tree.iter("route"):

            route_id = route.attrib['id']
            if single_route and route_id != single_route:
                continue

            new_config = RouteScenarioConfiguration()
            new_config.town = route.attrib['town']
            new_config.name = "RouteScenario_{}".format(route_id)
            new_config.weather = RouteParser.parse_weather(route)
            new_config.scenario_file = scenario_file

            waypoint_list = []  # 此路线上可找到的航点列表
            for waypoint in route.iter('waypoint'):
                waypoint_list.append(carla.Location(x=float(waypoint.attrib['x']),
                                                    y=float(waypoint.attrib['y']),
                                                    z=float(waypoint.attrib['z'])))

            new_config.trajectory = waypoint_list

            list_route_descriptions.append(new_config)

        return list_route_descriptions

    @staticmethod
    def parse_scenarios_routes_file(route_root, envs_dict, package_name):
        """
        返回路线元素的列表。
        :param route_filename: 一组路线的路径。
        :param single_route: 如果设置，则只返回此路线
        :return: 包含路线的航点、ID 和城镇的字典列表
        """

        list_scenario_route_descriptions = []

        for env_name, description in envs_dict.items():
            route_filename = os.path.join(route_root, description['route']['file'])
            tree = ET.parse(route_filename)
            tree.iterfind("route")
            for route in tree.iter("route"):
                route_id = route.attrib['id']
                if int(route_id) == int(description['route']['id']):
                    waypoint_list = []
                    for waypoint in route.iter('waypoint'):
                        waypoint_list.append(carla.Location(x=float(waypoint.attrib['x']),
                                                            y=float(waypoint.attrib['y']),
                                                            z=float(waypoint.attrib['z'])))

                    new_config = RouteScenarioConfiguration()
                    new_config.package_name = package_name
                    new_config.town = description['town_name']
                    new_config.name = env_name
                    new_config.ego_model = description['vehicle_model']
                    new_config.weather = RouteParser.parse_weather(description['weather_profile'])
                    new_config.scenarios = description['scenarios']
                    new_config.trajectory = waypoint_list
                    list_scenario_route_descriptions.append(new_config)

        return list_scenario_route_descriptions

    @staticmethod
    def parse_weather(preset_weather):
        """
        返回 carla.WeatherParameters，其中包含该路线的相应天气。如果路线没有天气属性，则触发默认属性。
        """

        weather = carla.WeatherParameters()
        if preset_weather == 'ClearNoon':
            weather = carla.WeatherParameters.ClearNoon  # 清晰的中午
        elif preset_weather == 'CloudyNoon':
            weather = carla.WeatherParameters.CloudyNoon  # 多云的中午
        elif preset_weather == 'WetNoon':
            weather = carla.WeatherParameters.WetNoon  # 潮湿的中午
        elif preset_weather == 'WetCloudyNoon':
            weather = carla.WeatherParameters.WetCloudyNoon  # 潮湿多余的中午
        elif preset_weather == 'MidRainyNoon':
            weather = carla.WeatherParameters.MidRainyNoon  # 下中雨的中午
        elif preset_weather == 'HardRainNoon':
            weather = carla.WeatherParameters.HardRainNoon  # 下大雨的中午
        elif preset_weather == 'SoftRainNoon':
            weather = carla.WeatherParameters.SoftRainNoon  # 下细雨的中午
        elif preset_weather == 'ClearSunset':
            weather = carla.WeatherParameters.ClearSunset
        elif preset_weather == 'CloudySunset':
            weather = carla.WeatherParameters.CloudySunset
        elif preset_weather == 'WetSunset':
            weather = carla.WeatherParameters.WetSunset
        elif preset_weather == 'WetCloudySunset':
            weather = carla.WeatherParameters.WetCloudySunset
        elif preset_weather == 'MidRainSunset':
            weather = carla.WeatherParameters.MidRainSunset
        elif preset_weather == 'HardRainSunset':
            weather = carla.WeatherParameters.HardRainSunset
        elif preset_weather == 'SoftRainSunset':
            weather = carla.WeatherParameters.SoftRainSunset

        return weather

    @staticmethod
    def check_trigger_position(new_trigger, existing_triggers):
        """
        检查此触发位置是否已经存在或者是否是新的。
        :param new_trigger:
        :param existing_triggers:
        :return:
        """

        for trigger_id in existing_triggers.keys():
            trigger = existing_triggers[trigger_id]
            dx = trigger['x'] - new_trigger['x']
            dy = trigger['y'] - new_trigger['y']
            distance = math.sqrt(dx * dx + dy * dy)

            dyaw = (trigger['yaw'] - new_trigger['yaw']) % 360
            if distance < TRIGGER_THRESHOLD \
                and (dyaw < TRIGGER_ANGLE_THRESHOLD or dyaw > (360 - TRIGGER_ANGLE_THRESHOLD)):
                return trigger_id

        return None

    @staticmethod
    def convert_waypoint_float(waypoint):
        """
        将航点值转换为浮点数 float
        """
        waypoint['x'] = float(waypoint['x'])
        waypoint['y'] = float(waypoint['y'])
        waypoint['z'] = float(waypoint['z'])
        waypoint['yaw'] = float(waypoint['yaw'])

    @staticmethod
    def match_world_location_to_route(world_location, route_description):
        """
        我们将此位置与给定的路线进行匹配。
            world_location:
            route_description:
        """
        def match_waypoints(waypoint1, wtransform):
            """
            检查 waypoint1 和 wtransform 是否相似
            """
            dx = float(waypoint1['x']) - wtransform.location.x
            dy = float(waypoint1['y']) - wtransform.location.y
            dz = float(waypoint1['z']) - wtransform.location.z
            dpos = math.sqrt(dx * dx + dy * dy + dz * dz)

            dyaw = (float(waypoint1['yaw']) - wtransform.rotation.yaw) % 360

            return dpos < TRIGGER_THRESHOLD \
                and (dyaw < TRIGGER_ANGLE_THRESHOLD or dyaw > (360 - TRIGGER_ANGLE_THRESHOLD))

        match_position = 0
        # TODO 该函数可以优化为以 Log(N) 时间运行
        for route_waypoint in route_description:
            if match_waypoints(world_location, route_waypoint[0]):
                return match_position
            match_position += 1

        return None

    @staticmethod
    def get_scenario_type(scenario, match_position, trajectory):
        """
        有些场景根据路线不同会有不同的类型。
        :param scenario: the scenario name
        :param match_position: the matching position for the scenarion
        :param trajectory: the route trajectory the ego is following
        :return: tag representing this subtype

        Also used to check which are not viable (Such as an scenario
        that triggers when turning but the route doesnt')
        WARNING: These tags are used at:
            - VehicleTurningRoute
            - SignalJunctionCrossingRoute
        and changes to these tags will affect them
        """

        def check_this_waypoint(tuple_wp_turn):
            """
            Decides whether or not the waypoint will define the scenario behavior
            """
            if RoadOption.LANEFOLLOW == tuple_wp_turn[1]:
                return False
            elif RoadOption.CHANGELANELEFT == tuple_wp_turn[1]:
                return False
            elif RoadOption.CHANGELANERIGHT == tuple_wp_turn[1]:
                return False
            return True

        # Unused tag for the rest of scenarios,
        # can't be None as they are still valid scenarios
        subtype = 'valid'

        if scenario == 'Scenario4':
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.LEFT == tuple_wp_turn[1]:
                        subtype = 'S4left'
                    elif RoadOption.RIGHT == tuple_wp_turn[1]:
                        subtype = 'S4right'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        if scenario == 'Scenario7':
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.LEFT == tuple_wp_turn[1]:
                        subtype = 'S7left'
                    elif RoadOption.RIGHT == tuple_wp_turn[1]:
                        subtype = 'S7right'
                    elif RoadOption.STRAIGHT == tuple_wp_turn[1]:
                        subtype = 'S7opposite'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        if scenario == 'Scenario8':
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.LEFT == tuple_wp_turn[1]:
                        subtype = 'S8left'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        if scenario == 'Scenario9':
            for tuple_wp_turn in trajectory[match_position:]:
                if check_this_waypoint(tuple_wp_turn):
                    if RoadOption.RIGHT == tuple_wp_turn[1]:
                        subtype = 'S9right'
                    else:
                        subtype = None
                    break  # Avoid checking all of them
                subtype = None

        return subtype

    @staticmethod
    def scan_route_for_scenarios(route_name, trajectory, world_annotations):
        """
        Just returns a plain list of possible scenarios that can happen in this route by matching
        the locations from the scenario into the route description

        :return:  A list of scenario definitions with their correspondent parameters
        """

        # the triggers dictionaries:
        existent_triggers = OrderedDict()
        # We have a table of IDs and trigger positions associated
        possible_scenarios = OrderedDict()

        # Keep track of the trigger ids being added
        latest_trigger_id = 0

        for town_name in world_annotations.keys():
            if town_name != route_name:
                continue

            scenarios = world_annotations[town_name]
            for scenario in scenarios:  # For each existent scenario
                scenario_name = scenario["scenario_type"]
                for event in scenario["available_event_configurations"]:
                    waypoint = event['transform']  # trigger point of this scenario
                    RouteParser.convert_waypoint_float(waypoint)
                    # We match trigger point to the  route, now we need to check if the route affects
                    match_position = RouteParser.match_world_location_to_route(
                        waypoint, trajectory)
                    if match_position is not None:
                        # We match a location for this scenario, create a scenario object so this scenario
                        # can be instantiated later

                        if 'other_actors' in event:
                            other_vehicles = event['other_actors']
                        else:
                            other_vehicles = None
                        scenario_subtype = RouteParser.get_scenario_type(scenario_name, match_position,
                                                                         trajectory)
                        if scenario_subtype is None:
                            continue
                        scenario_description = {
                            'name': scenario_name,
                            'other_actors': other_vehicles,
                            'trigger_position': waypoint,
                            'scenario_type': scenario_subtype, # some scenarios have route dependent configurations
                        }

                        trigger_id = RouteParser.check_trigger_position(waypoint, existent_triggers)
                        if trigger_id is None:
                            # This trigger does not exist create a new reference on existent triggers
                            existent_triggers.update({latest_trigger_id: waypoint})
                            # Update a reference for this trigger on the possible scenarios
                            possible_scenarios.update({latest_trigger_id: []})
                            trigger_id = latest_trigger_id
                            # Increment the latest trigger
                            latest_trigger_id += 1

                        possible_scenarios[trigger_id].append(scenario_description)

        return possible_scenarios, existent_triggers

    @staticmethod
    def setup_scenarios_for_route(route_name, trajectory, scenarios):
        """
                Just returns a plain list of possible scenarios that can happen in this route by matching
                the locations from the scenario into the route description

                :return:  A list of scenario definitions with their correspondent parameters
                """

        # the triggers dictionaries:
        existent_triggers = OrderedDict()
        # We have a table of IDs and trigger positions associated
        possible_scenarios = OrderedDict()

        # Keep track of the trigger ids being added
        latest_trigger_id = 0

        for town_name in world_annotations.keys():
            if town_name != route_name:
                continue

            scenarios = world_annotations[town_name]
            for scenario in scenarios:  # For each existent scenario
                scenario_name = scenario["scenario_type"]
                for event in scenario["available_event_configurations"]:
                    waypoint = event['transform']  # trigger point of this scenario
                    RouteParser.convert_waypoint_float(waypoint)
                    # We match trigger point to the  route, now we need to check if the route affects
                    match_position = RouteParser.match_world_location_to_route(
                        waypoint, trajectory)
                    if match_position is not None:
                        # We match a location for this scenario, create a scenario object so this scenario
                        # can be instantiated later

                        if 'other_actors' in event:
                            other_vehicles = event['other_actors']
                        else:
                            other_vehicles = None
                        scenario_subtype = RouteParser.get_scenario_type(scenario_name, match_position,
                                                                         trajectory)
                        if scenario_subtype is None:
                            continue
                        scenario_description = {
                            'name': scenario_name,
                            'other_actors': other_vehicles,
                            'trigger_position': waypoint,
                            'scenario_type': scenario_subtype,  # some scenarios have route dependent configurations
                        }

                        trigger_id = RouteParser.check_trigger_position(waypoint, existent_triggers)
                        if trigger_id is None:
                            # This trigger does not exist create a new reference on existent triggers
                            existent_triggers.update({latest_trigger_id: waypoint})
                            # Update a reference for this trigger on the possible scenarios
                            possible_scenarios.update({latest_trigger_id: []})
                            trigger_id = latest_trigger_id
                            # Increment the latest trigger
                            latest_trigger_id += 1

                        possible_scenarios[trigger_id].append(scenario_description)

        return possible_scenarios
