import argparse
import logging
import os
import time
from enum import Enum, auto
from typing import Callable

import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl

from .karma_data_provider import KarmaDataProvider


class KarmaStage(Enum):
    tick = auto()  # a new snapshot is received from the CARLA server
    close = auto()  # Karma is asked to close
    reload = auto()  # CARLA world is reloaded


class Karma(object):
    """
    Primary client/world/everything manager.

    Heavily inspired by main ScenarioRunner class & various gym Envs for CARLA.
    Assumption: this is a single-server single-client configuration, since we want to
    take the advantage of the ScenarioRunner's KarmaDataProvider helper,
    and it stores them as class variables.

    The sync mode if forced to be True, since TrafficManager is only supposed to be used
    in synchronous mode and conditionally using sync/not sync is very error prone.
    In the future, the async mode may be supported (possibly as explicit separate class),
    but for now, we'll keep it simple.
    """

    def __init__(
        self,
        host='server',
        port=2000,
        timeout=None,
        traffic_manager_port=8000,
        seed=22752,
        fps=30.0,
        hybrid_physics_mode=False,
        **kwargs
    ) -> None:
        self.__timeout = timeout if timeout is not None else float(
            os.getenv('CARLA_SERVER_START_PERIOD', '30.0'))
        self.__seed = seed
        self.__fps = fps
        self.__hybrid_physics_mode = hybrid_physics_mode

        self.__on_tick_callback_id = None

        self.__next_callback_id = 0
        self.__registered_callbacks = {
            event_type: {}
            for event_type in KarmaStage
        }
        self.__registered_controllers = {}

        self.__client = carla.Client(host, port)
        self.__client.set_timeout(self.__timeout)

        self.__traffic_manager = self.__client.get_trafficmanager(traffic_manager_port)
        KarmaDataProvider.set_traffic_manager_port(traffic_manager_port)

        # reset random seed
        KarmaDataProvider.get_rng(self.__seed)

        self.__world = None
        self.reset_world()

    @staticmethod
    def add_cli_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
        """
        Adds Karma-specific command line arguments.
        """
        subparser = parser.add_argument_group('Karma')

        subparser.add_argument('--host', default='server',
                               help='Hostname or IP of the CARLA server (default: server)')
        subparser.add_argument('--port', default=2000, type=int,
                               help='TCP port to listen to (default: 2000)')
        subparser.add_argument('--timeout', default=None, type=float,
                               help='Set the CARLA client timeout value in seconds (default: CARLA_SERVER_START_PERIOD, usually 30.0)')
        subparser.add_argument('--traffic-manager-port', default=8000, type=int,
                               help='Port to use for the TrafficManager (default: 8000)')
        subparser.add_argument('--seed', default=22752, type=int,
                               help='Seed used by the everything (default: 22752)')
        subparser.add_argument('--fps', default=30.0, type=float,
                               help='FPS of the simulation (default: 30)')
        subparser.add_argument('--hybrid-physics-mode', default=False, action='store_true',
                               help='Enable hybrid physics mode (default: False)')

        return parser

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

        return False

    def tick(self) -> int:
        return self.__world.tick()

    def reset_world(self, map_name=None):
        if self.__world is not None:
            prev_id = self.__world.id
            prev_name = KarmaDataProvider.get_map().name
        else:
            prev_id = None
            prev_name = None

        # clean up everything
        self.close(keep_client_alive=True)
        # reuse client
        KarmaDataProvider.set_client(self.__client)

        if self.__world is not None:
            tries = 3  # no point in trying more, the server is most probably dead
            while tries > 0:
                try:
                    if map_name is not None and prev_name != map_name:
                        logging.getLogger(__name__).debug(
                            f'Loading map {map_name}...')
                        self.__client.load_world(map_name)
                        logging.getLogger(__name__).debug(
                            f'Loading map {map_name} succeeded.')
                    else:
                        logging.getLogger(__name__).debug(
                            f'Reloading map {map_name}...')
                        self.__client.reload_world()
                        logging.getLogger(__name__).debug(
                            f'Reloading map {map_name} succeeded.')
                    break
                except RuntimeError:
                    tries -= 1
                    logging.getLogger(__name__).debug(
                        f'(Re)loading map {map_name} failed, {tries} tries left.')
                    # Sleep for a bit to give server a chance.
                    # This is actually the only place when it makes sense to 'wait and see'.
                    # If timeout or other error occurs anywhere else it should be dealt with by the caller.
                    time.sleep(self.__timeout)

        logging.getLogger(__name__).debug('Getting world...')
        self.__world = self.__client.get_world()
        tries = int(self.__timeout)
        while prev_id == self.__world.id and tries > 0:
            tries -= 1
            time.sleep(1)
            self.__world = self.__client.get_world()
        if tries < 1:
            raise RuntimeError('Could not reset world.')
        logging.getLogger(__name__).debug('Getting world succeeded.')

        # always reset settings
        settings = carla.WorldSettings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / self.__fps
        settings.deterministic_ragdolls = True
        self.__world.apply_settings(settings)
        logging.getLogger(__name__).debug(f'World settings applied: {settings}.')

        self.__world.set_pedestrians_seed(self.__seed)
        logging.getLogger(__name__).debug('Pedestrians seed set.')

        KarmaDataProvider.set_world(self.__world)

        self.__traffic_manager.set_synchronous_mode(True)
        self.__traffic_manager.set_random_device_seed(self.__seed)
        self.__traffic_manager.set_hybrid_physics_mode(self.__hybrid_physics_mode)
        logging.getLogger(__name__).debug('Traffic manager settings applied.')

        self.__on_tick_callback_id = self.__world.on_tick(self.on_carla_tick)
        logging.getLogger(__name__).debug('On world tick callback registered.')

        for callback in self.__registered_callbacks[KarmaStage.reload].copy().values():
            callback()
        logging.getLogger(__name__).debug('All reload callbacks called.')

    def close(self, keep_client_alive=False):
        KarmaDataProvider.cleanup()

        # are there any registered tick callbacks?
        # if so, give them a chance to complete
        if len(self.__registered_callbacks[KarmaStage.tick]) > 0:
            time.sleep(1)

        for callback in self.__registered_callbacks[KarmaStage.close].copy().values():
            callback()
        logging.getLogger(__name__).debug('All close callbacks called.')

        if self.__world:
            self.__world.remove_on_tick(self.__on_tick_callback_id)
            self.__world.apply_settings(carla.WorldSettings(
                synchronous_mode=False,
                fixed_delta_seconds=0.0
            ))
            logging.getLogger(__name__).debug('World settings reset.')
        if self.__traffic_manager:
            self.__traffic_manager.set_synchronous_mode(False)
            logging.getLogger(__name__).debug('Traffic manager settings reset.')

        if not keep_client_alive:
            del self.__client
            self.__client = None
            logging.getLogger(__name__).debug('Client stopped.')

    def on_carla_tick(self, snapshot: carla.WorldSnapshot):
        KarmaDataProvider.on_carla_tick(snapshot)

        for callback in self.__registered_callbacks[KarmaStage.tick].values():
            callback(snapshot)

    def register_callback(self, stage: KarmaStage, callback: Callable[[], None]) -> int:
        """
        Registers a callback to be called at specific life-cycle stage.
        """
        callback_id = self.__next_callback_id
        self.__next_callback_id += 1
        self.__registered_callbacks[stage][callback_id] = callback
        return callback_id

    def unregister_callback(self, callback_id: int) -> None:
        """
        Unregisters a callback previously registered with register_callback.
        """
        for stage_type in self.__registered_callbacks:
            if callback_id in self.__registered_callbacks[stage_type]:
                del self.__registered_callbacks[stage_type][callback_id]
                return

    def register_controller(self, controller: BasicControl) -> None:
        """
        Registers a controller to be called at tick / reload.
        """

        tick_callback_id = self.register_callback(KarmaStage.tick, controller.run_step)
        reload_callback_id = self.register_callback(KarmaStage.reload, controller.reset)

        self.__registered_controllers[controller] = (
            tick_callback_id, reload_callback_id)

    def unregister_controller(self, controller: BasicControl) -> None:
        """
        Unregisters a controller previously registered with register_controller.
        """
        if controller in self.__registered_controllers:
            tick_callback_id, reload_callback_id = self.__registered_controllers[controller]
            self.unregister_callback(tick_callback_id)
            self.unregister_callback(reload_callback_id)
            del self.__registered_controllers[controller]
