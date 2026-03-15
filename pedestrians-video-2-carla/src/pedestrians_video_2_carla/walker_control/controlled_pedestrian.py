import copy
import random
from typing import Dict, Type, TypeVar, Generic, Union
import warnings
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON

from pedestrians_video_2_carla.walker_control.pose import Pose, PoseDict

try:
    import carla
except (ImportError, ModuleNotFoundError) as e:
    import pedestrians_video_2_carla.carla_utils.mock_carla as carla
    warnings.warn("Using mock carla.", category=ImportWarning)

from pedestrians_scenarios.karma.utils.deepcopy import deepcopy_location, deepcopy_rotation, deepcopy_transform

from pedestrians_video_2_carla.data.carla.utils import load, yaml_to_pose_dict


P = TypeVar('P', Pose, 'P3dPose')


class ControlledPedestrian(Generic[P]):
    def __init__(self, world: 'carla.World' = None, age: str = 'adult', gender: str = 'female', max_spawn_tries=10, reference_pose: Union[P, Type[P]] = Pose, *args, **kwargs):
        """
        Initializes the pedestrian that keeps track of its current pose.

        :param world: world object of the connected client, if not specified all calculations will be done
            on the client side and with no rendering; defaults to None
        :type world: carla.World, optional
        :param age: one of 'adult' or 'child'; defaults to 'adult'
        :type age: str, optional
        :param gender: one of 'male' or 'female'; defaults to 'female'
        :type gender: str, optional
        """
        super().__init__()

        self._age = age
        self._gender = gender

        pose_dict, root_hips_transform = self._load_reference_pose()
        if isinstance(reference_pose, Pose):
            self._current_pose = copy.deepcopy(reference_pose)
        else:
            self._current_pose: P = reference_pose(**kwargs)
            self._current_pose.relative = pose_dict
        self._root_hips_transform = root_hips_transform

        # spawn point (may be different than actual location the pedesrian has spawned, especially Z-wise);
        # if world is not specified this will always be point 0,0,0
        self._spawn_loc = carla.Location()
        self._world: 'carla.World' = None
        self._walker: 'carla.Walker' = None
        self._initial_transform = carla.Transform()
        self._world_transform = carla.Transform()
        self._max_spawn_tries = max_spawn_tries

        if world is not None:
            self.bind(world, True)

    def __deepcopy__(self, memo):
        """
        Creates deep copy of the ControlledPedestrian.
        Please note that the result is unbound to world, since it is impossible to spawn
        exact same actor in exactly same location. It is up to copying script to actually
        `.bind()` it as needed.

        :param memo: [description]
        :type memo: [type]
        :return: [description]
        :rtype: ControlledPedestrian
        """
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            if k == '_spawn_loc':
                setattr(result, k, deepcopy_location(v))
            elif k in ['_initial_transform', '_world_transform']:
                setattr(result, k, deepcopy_transform(v))
            elif k in ['_walker', '_world']:
                setattr(result, k, None)
            else:
                setattr(result, k, copy.deepcopy(v, memo))
        return result

    def bind(self, world: 'carla.World', ignore_shift=False):
        """
        Binds the pedestrian to the instance of carla.World

        :param world:
        :type world: carla.World
        """
        # this method cannot be used with mock_carla, since it only makes sense to bind to a real world
        # so raise runtime error if carla has no World
        if getattr(carla, 'World', None) is None:
            raise RuntimeError(
                "You are using mock carla, calls to bind are not allowed!")

        # remember current shift from initial position,
        # so we are able to teleport pedestrian there directly
        if not ignore_shift:
            shift = self.transform

        self._world = world
        self._walker = self._spawn_walker()
        self._initial_transform = self._walker.get_transform()
        self._world_transform = self._walker.get_transform()

        if not ignore_shift:  # there was no shift
            self.teleport_by(shift)

        self._walker.set_simulate_physics(enabled=True)
        self.apply_pose(True)

    def _spawn_walker(self) -> 'carla.Walker':
        blueprint_library = self._world.get_blueprint_library()
        matching_blueprints = [bp for bp in blueprint_library.filter("walker.pedestrian.*")
                               if bp.get_attribute('age') == self._age and bp.get_attribute('gender') == self._gender]
        walker_bp = random.choice(matching_blueprints)

        # Make pedestrian mortal
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')

        walker = None
        tries = 0
        while walker is None and tries < self._max_spawn_tries:
            tries += 1
            walker_loc = self._world.get_random_location_from_navigation()
            walker = self._world.try_spawn_actor(walker_bp, carla.Transform(walker_loc))

        if walker is None:
            raise RuntimeError("Couldn't spawn walker")
        else:
            self._spawn_loc = walker_loc

        self._world.tick()

        return walker

    def _load_reference_pose(self):
        unreal_pose = load('{}_{}'.format(self._age, self._gender))
        relative_pose, root_hips_transform = yaml_to_pose_dict(
            unreal_pose['transforms'])

        return relative_pose, root_hips_transform

    def teleport_by(self, transform: 'carla.Transform', cue_tick=False, from_initial=False) -> int:
        """
        Teleports the pedestrian in the world.

        :param transform: Transform relative to the current world transform describing the desired shift
        :type transform: carla.Transform
        :param cue_tick: should carla.World.tick() be called after sending control; defaults to False
        :type cue_tick: bool, optional
        :param from_initial: should teleport be applied from current world position (False, default) or
            from the initial spawn position (True). Mainly used when copying position/movements between 
            pedestrian instances.
        :type from_initial: bool, optional
        :return: World frame number if cue_tick==True else 0
        :rtype: int
        """
        if from_initial:
            reference_transform = self.initial_transform
        else:
            reference_transform = self.world_transform

        self._world_transform = carla.Transform(
            location=carla.Location(
                x=reference_transform.location.x + transform.location.x,
                y=reference_transform.location.y + transform.location.y,
                z=reference_transform.location.z + transform.location.z
            ),
            rotation=carla.Rotation(
                pitch=reference_transform.rotation.pitch + transform.rotation.pitch,
                yaw=reference_transform.rotation.yaw + transform.rotation.yaw,
                roll=reference_transform.rotation.roll + transform.rotation.roll
            )
        )

        if self._walker is not None:
            self._walker.set_transform(self._world_transform)

            if cue_tick:
                return self._world.tick()

        return 0

    def update_pose(self, rotations: Dict[str, 'carla.Rotation'], cue_tick=False) -> int:
        """
        Apply the movement specified as change in local rotations for selected bones.
        For example `pedestrian.update_pose({ 'crl_foreArm__L': carla.Rotation(pitch=60) })`
        will make the arm bend in the elbow by 60deg around Y axis using its current rotation
        plane (which gives roughly 60deg bend around the global Z axis).

        :param rotations: Change in local rotations for selected bones
        :type rotations: Dict[str, carla.Rotation]
        :param cue_tick: should carla.World.tick() be called after sending control; defaults to False
        :type cue_tick: bool, optional
        :return: World frame number if cue_tick==True else 0
        :rtype: int
        """

        self._current_pose.move(rotations)
        return self.apply_pose(cue_tick)

    def apply_pose(self, cue_tick: bool = False, pose_snapshot: PoseDict = None, root_hips_transform: 'carla.Transform' = None) -> int:
        """
        Applies the current absolute pose to the carla.Walker if it exists.

        :param cue_tick: should carla.World.tick() be called after sending control; defaults to False
        :type cue_tick: bool, optional
        :param pose_snapshot: OrderedDict containing pose relative coordinates.
            If not None, will be used instead of self._current_pose.
            This will **NOT** update the internal pose representation.
        :type pose_snapshot: PoseDict, optional
        :param root_hips_transform: Transform used to recover hips vs root point location,
            needed to actually position the Pedestrian in space correctly.
            If not None, will be used instead of self._root_hips_transform.
            This will **NOT** update the internal pose representation.
        :type root_hips_transform: carla.Transform, optional
        :return: World frame number if cue_tick==True else 0
        :rtype: int
        """
        if self._walker is not None:
            control = carla.WalkerBoneControlIn()

            if pose_snapshot is None:
                # this is a deepcopy
                pose_snapshot = self._current_pose.relative

            if root_hips_transform is None:
                root_hips_transform = self._root_hips_transform

            pose_snapshot[CARLA_SKELETON.crl_hips__C.name] = carla.Transform(
                location=deepcopy_location(root_hips_transform.location),
                rotation=deepcopy_rotation(
                    pose_snapshot[CARLA_SKELETON.crl_hips__C.name].rotation)
            )
            pose_snapshot[CARLA_SKELETON.crl_root.name] = carla.Transform(
                location=carla.Location(),
                rotation=deepcopy_rotation(root_hips_transform.rotation)
            )

            control.bone_transforms = list(pose_snapshot.items())

            self._walker.set_bones(control)
            self._walker.blend_pose(1)

            if cue_tick:
                return self._world.tick()
        return 0

    @ property
    def age(self) -> str:
        return self._age

    @ property
    def gender(self) -> str:
        return self._gender

    @ property
    def walker(self) -> 'carla.Walker':
        return self._walker

    @ property
    def world_transform(self) -> 'carla.Transform':
        if self._walker is not None:
            # if possible, get it from CARLA
            # don't ask me why Z is is some 0.91m above the actual root sometimes
            return self._walker.get_transform()
        return self._world_transform

    @world_transform.setter
    def world_transform(self, transform: 'carla.Transform'):
        if self._walker is not None:
            self._walker.set_transform(transform)
        self._world_transform = transform

    @ property
    def transform(self) -> 'carla.Transform':
        """
        Current pedestrian transform relative to the position it was spawned at.
        """
        world_transform = self.world_transform
        return carla.Transform(
            location=carla.Location(
                x=world_transform.location.x - self._initial_transform.location.x,
                y=world_transform.location.y - self._initial_transform.location.y,
                z=world_transform.location.z - self._initial_transform.location.z
            ),
            rotation=carla.Rotation(
                pitch=world_transform.rotation.pitch - self._initial_transform.rotation.pitch,
                yaw=world_transform.rotation.yaw - self._initial_transform.rotation.yaw,
                roll=world_transform.rotation.roll - self._initial_transform.rotation.roll
            )
        )

    @ property
    def initial_transform(self) -> 'carla.Transform':
        return deepcopy_transform(self._initial_transform)

    @ property
    def current_pose(self) -> P:
        # TODO: for bound pedestrians, should this ask CARLA for the current pose (and update)?
        # or should requesting pose from CARLA be done explicitly, to avoid unintended
        # slowdowns and/or non-obvious bugs? The current implementations is such that when
        # utilizing the ControlledPedestrian methods, the changes are applied to CARLA,
        # but when manipulating Pose directly, they are not, since Pose is abstracted.
        return self._current_pose

    @ property
    def spawn_shift(self) -> 'carla.Location':
        """
        Difference between spawn point and actual spawn location
        """
        return carla.Location(
            x=self._initial_transform.location.x - self._spawn_loc.x,
            y=self._initial_transform.location.y - self._spawn_loc.y,
            z=self._initial_transform.location.z - self._spawn_loc.z
        )
