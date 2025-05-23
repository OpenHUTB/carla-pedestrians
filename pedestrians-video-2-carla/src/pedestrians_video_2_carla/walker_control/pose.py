import copy
import warnings
from typing import Any, Dict
from typing import OrderedDict as OrderedDictType

try:
    import carla
except (ImportError, ModuleNotFoundError) as e:
    import pedestrians_video_2_carla.carla_utils.mock_carla as carla
    warnings.warn("Using mock carla.", category=ImportWarning)

import time
from collections import OrderedDict

from pedestrians_scenarios.karma.utils.rotations import mul_carla_rotations
from pedestrians_scenarios.karma.utils.deepcopy import deepcopy_transform
from pedestrians_video_2_carla.data.carla.utils import load

from pedestrians_scenarios.karma.pose.types import PoseDict


class Pose(object):
    def __init__(self, structure=None, *args, **kwargs):
        """
        保持和操纵姿态的基类。
        """
        super().__init__()

        self._structure = structure if structure is not None else load('structure')[
            'structure']

        self.__relative_pose = OrderedDict()
        self.__add_to_pose(self._structure[0])

        self.__empty_pose = copy.deepcopy(self.__relative_pose)

        self._last_rel_mod = time.time_ns()
        self._last_abs_mod = None
        self._last_abs = None

    def __add_to_pose(self, structure: Dict[str, Any]):
        (bone_name, substructures) = list(structure.items())[0]
        self.__relative_pose[bone_name] = None

        if substructures is not None:
            for substructure in substructures:
                self.__add_to_pose(substructure)

    def __transform_descendants(self,
                                absolute_pose: OrderedDict,
                                relative_pose: OrderedDict,
                                substructure: Dict[str, Any],
                                prev_transform: 'carla.Transform'
                                ):
        # 这里不应该有多个项目 item
        (bone_name, subsubstructures) = list(substructure.items())[0]

        # 我们需要手动复制 carla.Location，因为它似乎已被修改
        absolute_pose[bone_name] = carla.Transform(
            location=prev_transform.transform(relative_pose[bone_name].location),
            rotation=mul_carla_rotations(
                prev_transform.rotation, relative_pose[bone_name].rotation)
        )
        if subsubstructures is not None:
            for subsubstructure in subsubstructures:
                self.__transform_descendants(
                    absolute_pose,
                    relative_pose,
                    subsubstructure,
                    absolute_pose[bone_name]
                )

    @staticmethod
    def _deepcopy_pose_dict(orig_pose_dict):
        pose_dict = OrderedDict()
        for bone_name, transform in orig_pose_dict.items():
            if transform is not None:
                # carla.Transform 无法进行深度复制，因此我们手动进行
                pose_dict[bone_name] = deepcopy_transform(transform)
            else:
                pose_dict[bone_name] = None
        return pose_dict

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            if k.endswith('__relative_pose'):
                pose_dict = self._deepcopy_pose_dict(self.__relative_pose)
                setattr(result, k, pose_dict)
            elif (k.endswith('__last_abs') or k.endswith('__last_abs_mod')):
                setattr(result, k, None)
            else:
                setattr(result, k, copy.deepcopy(v, memo))
        return result

    @property
    def structure(self):
        return self._structure

    @property
    def empty(self):
        return self._deepcopy_pose_dict(self.__empty_pose)

    @property
    def relative(self) -> PoseDict:
        """
        返回基于骨骼父级的参与者的姿态和变换。这类似于 CARLA 0.9.13 carla.WalkerBoneControlOut“相对”变换。
        See https://carla.readthedocs.io/en/0.9.13/python_api/#carla.Walker.get_bones for more info.

        :return: 以骨骼名称作为键、以相对变换作为值的姿势词典。
        :rtype: PoseDict
        """
        return self._deepcopy_pose_dict(self.__relative_pose)

    @relative.setter
    def relative(self, new_pose_dict):
        self.__relative_pose.update(new_pose_dict)
        self._last_rel_mod = time.time_ns()

    @property
    def absolute(self) -> PoseDict:
        """
        Returns the pose of the actor relative to its root (pivot) point.
        This is analogous to CARLA 0.9.13 carla.WalkerBoneControlOut 'component' (or 'actor') transforms.
        See https://carla.readthedocs.io/en/0.9.13/python_api/#carla.Walker.get_bones for more info.

        :return: Pose dictionary with bone names as keys and absolute (AKA component/actor) transforms as values.
        :rtype: PoseDict
        """
        if self._last_abs_mod != self._last_rel_mod:
            # ensure bones in absolute pose will be in the same order as they were in relative
            # this will be updated in-place
            absolute_pose = self.empty

            # we need to operate on copies of carla.Transform, since it seems that `.transform`
            # modifies them in place
            relative_pose = self.relative

            # we can only handle single root node
            self.__transform_descendants(
                absolute_pose,
                relative_pose,
                self._structure[0],
                carla.Transform()
            )

            self._last_abs = absolute_pose
            self._last_abs_mod = self._last_rel_mod

        return self._deepcopy_pose_dict(self._last_abs)

    def move(self, rotations: Dict[str, 'carla.Rotation']):
        # use getter to ensure we have a copy of self._relative_pose
        new_pose = self.relative

        # for each defined rotation, we merge it with the current one
        for bone_name, rotation_change in rotations.items():
            new_pose[bone_name].rotation = mul_carla_rotations(
                new_pose[bone_name].rotation, rotation_change)

        self.relative = new_pose
