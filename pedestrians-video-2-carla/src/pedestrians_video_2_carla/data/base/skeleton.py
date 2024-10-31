from functools import lru_cache
from typing import Type

from pedestrians_scenarios.karma.pose.skeleton import Skeleton


SKELETONS = {}
MAPPINGS = {}


def get_skeleton_type_by_name(name):
    return SKELETONS[name]


def get_skeleton_name_by_type(skeleton):
    return skeleton.__name__


def register_skeleton(name, skeleton, mapping=None):
    SKELETONS[name] = skeleton
    if mapping is not None:
        MAPPINGS[skeleton] = mapping


@lru_cache(maxsize=None)
def get_common_indices(input_nodes: Type[Skeleton] = None, output_nodes: Type[Skeleton] = None):
    # input and output are the same or mappings are not defined
    if (input_nodes == output_nodes) or (input_nodes is not None and input_nodes not in MAPPINGS) or (output_nodes is not None and output_nodes not in MAPPINGS):
        return slice(None), slice(None)

    if input_nodes is not None:
        (input_carla_indices, input_indices) = zip(
            *[(c.value, o.value) for (c, o) in MAPPINGS[input_nodes]])

        if output_nodes is None:
            return input_carla_indices, input_indices

    if output_nodes is not None:
        (output_carla_indices, output_indices) = zip(
            *[(c.value, o.value) for (c, o) in MAPPINGS[output_nodes]])

        if input_nodes is None:
            return output_indices, output_carla_indices

    # we have both input and output nodes and their intersections with CARLA_SKELETON
    common_carla_indices = set(input_carla_indices).intersection(output_carla_indices)
    filtered_input = [(c, i) for (c, i) in zip(
        input_carla_indices, input_indices) if c in common_carla_indices]
    filtered_output = [(c, o) for (c, o) in zip(
        output_carla_indices, output_indices) if c in common_carla_indices]

    # if we sort by CARLA indices now, we will have matching order in both lists
    sorted_input = sorted(filtered_input, key=lambda x: x[0])
    sorted_output = sorted(filtered_output, key=lambda x: x[0])

    return [x[1] for x in sorted_output], [x[1] for x in sorted_input]
