from enum import Enum


class PedestrianRenderers(Enum):
    # when selected, rendering is disabled
    none = 0

    # actual available renderers
    source_videos = 1
    source_carla = 2
    target_points = 3  # original data
    input_points = 4  # what model sees (e.g. after noise/missing data is introduced)
    projection_points = 5  # model outputs (2D projection)
    carla = 6
    smpl = 7

    # black window
    zeros = 100


class MergingMethod(Enum):
    vertical = 0
    horizontal = 1
    square = 2
