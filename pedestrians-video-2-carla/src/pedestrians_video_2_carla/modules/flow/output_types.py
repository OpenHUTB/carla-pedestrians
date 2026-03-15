from enum import Enum


class MovementsModelOutputType(Enum):
    """
    Enum for the different model types.
    """
    pose_changes = 0  # default, prefferred

    # undesired, but possible; it will most likely deform the skeleton; incompatible with some loss functions
    absolute_loc_rot = 1

    # undesired, but possible; it will most likely deform the skeleton and results in broken rotations; incompatible with some loss functions
    absolute_loc = 2

    # somewhat ok
    relative_rot = 3

    # 2D pose to 2D pose; used in autoencoder flow
    pose_2d = 4


class TrajectoryModelOutputType(Enum):
    """
    Enum for the different model types.
    """
    changes = 0  # default
    loc_rot = 1


class ClassificationModelOutputType(Enum):
    """
    Enum for the different model types.
    """
    multiclass = 0  # default
    binary = 1


class PoseEstimationModelOutputType(Enum):
    """
    Enum for the different model types.
    """
    heatmaps = 100  # default
    pose_2d = 4
