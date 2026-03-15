from enum import Enum

from torch import nn

from .pose_changes import calculate_loss_pose_changes
from .loc_3d import calculate_loss_loc_3d
from .loc_2d import calculate_loss_loc_2d, Loc2DPoseLoss
from .loc_2d_3d import calculate_loss_loc_2d_3d
from .cum_pose_changes import calculate_loss_cum_pose_changes
from .rot_3d import calculate_loss_rot_3d
from .loc_2d_loc_rot_3d import calculate_loss_loc_2d_loc_rot_3d
from .weighted_loc_2d_loc_rot_3d import calculate_loss_weighted_loc_2d_loc_rot_3d
from .loc_rot_3d import calculate_loss_loc_rot_3d
from .per_joint_loc_2d import PerJointLoc2DPoseLoss
from .heatmaps_loss import HeatmapsLoss


class LossModes(Enum):
    """
    Enum for loss modes.

    For now this will not work with ddp_spawn, because it is not picklable, use ddp accelerator instead.
    """
    # Base functions with MSE
    loc_2d = (Loc2DPoseLoss, nn.MSELoss(reduction='mean'))
    common_loc_2d = (calculate_loss_loc_2d, nn.MSELoss(reduction='mean'))  # deprecated
    loc_3d = (calculate_loss_loc_3d, nn.MSELoss(reduction='mean'))
    rot_3d = (calculate_loss_rot_3d, nn.MSELoss(reduction='mean'))
    cum_pose_changes = (calculate_loss_cum_pose_changes, nn.MSELoss(reduction='mean'))
    pose_changes = (calculate_loss_pose_changes, nn.MSELoss(reduction='sum'))

    # Complex losses depending on base losses
    # Do NOT declare a complex loss depending on another complex loss,
    # it will most likely not work since there is no complex
    # dependencies resolving done, only "lets put all the dependencies first,
    # and actual losses later in the order of calculations".
    loc_2d_3d = (calculate_loss_loc_2d_3d, None, (
        'loc_2d', 'loc_3d'
    ))
    loc_2d_loc_rot_3d = (calculate_loss_loc_2d_loc_rot_3d, None, (
        'loc_2d', 'loc_3d', 'rot_3d'
    ))
    weighted_loc_2d_loc_rot_3d = (calculate_loss_weighted_loc_2d_loc_rot_3d, None, (
        'loc_2d', 'loc_3d', 'rot_3d'
    ))
    loc_rot_3d = (calculate_loss_loc_rot_3d, None, (
        'loc_3d', 'rot_3d'
    ))

    per_joint_loc_2d = (PerJointLoc2DPoseLoss, nn.MSELoss(reduction='mean'))

    # for images
    heatmaps = (HeatmapsLoss, nn.MSELoss(reduction='mean'))
