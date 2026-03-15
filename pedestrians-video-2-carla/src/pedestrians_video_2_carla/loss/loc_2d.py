from typing import Dict, Type

import torch

from pedestrians_video_2_carla.data.base.skeleton import (Skeleton,
                                                          get_common_indices)
from torch import Tensor
from torch.nn.modules import loss

from pedestrians_video_2_carla.loss.base_pose_loss import BasePoseLoss
from pedestrians_video_2_carla.utils.tensors import get_missing_joints_mask


def calculate_loss_loc_2d(
        criterion: loss._Loss,
        input_nodes: Type[Skeleton],
        output_nodes: Type[Skeleton],
        targets: Dict[str, Tensor],
        projection_2d: Tensor = None,
        projection_2d_transformed: Tensor = None,
        mask_missing_joints: bool = True,
        **kwargs) -> Tensor:
    """
    Calculates the loss for the 2D pose projection.
    Only accounts for common nodes between input skeleton and CARLA_SKELETON, as defined in MAPPINGS.

    :param criterion: Criterion to use for the loss calculation, e.g. nn.MSELoss().
    :type criterion: _Loss
    :param input_nodes: Type of the input skeleton, e.g. BODY_25_SKELETON or CARLA_SKELETON.
    :type input_nodes: Type[Skeleton]
    :param output_nodes: Type of the output skeleton, e.g. BODY_25_SKELETON or CARLA_SKELETON.
    :type output_nodes: Type[Skeleton]
    :param targets: Dictionary returned from dataset that contains the target absolute poses.
    :type targets: Dict[str, Tensor]
    :param projection_2d: Projection as calculated by the projection module.
    :type projection_2d: Tensor
    :param projection_2d_transformed: Projection as calculated by the projection module, transformed using same transform as in DataModule.
    :type projection_2d_transformed: Tensor
    :param mask_missing_joints: Whether to mask out ground truth missing joints.
    :type mask_missing_joints: bool
    :return: Calculated loss.
    :rtype: Tensor
    """
    assert projection_2d is not None or projection_2d_transformed is not None, 'Either projection_2d or projection_2d_transformed must be provided.'

    output_indices, input_indices = get_common_indices(input_nodes, output_nodes)

    if projection_2d_transformed is not None:
        common_projection = projection_2d_transformed[..., output_indices, 0:2]
        common_gt = targets['projection_2d_transformed'][..., input_indices, 0:2]
    else:
        common_projection = projection_2d[..., output_indices, 0:2]
        common_gt = targets['projection_2d'][..., input_indices, 0:2]

    if mask_missing_joints:
        mask = get_missing_joints_mask(
            common_gt, input_nodes.get_hips_point(), input_indices)
        common_projection = common_projection[mask]
        common_gt = common_gt[mask]

    loss = criterion(
        common_projection,
        common_gt
    )

    return loss


class Loc2DPoseLoss(BasePoseLoss):
    def _extract_gt_targets(self,
                            targets: Dict[str, Tensor],
                            **kwargs) -> Tensor:
        assert 'projection_2d' in targets or 'projection_2d_transformed' in targets, 'Either projection_2d or projection_2d_transformed must be provided.'

        if 'projection_2d_transformed' in targets:
            return targets['projection_2d_transformed'][..., 0:2]
        else:
            return targets['projection_2d'][..., 0:2]

    def _extract_predicted_targets(self,
                                   projection_2d: Tensor = None,
                                   projection_2d_transformed: Tensor = None,
                                   **kwargs) -> Tensor:
        assert projection_2d is not None or projection_2d_transformed is not None, 'Either projection_2d or projection_2d_transformed must be provided.'

        if projection_2d_transformed is not None:
            return projection_2d_transformed[..., 0:2]
        else:
            return projection_2d[..., 0:2]
