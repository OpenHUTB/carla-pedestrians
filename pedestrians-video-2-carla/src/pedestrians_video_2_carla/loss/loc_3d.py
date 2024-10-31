from typing import Dict, Type

from pedestrians_video_2_carla.data.base.skeleton import (Skeleton,
                                                          get_common_indices)
from pedestrians_video_2_carla.transforms.pose.normalization.hips_neck_extractor import HipsNeckExtractor
from pedestrians_video_2_carla.transforms.pose.normalization.normalizer import Normalizer
from pytorch_lightning.utilities.warnings import rank_zero_warn
from torch import Tensor
from torch.nn.modules import loss


def calculate_loss_loc_3d(criterion: loss._Loss, input_nodes: Type[Skeleton], output_nodes: Type[Skeleton], absolute_pose_loc: Tensor, targets: Dict[str, Tensor], **kwargs) -> Tensor:
    """
    Calculates the loss for the 3D pose.

    :param criterion: Criterion to use for the loss calculation, e.g. nn.MSELoss().
    :type criterion: _Loss
    :param input_nodes: Type of the input skeleton, e.g. BODY_25_SKELETON or CARLA_SKELETON.
    :type input_nodes: Type[Skeleton]
    :param output_nodes: Type of the output skeleton, e.g. BODY_25_SKELETON or CARLA_SKELETON.
    :type output_nodes: Type[Skeleton]
    :param absolute_pose_loc: Absolute pose location coordinates as calculates by the projection module.
    :type absolute_pose_loc: Tensor
    :param targets: Dictionary returned from dataset that contains the target absolute poses.
    :type targets: Dict[str, Tensor]
    :return: Calculated loss.
    :rtype: Tensor
    """
    try:
        output_indices, input_indices = get_common_indices(input_nodes, output_nodes)

        loss = criterion(
            absolute_pose_loc[:, :, output_indices],
            targets['absolute_pose_loc'][:, :, input_indices]
        )
    except KeyError:
        rank_zero_warn(
            "The 'loc_3d' loss is not supported for this data, missing 'absolute_pose_loc' in targets.")
        return None
    return loss
