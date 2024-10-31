from typing import Dict
import torch
from torch import Tensor


def calculate_loss_weighted_loc_2d_loc_rot_3d(requirements: Dict[str, Tensor], loss_weights: Dict[str, Tensor], **kwargs) -> Tensor:
    """
    Calculates the weighted sum of the 'loc_2d', 'loc_3d' and 'rot_3d' losses.

    :param requirements: The dictionary containing the calculated 'loc_2d', 'loc_3d', and 'rot_3d'.
    :type requirements: Dict[str, Tensor]
    :param loss_weights: The dictionary containing the weights for 'loc_2d', 'loc_3d', and 'rot_3d'.
    :type requirements: Dict[str, Tensor]
    :return: The weighted sum of the 'loc_2d', 'loc_3d', and 'rot_3d' losses.
    :rtype: Tensor
    """
    try:
        loss = torch.tensor(loss_weights.get('loc_2d', 1.0), device=requirements['loc_2d'].device) * requirements['loc_2d'] + \
            torch.tensor(loss_weights.get('loc_3d', 1.0), device=requirements['loc_3d'].device) * requirements['loc_3d'] + \
            torch.tensor(loss_weights.get('rot_3d', 1.0),
                         device=requirements['rot_3d'].device) * requirements['rot_3d']
    except KeyError:
        return None

    return loss
