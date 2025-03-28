from typing import Dict

from torch import Tensor


def calculate_loss_loc_2d_3d(requirements: Dict[str, Tensor], **kwargs) -> Tensor:
    """
    Calculates the simple sum of the 'loc_2d' and 'loc_3d' losses.

    :param requirements: The dictionary containing the calculated 'loc_2d' and 'loc_3d'.
    :type requirements: Dict[str, Tensor]
    :return: The sum of the 'loc_2d' and 'loc_3d' losses.
    :rtype: Tensor
    """
    loss = requirements['loc_2d'] + requirements['loc_3d']

    return loss
