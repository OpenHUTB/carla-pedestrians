from typing import Dict

from torch import Tensor


def calculate_loss_loc_rot_3d(requirements: Dict[str, Tensor], **kwargs) -> Tensor:
    """
    Calculates the simple sum of the 'loc_3d' and 'rot_3d' losses.

    :param requirements: The dictionary containing the calculated 'loc_3d' and 'rot_3d'.
    :type requirements: Dict[str, Tensor]
    :return: The sum of the 'loc_3d' and 'rot_3d' losses.
    :rtype: Tensor
    """
    try:
        loss = requirements['loc_3d'] + requirements['rot_3d']
    except KeyError:
        return None

    return loss
