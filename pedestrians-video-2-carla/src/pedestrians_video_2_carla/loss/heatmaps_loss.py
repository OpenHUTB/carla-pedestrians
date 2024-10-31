from typing import Dict, Type

import torch
from pedestrians_video_2_carla.loss.base_pose_loss import BasePoseLoss
from torch import Tensor
from pedestrians_video_2_carla.data.base.skeleton import Skeleton


class HeatmapsLoss(BasePoseLoss):
    def __init__(self,
                 input_nodes: Type[Skeleton],
                 output_nodes: Type[Skeleton],
                 **kwargs):
        super().__init__(**{
            **kwargs,
            'input_nodes': input_nodes,
            'output_nodes': output_nodes,
            # always force sum_per_frame to True
            # TODO: this us to match UniPose, but should probably be configurable in the future
            'sum_per_frame': True
        })

        if self._input_indices != slice(None):
            # add bg headmap index to the end
            self._input_indices = tuple(self._input_indices) + (len(input_nodes) - 1,)
            self._output_indices = tuple(
                self._output_indices) + (len(output_nodes) - 1,)

    def _flatten_heatmaps(self,
                          heatmaps: Tensor) -> Tensor:
        flat_heatmaps = heatmaps.view((*heatmaps.shape[:-2], -1))

        # additionally, we need to move the bg heatmap to the end
        # so that common indices work correctly
        flat_heatmaps = torch.cat((flat_heatmaps[..., 1:], flat_heatmaps[..., 0:1]), -1)

        return flat_heatmaps

    def _extract_gt_targets(self,
                            targets: Dict[str, Tensor],
                            **kwargs) -> Tensor:
        return self._flatten_heatmaps(targets['heatmaps'])

    def _extract_predicted_targets(self,
                                   heatmaps: Tensor,
                                   **kwargs) -> Tensor:
        return self._flatten_heatmaps(heatmaps)
