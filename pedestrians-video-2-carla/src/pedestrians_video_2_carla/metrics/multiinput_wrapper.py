from typing import Any, Dict, Type
import torch
from torchmetrics import Metric

from pedestrians_video_2_carla.data.base.skeleton import Skeleton, get_common_indices
from pedestrians_video_2_carla.utils.tensors import get_missing_joints_mask


class MultiinputWrapper(Metric):
    def __init__(
        self,
        base_metric,
        pred_key,
        target_key,
        input_nodes: Type[Skeleton],
        output_nodes: Type[Skeleton],
        mask_missing_joints: bool = True,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.base_metric = base_metric

        # workaround so that MetricCollection is not merging the computing groups
        # TODO: can it be done in a better way?
        self.add_state(base_metric.__class__.__name__,
                       default=torch.tensor(0.0), dist_reduce_fx="sum")

        self.pred_key = pred_key
        self.target_key = target_key

        if input_nodes is None and output_nodes is None:
            # this metric is not calculated on joints
            self._input_indices = None
            self._output_indices = None
            self._input_hips = None
            self._mask_missing_joints = False
        else:
            self._output_indices, self._input_indices = get_common_indices(
                input_nodes, output_nodes)

            self._mask_missing_joints = mask_missing_joints
            self._input_hips = input_nodes.get_hips_point()
            if isinstance(self._input_hips, (list, tuple)):
                self._input_hips = None

    def update(self, predictions: Dict[str, torch.Tensor], targets: Dict[str, torch.Tensor]):
        if self._input_indices is None and self._output_indices is None:
            # this metric is not calculated on joints
            return self.base_metric.update(
                predictions[self.pred_key],
                torch.atleast_1d(targets[self.target_key])
            )

        common_pred = predictions[self.pred_key][..., self._output_indices, :]
        common_gt = targets[self.target_key][..., self._input_indices, :]

        if self._mask_missing_joints:
            mask = get_missing_joints_mask(
                common_gt, self._input_hips, self._input_indices)
            common_pred = common_pred[mask]
            common_gt = common_gt[mask]

        return self.base_metric.update(common_pred, common_gt)

    def compute(self):
        return self.base_metric.compute()

    def reset(self) -> None:
        self.base_metric.reset()
