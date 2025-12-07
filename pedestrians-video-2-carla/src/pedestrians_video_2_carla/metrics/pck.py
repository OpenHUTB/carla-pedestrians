from typing import Callable, Dict, Literal, Type, Union
import torch
from torchmetrics import Metric

from pedestrians_video_2_carla.data.base.skeleton import get_common_indices
from pedestrians_video_2_carla.transforms.pose.normalization.hips_neck_extractor import HipsNeckExtractor
from pedestrians_video_2_carla.utils.tensors import get_bboxes, get_missing_joints_mask
from pedestrians_video_2_carla.data.base.skeleton import Skeleton
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON


class PCK(Metric):
    def __init__(
        self,
        dist_sync_on_step=False,
        input_nodes: Type[Skeleton] = CARLA_SKELETON,
        output_nodes: Type[Skeleton] = CARLA_SKELETON,
        mask_missing_joints: bool = True,
        key: Literal['projection_2d', 'projection_2d_transformed'] = 'projection_2d',
        threshold: float = 0.05,
        get_normalization_tensor: Union[Literal['hn', 'bbox'],
                                        Callable[[torch.Tensor], torch.Tensor]] = None,
    ) -> None:
        """
        Percentage of correct keypoints.
        Percentage of detections that fall within a normalized distance of the ground truth.
        """
        super().__init__(dist_sync_on_step=dist_sync_on_step)

        self.input_nodes = input_nodes
        self.output_nodes = output_nodes
        self.output_indices, self.input_indices = get_common_indices(
            input_nodes, output_nodes)

        self.key = key
        self.threshold = threshold

        if get_normalization_tensor == 'hn':
            self.get_normalization_tensor = self.hn_norm_dist
        elif callable(get_normalization_tensor):
            self.get_normalization_tensor = get_normalization_tensor
        else:
            self.get_normalization_tensor = self.bbox_norm_dist

        self.mask_missing_joints = mask_missing_joints
        self._input_hips = input_nodes.get_hips_point()
        if isinstance(self._input_hips, (list, tuple)):
            self._input_hips = None
        self.near_zero = 1e-5

        self.add_state("correct", default=torch.tensor(0), dist_reduce_fx="sum")
        self.add_state("total", default=torch.tensor(0), dist_reduce_fx="sum")

    def hn_norm_dist(self, sample: torch.Tensor) -> torch.Tensor:
        return HipsNeckExtractor(
            input_nodes=self.input_nodes
        ).get_shift_scale(sample)[1]

    def bbox_norm_dist(self, sample: torch.Tensor) -> torch.Tensor:
        """
        Compute the bounding box sizes.
        """
        bboxes = get_bboxes(sample)
        diffs = bboxes[..., 1, :] - bboxes[..., 0, :]
        return torch.linalg.norm(diffs, dim=-1, ord=2)

    def update(self, predictions: Dict[str, torch.Tensor], targets: Dict[str, torch.Tensor]):
        if self.mask_missing_joints and 'projection_2d' in targets:
            target_with_missing = targets['projection_2d'][:, :, self.input_indices]
            mask = get_missing_joints_mask(
                target_with_missing, self._input_hips, self.input_indices)
        else:
            # if there are only data after normalization, we can't ignore missing joints
            mask = torch.ones(
                targets[self.key][:, :, self.input_indices].shape[:-1], dtype=torch.bool)

        try:
            prediction = predictions[self.key][:, :, self.output_indices]
            target = targets[self.key][:, :, self.input_indices]

            assert prediction.shape == target.shape

            normalize = self.get_normalization_tensor(targets[self.key])
            mask[normalize < self.near_zero] = 0
            normalize[normalize < self.near_zero] = 1
            normalize = normalize[(slice(None),) * normalize.ndim +
                                  (None,) * (prediction.ndim - normalize.ndim)]

            norm_dist = torch.linalg.norm(
                (prediction - target) / normalize, dim=-1, ord=2)

            self.correct += torch.sum(norm_dist[mask] < self.threshold)
            self.total += norm_dist[mask].numel()
        except KeyError:
            pass

    def compute(self):
        return self.correct.float() / self.total
