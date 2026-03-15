from typing import Dict, Iterable, Literal, Tuple, Type
from torchmetrics import Metric
import torch
import math
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.base.skeleton import Skeleton, get_common_indices


class MissingJointsRatio(Metric):
    """
    Missing Joints Ratio.

    This metric is computed using 'projection_2d' ground truth and predictions.
    """

    def __init__(self,
                 dist_sync_on_step=False,
                 input_nodes: Type[Skeleton] = CARLA_SKELETON,
                 output_nodes: Type[Skeleton] = CARLA_SKELETON,
                 report_per_joint=False,
                 ):
        super().__init__(dist_sync_on_step=dist_sync_on_step)

        self.input_nodes = input_nodes
        self.output_nodes = output_nodes
        self.report_per_joint = report_per_joint

        self.output_indices, self.input_indices = get_common_indices(
            input_nodes, output_nodes)
        self.output_num_joints = torch.arange(len(output_nodes))[
            self.output_indices].shape[0]

        if self.report_per_joint:
            self.items = self._items

        self.add_state("present_joints", default=torch.zeros(
            (self.output_num_joints,)), dist_reduce_fx="sum")
        self.add_state("total", default=torch.tensor(0), dist_reduce_fx="sum")

    def update(self, predictions: Dict[str, torch.Tensor], targets: Dict[str, torch.Tensor]):
        try:
            prediction = predictions['projection_2d'][:, :, self.output_indices]
            self.present_joints += prediction.all(dim=-
                                                  1).sum(dim=tuple(range(prediction.ndim-2)))
            self.total += math.prod(prediction.shape[:-2])
        except KeyError:
            # MJR only makes sense if we have 'absolute' predictions
            pass

    def compute(self):
        # compute final result
        per_joint = 1.0 - self.present_joints.float() / self.total
        mean_mjr = 1.0 - self.present_joints.sum() / (self.output_num_joints * self.total)

        if self.report_per_joint:
            return {
                'mean': mean_mjr,
                "per_joint": self.__map_joints(
                    per_joint, self.output_nodes, self.output_indices)
            }

        return mean_mjr

    def __map_joints(self, joint_results: Iterable[float], nodes: Type[Skeleton], indices: Iterable[int]):
        if isinstance(indices, slice):
            indices = range(len(nodes))
        return {
            nodes(i).name: joint_results[i]
            for i in indices
        }

    def _items(self) -> Iterable[Tuple[str, torch.Tensor]]:
        return {
            'mean': 0.0,
            "per_joint": self.__map_joints(
                self.present_joints, self.output_nodes, self.output_indices)
        }
