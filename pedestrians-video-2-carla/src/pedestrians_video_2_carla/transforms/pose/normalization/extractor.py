from typing import Iterable, Tuple, Type, Union

import torch
from pedestrians_video_2_carla.data.base.skeleton import Skeleton


class Extractor(object):
    def __init__(self, input_nodes: Type[Skeleton], near_zero: float = 1e-5) -> None:
        self.input_nodes = input_nodes
        self.near_zero = near_zero

    def _point_to_tuple(self, point: Union[Skeleton, Iterable[Skeleton]]) -> Tuple[int]:
        if isinstance(point, Skeleton):
            return (point.value, )
        return tuple(p.value for p in point)

    def _get_shift_point(self, sample: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError()

    def _get_scale_point(self, sample: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError()

    def get_shift_scale(self, sample: torch.Tensor, return_scale_point: bool = False) -> torch.Tensor:
        shift_points = self._get_shift_point(sample)
        scale_points = self._get_scale_point(sample)

        scale = torch.linalg.norm(scale_points - shift_points,
                                  dim=shift_points.ndim - 1, ord=2)

        atleast_2d = (slice(None), ) * scale.ndim + (None, ) * 2
        scale = scale[atleast_2d[:(sample.ndim-2)]]

        if return_scale_point:
            return shift_points, scale, scale_points

        return shift_points, scale
