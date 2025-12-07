from typing import Any
from pedestrians_video_2_carla.utils.tensors import nan_to_zero
from .extractor import Extractor
import torch


class Normalizer(object):
    """
    Normalize each sample so that hips x,y = 0,0 and distance between shift point (e.g. hips) & dist point (e.g. neck) == 1.
    """

    def __init__(self, extractor: Extractor, near_zero: float = 1e-5) -> None:
        self.extractor = extractor
        self.__near_zero = near_zero
        self.__last_scale = None

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}(extractor={self.extractor.__class__.__name__})'

    def __call__(self, sample: torch.Tensor, dim=2, *args: Any, **kwargs: Any) -> torch.Tensor:
        shift, scale = self.extractor.get_shift_scale(sample[..., 0:dim])

        normalized_sample = torch.empty_like(sample)
        normalized_sample[..., 0:dim] = (sample[..., 0:dim] -
                                         torch.unsqueeze(shift, -2)) / scale[(..., ) + (None, ) * 2]

        if dim == 2 and sample.shape[-1] > 2:
            normalized_sample[..., 2] = sample[..., 2]

        normalized_sample = nan_to_zero(normalized_sample)

        # if confidence is 0, we will assume the point overlaps with hips
        # so that values that were originally 0,0 (not detected)
        # do not skew the values range
        if dim == 2 and normalized_sample.shape[-1] > 2:
            normalized_sample[..., 0:2] = normalized_sample[..., 0:2].where(
                normalized_sample[..., 2:] >= self.__near_zero, torch.tensor(0.0, device=normalized_sample.device))

        self.__last_scale = scale
        self.__last_shift = shift
        return normalized_sample

    @property
    def scale(self) -> torch.Tensor:
        return self.__last_scale.clone()

    @property
    def shift(self) -> torch.Tensor:
        return self.__last_shift.clone()
