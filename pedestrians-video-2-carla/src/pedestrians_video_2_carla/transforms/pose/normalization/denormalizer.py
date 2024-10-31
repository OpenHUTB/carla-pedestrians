from typing import Any, Callable
import torch

from .extractor import Extractor


class DeNormalizer(object):
    """
    Denormalize each sample based on provided scale & shift.
    """

    def __call__(self, sample: torch.Tensor, scale: torch.Tensor, shift: torch.Tensor, dim=2, *args: Any, **kwargs: Any) -> torch.Tensor:
        denormalized_sample = torch.empty_like(sample)

        # match dims
        # scale[:, as_many_dims_as_needed].ndim == sample.ndim
        d = scale[(slice(None), ) * scale.ndim + (None, ) * (sample.ndim - scale.ndim)]
        # shift[:, as_many_dims_as_needed, :].ndim == sample.ndim
        h = shift[(slice(None), ) * scale.ndim + (None, ) *
                  (sample.ndim - shift.ndim) + (slice(None), )]

        denormalized_sample[..., 0:dim] = (sample[..., 0:dim] * d) + h

        if dim == 2 and sample.shape[-1] > 2:
            denormalized_sample[..., 2] = sample[..., 2]

        return denormalized_sample

    @staticmethod
    def from_reference(extractor: Extractor, reference: torch.Tensor) -> Callable:
        shift, scale = extractor.get_shift_scale(reference)
        instance = DeNormalizer()
        return lambda sample, dim=2: instance(sample, scale, shift, dim)
