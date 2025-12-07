import torch

from .extractor import Extractor


class HipsNeckExtractor(Extractor):
    def _get_shift_point(self, sample: torch.Tensor) -> torch.Tensor:
        points = self._point_to_tuple(self.input_nodes.get_hips_point())
        return sample[..., points, :].mean(dim=-2)

    def _get_scale_point(self, sample: torch.Tensor) -> torch.Tensor:
        points = self._point_to_tuple(self.input_nodes.get_neck_point())
        return sample[..., points, :].mean(dim=-2)
