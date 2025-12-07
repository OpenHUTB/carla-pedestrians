from typing import Type
import torch
from .hips_neck_extractor import HipsNeckExtractor
from .bbox_extractor import BBoxExtractor
from .extractor import Extractor
from pedestrians_video_2_carla.data.base.skeleton import Skeleton


class HipsNeckBBoxFallbackExtractor(Extractor):
    def __init__(self, input_nodes: Type[Skeleton], near_zero: float = 0.00001) -> None:
        super().__init__(input_nodes, near_zero)

        self.__hn_extractor = HipsNeckExtractor(input_nodes, near_zero)
        self.__bb_extractor = BBoxExtractor(input_nodes, near_zero)

        self.__fallback_x_shift = 0.0
        self.__fallback_y_shift = -0.1059
        self.__fallback_scale = 0.5748

    def get_shift_scale(self, sample: torch.Tensor) -> torch.Tensor:
        hn_shift, hn_scale, hn_neck = self.__hn_extractor.get_shift_scale(
            sample, return_scale_point=True)
        bb_shift, bb_scale = self.__bb_extractor.get_shift_scale(sample)

        missing_hips = torch.all(hn_shift < self.near_zero, dim=-1)
        out_shift = hn_shift.clone()
        if torch.any(missing_hips):
            out_shift[missing_hips][:, 0] = bb_shift[missing_hips][:, 0] + \
                (bb_scale[missing_hips] * self.__fallback_x_shift)
            out_shift[missing_hips][:, 1] = bb_shift[missing_hips][:, 1] + \
                (bb_scale[missing_hips] * self.__fallback_y_shift)

        missing_neck = torch.all(hn_neck < self.near_zero, dim=-1)
        out_scale = hn_scale.clone()
        if torch.any(missing_hips):
            out_scale[missing_hips] = bb_scale[missing_hips] * self.__fallback_scale
        if torch.any(missing_neck):
            out_scale[missing_neck] = bb_scale[missing_neck] * self.__fallback_scale

        return out_shift, out_scale
