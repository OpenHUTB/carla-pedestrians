from typing import Any, Dict, List

import torch
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.carla.reference import get_absolute_tensors, get_projections
from . import (DeNormalizer, Extractor, Normalizer)
from .hips_neck_extractor import HipsNeckExtractor
from torch import Tensor

AGE_MAPPINGS = {
    # available in CARLA
    'adult': 'adult',
    'child': 'child',
    # substitutions
    'senior': 'adult',
    'young': 'child',
    float('nan'): 'adult',
    'nan': 'adult'
}

GENDER_MAPPINGS = {
    # available in CARLA
    'female': 'female',
    'male': 'male',
    # substitutions
    'neutral': 'female',
    float('nan'): 'female',
    'nan': 'female'
}


class ReferenceSkeletonsDeNormalizer(DeNormalizer):
    def __init__(self,
                 extractor: Extractor = None
                 ) -> None:
        if extractor is None:
            extractor = HipsNeckExtractor(CARLA_SKELETON)
        self._extractor = extractor
        self._normalizer = Normalizer(self._extractor)

    def from_projection(self, frames: Tensor, meta: Dict[str, List[Any]], autonormalize: bool = False) -> Tensor:
        """
        Denormalizes (after optional autonormalization) the 2D pose coordinates,
        applying the shift and scale of the reference skeleton. By default uses CARLA skeletons.

        :param frames: (B, L, P, 2) tensor of 2D pose coordinates.
        :type frames: Tensor
        :param meta: Meta data for each clip, containing the age and gender to be used for denormalization.
        :type meta: Dict[str, List[Any]]
        :param autonormalize: Should data be normalized first, defaults to False
        :type autonormalize: bool, optional
        :return: Denormalized 2D pose coordinates.
        :rtype: Tensor
        """
        if autonormalize:
            frames = self._normalizer(frames, dim=2)

        reference_projections = get_projections(frames.device, as_dict=True)

        frame_projections = torch.stack([
            reference_projections[(AGE_MAPPINGS[age], GENDER_MAPPINGS[gender])]
            for (age, gender) in zip(meta.get('age', ['adult']*len(frames)), meta.get('gender', ['female']*len(frames)))
        ], dim=0)

        return self.from_reference(self._extractor, frame_projections[..., :2])(frames, dim=2)

    def from_abs(self, frames: Tensor, meta: Dict[str, List[Any]], autonormalize: bool = False) -> Tensor:
        """
        Denormalizes (after optional autonormalization) the 3D "absolute" pose coordinates,
        applying the shift and scale of the reference skeleton. By default uses CARLA skeletons.

        :param frames: (B, L, P, 3) tensor of 3D pose coordinates.
        :type frames: Tensor
        :param meta: Meta data for each clip, containing the age and gender to be used for denormalization.
        :type meta: Dict[str, List[Any]]
        :param autonormalize: Should data be normalized first, defaults to False
        :type autonormalize: bool, optional
        :return: Denormalized 3D pose coordinates.
        :rtype: Tensor
        """
        if autonormalize:
            frames = self._normalizer(frames, dim=3)

        reference_abs = get_absolute_tensors(frames.device, as_dict=True)

        frame_abs = torch.stack([
            reference_abs[(AGE_MAPPINGS[age], GENDER_MAPPINGS[gender])][0]
            for (age, gender) in zip(meta['age'], meta['gender'])
        ], dim=0)

        return self.from_reference(self._extractor, frame_abs)(frames, dim=3)
