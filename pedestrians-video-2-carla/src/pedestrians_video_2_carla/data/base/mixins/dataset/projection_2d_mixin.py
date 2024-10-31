import logging
import math
from typing import Callable, Dict, Iterable, Literal, Optional, Tuple, Union
from pedestrians_video_2_carla.data.base.base_transforms import BaseTransforms
from pedestrians_video_2_carla.transforms.pose.augmentation.augment_pose import AugmentPose
from pedestrians_video_2_carla.transforms.pose.augmentation.random_flip import RandomFlip
from pedestrians_video_2_carla.transforms.pose.augmentation.random_rotation import RandomRotation
from pedestrians_video_2_carla.utils.argparse import boolean, boolean_or_float, flat_args_as_list_arg, list_arg_as_flat_args

import torch

from pedestrians_video_2_carla.utils.tensors import get_bboxes, nan_to_zero


class Projection2DMixin:
    def __init__(self,
                 transform: Optional[Callable[[torch.Tensor], torch.Tensor]] = None,
                 noise: Optional[Literal['zero', 'gaussian', 'uniform']] = 'zero',
                 noise_param: Optional[float] = 1.0,
                 augment_flip: Optional[Union[bool, float]] = False,
                 augment_rotate: Optional[Union[bool, float]] = False,
                 **kwargs):
        """
        Mixing to handle common operations on 2D input data.
        Optional seed can be provided for torch.Generator handling
        noise and missing points, HOWEVER to ensure reproducibility
        important DataModule parameters like batch_size and num_workers
        need to also be consistent across calls.
        """
        super().__init__(**kwargs)

        missing_joint_probabilities = flat_args_as_list_arg(
            kwargs, 'missing_joint_probabilities')

        if len(missing_joint_probabilities) == 0:
            self.missing_joint_probabilities = (0.0,)
        elif len(missing_joint_probabilities) == 1:
            self.missing_joint_probabilities = missing_joint_probabilities * self.num_data_joints
        elif len(missing_joint_probabilities) == self.num_data_joints:
            self.missing_joint_probabilities = missing_joint_probabilities
        else:
            raise ValueError(
                f'Missing joint probabilities must have length 1 or {self.num_data_joints}, got {len(missing_joint_probabilities)}.')

        self.noise = noise
        self.noise_param = noise_param

        if kwargs.get('overfit_batches', 0):
            self.generator = None
        else:
            self.generator = torch.Generator()

        # TODO: this really should be called 'normalization'
        self.transform = transform
        self.augmentation = AugmentPose(
            nodes=self.data_nodes,
            flip=augment_flip,
            rotate=augment_rotate,
            generator=self.generator
        ) if (augment_flip or augment_rotate) else None

    @property
    def needs_missing_points(self) -> bool:
        return any(self.missing_joint_probabilities)

    @property
    def needs_noise(self) -> bool:
        return self.noise is not None and self.noise != 'zero'

    @property
    def needs_deform(self) -> bool:
        return self.needs_missing_points or self.needs_noise

    @staticmethod
    def add_cli_args(parser):
        parser = list_arg_as_flat_args(
            parser, 'missing_joint_probabilities', 26, None, float,
            help="""
                Probability that a joint/node will be missing ("not detected") in a skeleton in a frame.
                Missing nodes are selected separately for each frame. If multiple values are provided,
                they are assumed to be probabilities for each joint/node separately and in the same order
                as in `data_nodes` Skeleton.
            """
        )
        parser.add_argument(
            "--noise",
            type=str,
            default='zero',
            choices=['zero', 'gaussian', 'uniform'],
            help="""
                Noise to add to the skeleton. Noise is added to each joint/node separately
                and before the missing_joint_probabilities and any transformation
                is applied (before normalization, too!).
                'zero' (default) means no noise.
            """
        )
        parser.add_argument(
            "--noise_param",
            type=float,
            default=1.0,
            metavar='STD or SCALE',
            help="""
                Standard deviation of the Gaussian noise to add to the skeleton OR
                Scale of the uniform noise to add to the skeleton.
            """
        )
        parser.add_argument(
            "--augment_flip",
            type=boolean_or_float,
            default=False,
            help="""
                Randomly flip the skeleton horizontally. If a float is provided,
                it is assumed to be the probability of flipping (defaults to 0.5).
            """
        )
        parser.add_argument(
            "--augment_rotate",
            type=boolean_or_float,
            default=False,
            help="""
                Randomly rotate the skeleton around the bounding box center. If a float is provided,
                it is assumed to be the max +/- angle in deg (defaults to 10deg).
            """
        )
        return parser

    @staticmethod
    def extract_hparams(kwargs) -> dict:
        return {
            'missing_joint_probabilities': flat_args_as_list_arg(kwargs, 'missing_joint_probabilities'),
            'noise': kwargs.get('noise', 'zero'),
            'noise_param': kwargs.get('noise_param', 1.0),
            'augment_flip': kwargs.get('augment_flip', False),
            'augment_rotate': kwargs.get('augment_rotate', False),
        }

    def apply_deform(self, projection_2d: torch.Tensor) -> torch.Tensor:
        """
        Deforms the data by adding noise and missing points.
        Returns a clone of the original data.
        """
        deformed_projection_2d = projection_2d[..., :2].clone()

        if self.needs_noise:
            if self.noise == 'gaussian':
                noise = torch.normal(mean=0.0, std=self.noise_param,
                                     size=deformed_projection_2d.shape,
                                     generator=self.generator,
                                     dtype=deformed_projection_2d.dtype,
                                     device=deformed_projection_2d.device)
            elif self.noise == 'uniform':
                # TODO: this is not fully symmetric, since rand generates numbers from [0,1) interval
                noise = torch.rand_like(deformed_projection_2d, generator=self.generator) * \
                    self.noise_param - self.noise_param / 2.0
            else:
                raise ValueError('Unknown noise type: {}'.format(self.noise))

            deformed_projection_2d += noise

        if self.needs_missing_points:
            missing_indices = torch.rand(
                deformed_projection_2d.shape[:-1], generator=self.generator) < torch.tensor(
                    self.missing_joint_probabilities, dtype=deformed_projection_2d.dtype,
                    device=deformed_projection_2d.device)
            deformed_projection_2d[missing_indices] = torch.zeros(
                deformed_projection_2d.shape[-1:], device=deformed_projection_2d.device)

        if projection_2d.shape[-1] > 2:
            return torch.cat((deformed_projection_2d, projection_2d[..., 2:]), dim=-1)

        return deformed_projection_2d

    @property
    def needs_transform(self) -> bool:
        return self.transform is not None

    def apply_transform(self, projection_2d: torch.Tensor) -> torch.Tensor:
        """
        Transforms the data by applying a transformation.
        Returns a clone of the original data.
        """
        # clone data before user transform
        transformed_projection_2d = projection_2d.clone()

        if self.needs_transform:
            transformed_projection_2d = self.transform(transformed_projection_2d)

        return transformed_projection_2d

    @property
    def needs_augmentation(self) -> bool:
        return self._is_training and self.augmentation is not None

    def apply_augmentation(self, projection_2d: torch.Tensor, targets: Dict, meta: Dict) -> torch.Tensor:
        """
        Applies augmentation steps i.e. transforms that will carry over to the ground truth,
        like flipping or rotating. It does not apply the missing points or noise.
        Returns a clone of the original data.
        """
        augmented_projection_2d = projection_2d.clone()
        new_targets = {}

        if self.needs_augmentation:
            augmented_projection_2d, new_targets = self.augmentation(
                augmented_projection_2d, targets, meta)

        return augmented_projection_2d, new_targets

    def process_projection_2d(self, projection_2d: torch.Tensor, clip_targets: Dict[str, torch.Tensor], meta: Dict[str, Iterable]) -> torch.Tensor:
        """
        Deforms the data by adding noise and missing points and then applies a transformation.
        Returns a clone of the original data suitable for training and a dict of the relevant targets.
        Targets never have confidence values, only (x,y) coordinates.
        """
        augmented_projection_2d, targets = self.apply_augmentation(
            projection_2d, clip_targets, meta)
        deformed_projection_2d = self.apply_deform(augmented_projection_2d)
        transformed_deformed_projection_2d = self.apply_transform(
            deformed_projection_2d)
        transformed_projection_2d = self.apply_transform(augmented_projection_2d)

        targets['projection_2d'] = augmented_projection_2d[..., :2]

        if self.needs_deform:
            targets['projection_2d_deformed'] = deformed_projection_2d[..., :2]

        if self.needs_transform:
            targets['projection_2d_transformed'] = transformed_projection_2d[..., :2]
            targets['projection_2d_shift'] = self.transform.shift
            targets['projection_2d_scale'] = self.transform.scale

        return transformed_deformed_projection_2d, targets
