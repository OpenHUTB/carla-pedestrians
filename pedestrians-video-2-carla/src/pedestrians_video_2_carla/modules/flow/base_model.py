from enum import Enum
import logging
from typing import Dict, List, Tuple
import torch
from torch.optim.lr_scheduler import ReduceLROnPlateau, StepLR, CosineAnnealingWarmRestarts

from pedestrians_video_2_carla.data.base.skeleton import get_skeleton_name_by_type, get_skeleton_type_by_name


class BaseModel(torch.nn.Module):
    """
    Base model class that serves as an interface for all models.
    """

    def __init__(self,
                 prefix: str,
                 **kwargs
                 ):
        super().__init__()

        self._prefix = prefix
        self._hparams = {}

        self.enable_lr_scheduler = kwargs.get(f'{self._prefix}_enable_lr_scheduler')
        lr = kwargs.get(f'{self._prefix}_lr')

        if lr is None:
            self.learning_rate = 5e-2 if self.enable_lr_scheduler else 1e-4
        else:
            self.learning_rate = lr

        self.lr_scheduler_type = kwargs.get(
            f'{self._prefix}_scheduler_type', 'ReduceLROnPlateau')
        self.lr_scheduler_gamma = kwargs.get(f'{self._prefix}_scheduler_gamma', 0.98)
        self.lr_scheduler_step_size = kwargs.get(
            f'{self._prefix}_scheduler_step_size', 1)
        self.lr_scheduler_min_lr = kwargs.get(f'{self._prefix}_scheduler_min_lr', 1e-8)
        self.lr_scheduler_patience = kwargs.get(
            f'{self._prefix}_scheduler_patience', 50)
        self.lr_scheduler_cooldown = kwargs.get(
            f'{self._prefix}_scheduler_cooldown', 20)
        self.lr_weight_decay = kwargs.get(f'{self._prefix}_weight_decay', 1e-8)

        input_nodes = kwargs.get('input_nodes', None)
        if input_nodes is None:
            input_nodes = kwargs.get('data_nodes')
        self.input_nodes = get_skeleton_type_by_name(
            input_nodes) if isinstance(input_nodes, str) else input_nodes

    @property
    def hparams(self):
        base_hparams = {
            f'{self._prefix}_model_name': self.__class__.__name__,
            f'{self._prefix}_output_type': self.output_type.name,
            f'{self._prefix}_enable_lr_scheduler': self.enable_lr_scheduler,
            f'{self._prefix}_lr': self.learning_rate,
            f'{self._prefix}_scheduler_type': self.lr_scheduler_type,
            f'{self._prefix}_scheduler_gamma': self.lr_scheduler_gamma,
            f'{self._prefix}_scheduler_step_size': self.lr_scheduler_step_size,
            f'{self._prefix}_scheduler_min_lr': self.lr_scheduler_min_lr,
            f'{self._prefix}_scheduler_patience': self.lr_scheduler_patience,
            f'{self._prefix}_scheduler_cooldown': self.lr_scheduler_cooldown,
            f'{self._prefix}_weight_decay': self.lr_weight_decay,
            f'input_nodes': get_skeleton_name_by_type(self.input_nodes) if self.input_nodes is not None else None,
        }
        try:
            return {
                **base_hparams,
                **self._hparams
            }
        except AttributeError as e:
            logging.getLogger(__name__).warn('AttributeError: {}. Skipping non-base hparams.'.format(e))
            return base_hparams

    @property
    def output_type(self) -> Enum:
        raise NotImplementedError()

    @property
    def needs_targets(self) -> bool:
        return False

    @property
    def needs_confidence(self) -> bool:
        return False

    @property
    def needs_graph(self) -> bool:
        return False

    @staticmethod
    def add_model_specific_args(parent_parser, prefix: str):
        """
        Add model-specific arguments to the CLI args parser.
        """
        parser = parent_parser.add_argument_group("Base Model")
        parser.add_argument(
            f'--{prefix}_lr',
            default=None,
            type=float,
        )
        parser.add_argument(
            f'--{prefix}_enable_lr_scheduler',
            default=False,
            action='store_true',
        )
        parser.add_argument(
            f'--{prefix}_scheduler_type',
            default='ReduceLROnPlateau',
            type=str,
            choices=['ReduceLROnPlateau', 'StepLR', 'CosineAnnealingWarmRestarts'],  # TODO: Add more schedulers
        )
        parser.add_argument(
            f'--{prefix}_scheduler_gamma',
            default=0.98,
            type=float,
        )
        parser.add_argument(
            f'--{prefix}_scheduler_step_size',
            default=1,
            type=int,
        )
        parser.add_argument(
            f'--{prefix}_scheduler_min_lr',
            default=1e-8,
            type=float,
        )
        parser.add_argument(
            f'--{prefix}_scheduler_patience',
            default=50,
            type=int,
        )
        parser.add_argument(
            f'--{prefix}_scheduler_cooldown',
            default=20,
            type=int,
        )
        parser.add_argument(
            f'--{prefix}_weight_decay',
            default=1e-8,
            type=float,
        )
        # not prefixed because virtually all models use it
        # it is only important for the first model in the pipeline
        # also, data modules use it to determine the output type
        # We need to ensure it is added to the parser only once; TODO: is this the best way?
        if not 'input_nodes' in [action.dest for action in parser._actions]:
            parser.add_argument(
                '--input_nodes',
                type=get_skeleton_type_by_name,
                default=None,
                help='Input nodes for the model (data module output). If not specified, the model will use data_nodes.'
            )
        return parent_parser

    def configure_optimizers(self) -> Tuple[List[torch.optim.Optimizer], List[Dict[str, '_LRScheduler']]]:
        optimizer = torch.optim.AdamW(
            self.parameters(), lr=self.learning_rate, weight_decay=self.lr_weight_decay)

        config = {
            'optimizer': optimizer,
        }

        if self.enable_lr_scheduler:
            if self.lr_scheduler_type == 'ReduceLROnPlateau':
                lr_scheduler = {
                    'scheduler': ReduceLROnPlateau(
                        optimizer,
                        mode='min',
                        min_lr=self.lr_scheduler_min_lr,
                        factor=self.lr_scheduler_gamma,
                        patience=self.lr_scheduler_patience,
                        cooldown=self.lr_scheduler_cooldown,
                    ),
                    'interval': 'epoch',
                    'monitor': 'val_loss/primary'
                }
            elif self.lr_scheduler_type == 'StepLR':
                lr_scheduler = {
                    'scheduler': StepLR(
                        optimizer,
                        step_size=self.lr_scheduler_step_size,
                        gamma=self.lr_scheduler_gamma
                    ),
                }
            elif self.lr_scheduler_type == 'CosineAnnealingWarmRestarts':
                lr_scheduler = {
                    'scheduler': CosineAnnealingWarmRestarts(
                        optimizer,
                        T_0=self.lr_scheduler_step_size,
                        eta_min=self.lr_scheduler_min_lr,
                    ),
                }
            else:
                raise ValueError('Unknown lr scheduler type: {}'.format(
                    self.lr_scheduler_type))
            config['lr_scheduler'] = lr_scheduler

        return config

    def forward(self, x, *args, **kwargs):
        raise NotImplementedError()
