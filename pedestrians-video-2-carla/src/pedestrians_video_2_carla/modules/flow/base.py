
import platform
from typing import Any, Dict, List, Tuple, Union
from types import FunctionType

import pytorch_lightning as pl
import torch
import torchmetrics
from pedestrians_video_2_carla.data.base.base_transforms import BaseTransforms
from pedestrians_video_2_carla.loss.base_pose_loss import BasePoseLoss
from pedestrians_video_2_carla.modules.flow.output_types import TrajectoryModelOutputType
from pedestrians_video_2_carla.loss import LossModes
from pedestrians_video_2_carla.modules.movements.movements import \
    MovementsModel, MovementsModelOutputType
from pedestrians_video_2_carla.modules.movements.zero import ZeroMovements
from pedestrians_video_2_carla.modules.trajectory.trajectory import \
    TrajectoryModel
from pedestrians_video_2_carla.modules.trajectory.zero import ZeroTrajectory
from pedestrians_video_2_carla.utils.argparse import DictAction, flat_args_as_list_arg, list_arg_as_flat_args
from pytorch_lightning.loggers.tensorboard import TensorBoardLogger
from pytorch_lightning.utilities import rank_zero_only
from torch import Tensor
try:
    from torch_geometric.data import Batch
except ImportError:
    Batch = None

from torchmetrics import MetricCollection
from pedestrians_video_2_carla.utils.argparse import boolean
from pedestrians_video_2_carla.utils.printing import print_metrics


class LitBaseFlow(pl.LightningModule):
    """
    Base LightningModule - all other LightningModules should inherit from this.
    It contains movements model, trajectory model, projection layer, loss modes handling and video logging.

    Movements & Trajectory models should implement forward() and configure_optimizers() method.
    If they use additional hyperparameters, they should also set self._hparams dict
    in __init__() and (optionally) override add_model_specific_args() method.
    """

    def __init__(
        self,
        movements_model: MovementsModel = None,
        trajectory_model: TrajectoryModel = None,
        loss_modes: List[LossModes] = None,
        loss_weights: Dict[str, Tensor] = None,
        mask_missing_joints: bool = True,
        **kwargs
    ):
        super().__init__()

        # default layers
        if movements_model is None:
            movements_model = ZeroMovements()
        self.movements_model = movements_model

        # TODO: extract trajectory model from Base into PoseLifting flow
        if trajectory_model is None:
            trajectory_model = ZeroTrajectory()
        self.trajectory_model = trajectory_model

        self.mask_missing_joints = mask_missing_joints

        # losses
        if loss_weights is None:
            loss_weights = {}
        self.loss_weights = loss_weights

        if loss_modes is None or len(loss_modes) == 0:
            loss_modes = [LossModes.loc_2d]
        self._loss_modes = [LossModes[lm] if isinstance(
            lm, str) else lm for lm in loss_modes]

        modes = []
        for mode in self._loss_modes:
            if len(mode.value) > 2:
                for k in mode.value[2]:
                    modes.append(LossModes[k])
            modes.append(mode)
        # TODO: resolve requirements chain and put modes in correct order, not just 'hopefully correct' one
        # TODO: convert all losses to classes
        self._losses_to_calculate = [
            (mode.name, mode.value[0](
                criterion=mode.value[1],
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes,
                mask_missing_joints=self.mask_missing_joints,
                loss_params=flat_args_as_list_arg(kwargs, 'loss_params'),
            ), None, mode.value[2] if len(mode.value) > 2 else tuple()) if not isinstance(mode.value[0], FunctionType) else (mode.name, *mode.value)
            for mode in list(dict.fromkeys(modes))
        ]

        kwargs_transform = kwargs.get('transform', BaseTransforms.hips_neck)
        if isinstance(kwargs_transform, str):
            kwargs_transform = BaseTransforms[kwargs_transform.lower()]
        self._outputs_key = 'projection_2d_transformed' if kwargs_transform != BaseTransforms.none else 'projection_2d'
        self._crucial_keys = self._get_crucial_keys()

        # default metrics
        self.metrics = MetricCollection(self.get_metrics())

        self.save_hyperparameters({
            'host': platform.node(),
            'loss_modes': [mode.name for mode in self._loss_modes],
            'loss_weights': self.loss_weights,
            **self.movements_model.hparams,
            **self.trajectory_model.hparams,
        })

    def _get_crucial_keys(self):
        return [
            self._outputs_key,
        ]

    @property
    def outputs_key(self) -> str:
        return self._outputs_key

    @property
    def crucial_keys(self) -> List[str]:
        return self._crucial_keys.copy()

    @classmethod
    def get_available_models(cls) -> Dict[str, Dict[str, torch.nn.Module]]:
        """
        Returns a dictionary with available/required models.
        """
        return {}

    @classmethod
    def get_default_models(cls) -> Dict[str, torch.nn.Module]:
        """
        Returns a dictionary with default models.
        """
        return {}

    def get_initial_metrics(self) -> Dict[str, torchmetrics.Metric]:
        """
        Returns metrics to calculate on each validation batch at the beginning of the training.
        They will be added to the metrics returned by get_metrics() method.
        """
        return {}

    def get_metrics(self) -> Dict[str, torchmetrics.Metric]:
        """
        Returns metrics to calculate on each batch. They should take into account
        the self.mask_missing_joints flag.
        """
        return {}

    def configure_optimizers(self):
        movements_optimizers = self.movements_model.configure_optimizers()
        trajectory_optimizers = self.trajectory_model.configure_optimizers()

        if 'optimizer' not in movements_optimizers:
            movements_optimizers = None

        if 'optimizer' not in trajectory_optimizers:
            trajectory_optimizers = None

        return [opt for opt in [movements_optimizers, trajectory_optimizers] if opt is not None]

    @staticmethod
    def add_model_specific_args(parent_parser):
        """
        Add model specific commandline arguments. 

        By default, this method adds parameters for projection layer and loss modes.
        If overriding, remember to call super().
        """

        parser = parent_parser.add_argument_group("BaseFlow Module")
        parser.add_argument(
            '--mask_missing_joints',
            type=boolean,
            default=True,
            help='Mask missing ground truth joints when calculating loss and metrics.'
        )
        parser.add_argument(
            '--loss_modes',
            help="""
                Set loss modes to use in the preferred order.
                Choices: {}.
                Default: ['loc_2d']
                """.format(
                set(LossModes.__members__.keys())),
            metavar="MODE",
            default=[],
            choices=list(LossModes),
            nargs="+",
            action="extend",
            type=LossModes.__getitem__
        )
        parser.add_argument(
            '--loss_weights',
            help="""
                Set loss weights for each loss part when using weighted loss.
                Example: --loss_weights common_loc_2d=1.0 loc_3d=1.0 rot_3d=3.0
                Default: ANY_LOSS=1.0
                """,
            metavar="WEIGHT",
            default={},
            nargs="+",
            action=DictAction,
            value_type=float
        )

        parser = list_arg_as_flat_args(parser, 'loss_params', 26, None, float)

        return parent_parser

    @property
    def needs_graph(self):
        return self.movements_model.needs_graph

    @property
    def needs_heatmaps(self):
        return getattr(self.movements_model, 'needs_heatmaps', False)

    @property
    def needs_confidence(self):
        return getattr(self.movements_model, 'needs_confidence', False)

    @rank_zero_only
    def on_fit_start(self) -> None:
        initial_metrics = self._calculate_initial_metrics()
        self._update_hparams(initial_metrics)

    def _unwrap_batch(self, batch):
        if isinstance(batch, (Tuple, List)):
            return (*batch, None, None)

        if isinstance(batch, Batch):
            return (
                batch.x,
                {
                    k.replace('targets_', ''): batch.get(k)
                    for k in batch.keys
                    if k.startswith('targets_')
                },
                batch.meta,
                batch.edge_index,
                batch.batch
            )

    def _fix_dimensions(self, data: torch.Tensor):
        if self.needs_graph:
            s = data.shape
            cl = self.trainer.datamodule.clip_length
            if len(s) < 2 or s[1] != cl:
                # batch size can be smaller than specified in datamodule in the last batch
                bs = int(s[0] / cl)
                return data.view((bs, cl, *s[1:]))
        return data

    def _calculate_initial_metrics(self) -> Dict[str, float]:
        dl = self.trainer.datamodule.val_dataloader()
        if dl is None:
            return {}

        initial_metrics = MetricCollection({
            **self.get_metrics(),
            **self.get_initial_metrics()
        }).to(self.device)

        if not len(initial_metrics):
            return {}

        for batch in dl:
            (inputs, targets, *_) = self._unwrap_batch(batch)
            if 'projection_2d_deformed' in targets:
                # this will be true if there are artificial missing joints
                key = 'projection_2d_deformed'
            else:
                # this will measure it for ground truth; usually should be 0s/1s
                # but it will serve as a sanity check and can be something else
                # when mixed datasets are used
                key = 'projection_2d'

            d_targets = {k: v.to(self.device) for k, v in targets.items()}

            initial_metrics.update({
                'projection_2d': d_targets[key],
                'projection_2d_transformed': inputs.to(self.device)
            }, d_targets)

        results = initial_metrics.compute()
        unwrapped = {
            k: v.item()
            for k, v in self._unwrap_nested_metrics(results, ['initial']).items()
        }

        if len(unwrapped) > 0:
            print_metrics(unwrapped, header='Initial metrics:')

        return unwrapped

    def _update_hparams(self, initial_metrics: Dict[str, float] = None):
        # We need to manually add the datamodule hparams,
        # because the merge is automatically handled only for initial_hparams
        # additionally, store info on train set size for easy access

        sizes = {}
        subsets = ['train', 'val', 'test']
        for set_name in subsets:
            try:
                set_size = len(getattr(self.trainer.datamodule, f'{set_name}_set'))
            except (AttributeError, TypeError):
                set_size = None

            limit = getattr(self.trainer, f'limit_{set_name}_batches', None)
            if limit is not None:
                if isinstance(limit, int):
                    set_size = limit * self.trainer.datamodule.batch_size
                elif set_size is not None:
                    set_size = int(limit * set_size)

            if set_size is not None:
                sizes[f'used_{set_name}_set_size'] = set_size

        additional_config = {
            **self.trainer.datamodule.hparams,
            **sizes,
            **(initial_metrics or {}),
        }

        self.hparams.update(additional_config)

        if len(self.trainer.loggers) and isinstance(self.trainer.loggers[0], TensorBoardLogger):
            # TensorBoard requires 'special' updating to log multiple metrics
            self.trainer.loggers[0].log_hyperparams(
                self.hparams,
                self._unwrap_nested_metrics(self.metrics, ['hp'], nans=True)
            )
        else:
            self.trainer.loggers[0].log_hyperparams(self.hparams)

    def _unwrap_nested_metrics(self, items: Union[Dict, float], keys: List[str], zeros: bool = False, nans: bool = False):
        r = {}
        if hasattr(items, 'items'):
            for k, v in items.items():
                r.update(self._unwrap_nested_metrics(v, keys + [k], zeros, nans))
        else:
            r['/'.join(keys)] = 0.0 if zeros else (float('nan') if nans else items)
        return r

    def _on_batch_start(self, batch, batch_idx):
        pass

    def on_train_batch_start(self, batch, batch_idx, *args, **kwargs):
        self._on_batch_start(batch, batch_idx)

    def on_validation_batch_start(self, batch, batch_idx, *args, **kwargs):
        self._on_batch_start(batch, batch_idx)

    def on_test_batch_start(self, batch, batch_idx, *args, **kwargs):
        self._on_batch_start(batch, batch_idx)

    def predict_step(self, batch, batch_idx):
        self._on_batch_start(batch, batch_idx)
        return self(batch)

    def training_step(self, batch, batch_idx):
        return self._step(batch, batch_idx, 'train')

    def validation_step(self, batch, batch_idx):
        return self._step(batch, batch_idx, 'val')

    def test_step(self, batch, batch_idx):
        return self._step(batch, batch_idx, 'test')

    def validation_step_end(self, outputs):
        self._eval_step_end(outputs, 'val')

    def test_step_end(self, outputs):
        self._eval_step_end(outputs, 'test')

    def training_epoch_end(self, outputs: Any) -> None:
        to_log = {}

        if hasattr(self.movements_model, 'training_epoch_end'):
            to_log.update(self.movements_model.training_epoch_end(outputs))

        if hasattr(self.trajectory_model, 'training_epoch_end'):
            to_log.update(self.trajectory_model.training_epoch_end(outputs))

        if len(to_log) > 0:
            batch_size = len(outputs[0]['preds'][self._outputs_key])
            self.log_dict(to_log, batch_size=batch_size)

    def forward(self, batch, *args, **kwargs) -> Any:
        (frames, targets, meta, edge_index, batch_vector) = self._unwrap_batch(batch)
        return self._inner_step(frames, targets, edge_index, batch_vector), meta

    def _step(self, batch, batch_idx, stage):
        (frames, targets, meta, edge_index, batch_vector) = self._unwrap_batch(batch)
        sliced = self._inner_step(frames, targets, edge_index, batch_vector)

        loss_dict = self._calculate_lossess(stage, len(frames), sliced, meta)

        # after losses are calculated, we will not need any gradients anymore
        # so detach everything in sliced for the rest of the processing
        sliced = {k: v.detach() if isinstance(v, torch.Tensor)
                  else v for k, v in sliced.items()}

        self._log_videos(meta=meta, batch_idx=batch_idx, stage=stage, **sliced)

        return self._get_outputs(stage, len(frames), sliced, loss_dict)

    def _inner_step(self, frames: torch.Tensor, targets: Dict[str, torch.Tensor], edge_index: torch.Tensor, batch_vector: torch.Tensor) -> Dict[str, Union[Dict, torch.Tensor]]:
        raise NotImplementedError()

    def _get_outputs(self, stage, batch_size, sliced, loss_dict):
        # return primary loss - the first one available from loss_modes list
        # also log it as 'primary' for monitoring purposes
        # TODO: monitoring should be done based on the metric, not the loss
        # so 'primary' loss should be removed in the future
        for mode in self._loss_modes:
            if mode.name in loss_dict:
                self.log('{}_loss/primary'.format(stage),
                         loss_dict[mode.name], batch_size=batch_size)
                return {
                    'loss': loss_dict[mode.name],
                    'preds': {
                        'pose_changes': sliced['pose_inputs'] if self.movements_model.output_type == MovementsModelOutputType.pose_changes else None,
                        'world_rot_changes': sliced['world_rot_inputs'] if self.trajectory_model.output_type == TrajectoryModelOutputType.changes and 'world_rot_inputs' in sliced else None,
                        'world_loc_changes': sliced['world_loc_inputs'] if self.trajectory_model.output_type == TrajectoryModelOutputType.changes and 'world_loc_inputs' in sliced else None,
                        **{
                            k: sliced[k] if k in sliced and sliced[k] is not None else None
                            for k in self._crucial_keys
                        }
                    },
                    'targets': sliced['targets']
                }

        raise RuntimeError("Couldn't calculate any loss.")

    def _calculate_lossess(self, stage, batch_size, sliced, meta):
        # TODO: this will work for mono-type batches, but not for mixed-type batches;
        # Figure if/how to do mixed-type batches - should we even support it?
        # Maybe force reduction='none' in criterions and then reduce here?
        loss_dict = {}

        for mode in self._losses_to_calculate:
            (name, loss_fn, criterion, *_) = mode
            loss = loss_fn(
                criterion=criterion,
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes,
                mask_missing_joints=self.mask_missing_joints,
                requirements={
                    k: v
                    for k, v in loss_dict.items()
                    if k in mode[3]
                } if len(mode) > 3 else None,
                loss_weights=self.loss_weights,
                **sliced
            )
            if loss is not None and not torch.isnan(loss):
                loss_dict[name] = loss

                if LossModes[name] in self._loss_modes:
                    break  # stop after first successfully requested calculated loss, since

        for k, v in loss_dict.items():
            self.log('{}_loss/{}'.format(stage, k), v, batch_size=batch_size)
        return loss_dict

    def _eval_step_end(self, outputs, stage):
        # calculate and log metrics
        m = self.metrics(outputs['preds'], outputs['targets'])
        batch_size = len(outputs['preds'][self._outputs_key])

        unwrapped_m = self._unwrap_nested_metrics(m, [stage])
        for k, v in unwrapped_m.items():
            self.log(k, v, batch_size=batch_size, on_step=False, on_epoch=True)

    def _video_to_logger(self, vid, vid_idx, fps, stage, meta):
        if isinstance(self.trainer.loggers[0], TensorBoardLogger):
            vid = vid.permute(0, 1, 4, 2, 3).unsqueeze(0)  # B,T,H,W,C -> B,T,C,H,W
            self.trainer.loggers[0].experiment.add_video(
                '{}_{}_render'.format(stage, vid_idx),
                vid, self.global_step, fps=fps
            )
        # TODO: handle W&B too

    def _log_videos(self,
                    meta: Tensor,
                    batch_idx: int,
                    stage: str,
                    save_to_logger: bool = False,
                    **kwargs
                    ):
        if save_to_logger:
            vid_callback = self._video_to_logger
        else:
            vid_callback = None

        if len(self.trainer.loggers) > 1:
            self.trainer.loggers[1].experiment.log_videos(
                meta=meta,
                step=self.global_step,
                batch_idx=batch_idx,
                stage=stage,
                vid_callback=vid_callback,
                force=(stage != 'train' and batch_idx == 0),
                **kwargs
            )
