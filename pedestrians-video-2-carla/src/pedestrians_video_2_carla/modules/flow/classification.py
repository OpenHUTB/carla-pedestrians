
from functools import cached_property
from typing import Any, Dict, List, Tuple, Type, Union
from pedestrians_video_2_carla.modules.classification.classification import ClassificationModel
import pytorch_lightning as pl
import platform

try:
    from torch_geometric.data import Batch
except ImportError:
    Batch = None

import torch
from pytorch_lightning.loggers.tensorboard import TensorBoardLogger
from torchmetrics import AUROC, ROC, ConfusionMatrix, F1Score, MetricCollection, Accuracy, Precision, PrecisionRecallCurve, Recall
import torchmetrics
from pedestrians_video_2_carla.data.base.skeleton import get_skeleton_type_by_name
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.metrics.multiinput_wrapper import MultiinputWrapper

from pedestrians_video_2_carla.modules.classification.gnn.dcrnn import DCRNNModel
from pedestrians_video_2_carla.modules.classification.gnn.gconv_gru import GConvGRUModel
from pedestrians_video_2_carla.modules.classification.gnn.gconv_lstm import GConvLSTMModel
from pedestrians_video_2_carla.modules.classification.gnn.tgcn import TGCNModel
from pedestrians_video_2_carla.modules.classification.lstm import LSTM
from pedestrians_video_2_carla.modules.classification.gru import GRU
from pedestrians_video_2_carla.modules.classification.gnn.gcn_best_paper import GCNBestPaper
from pedestrians_video_2_carla.modules.classification.gnn.gcn_best_paper_transformer import GCNBestPaperTransformer
from pedestrians_video_2_carla.modules.flow.output_types import ClassificationModelOutputType

from pedestrians_video_2_carla.utils.printing import print_metrics
from pytorch_lightning.utilities import rank_zero_only

try:
    import wandb
    from pytorch_lightning.loggers.wandb import WandbLogger
except ImportError:
    WandbLogger = None


class LitClassificationFlow(pl.LightningModule):
    # TODO: potentially update to re-use base flow when feasible

    def __init__(self,
                 classification_model: ClassificationModel,
                 classification_targets_key: str,
                 classification_average: Union[str, Dict[str, str]] = 'macro',
                 num_classes: int = 2,
                 **kwargs: Any) -> None:
        super().__init__()

        self.classification_model = classification_model

        self._targets_key = classification_targets_key
        self._outputs_key = classification_targets_key + '_logits'

        self._num_classes = num_classes

        if isinstance(classification_average, str):
            if classification_average == 'benchmark':
                self._average = {
                    'Accuracy': 'micro',
                    'Precision': 'none',  # binary
                    'Recall': 'none',  # binary
                    'F1Score': 'none',  # binary
                }
            else:
                self._average = {
                    'Accuracy': classification_average,
                    'Precision': classification_average,
                    'Recall': classification_average,
                    'F1Score': classification_average,
                }
        else:
            self._average = classification_average

        if self._num_classes == 2 and classification_model.output_type == ClassificationModelOutputType.binary:
            self.criterion = torch.nn.BCEWithLogitsLoss()
        else:
            self.criterion = torch.nn.CrossEntropyLoss()

        self.metrics = MetricCollection(self.get_metrics())

        self.save_hyperparameters({
            'host': platform.node(),
            'classification_average': self._average,
            **self.classification_model.hparams,
        })

    def get_initial_metrics(self) -> Dict[str, torchmetrics.Metric]:
        """
        Returns metrics to calculate on each validation batch at the beginning of the training.
        They will be added to the metrics returned by get_metrics() method.
        """
        return {}

    def get_metrics(self) -> Dict[str, torchmetrics.Metric]:
        multiclass = self.classification_model.output_type == ClassificationModelOutputType.multiclass
        num_classes = self._num_classes if multiclass else None

        return {
            'Accuracy': MultiinputWrapper(
                Accuracy(dist_sync_on_step=True,
                         average=self._average['Accuracy'],
                         num_classes=self._num_classes if self._average['Accuracy'] != 'micro' else num_classes,
                         multiclass=multiclass if self._average['Accuracy'] != 'none' else True),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'Precision': MultiinputWrapper(
                Precision(dist_sync_on_step=True,
                          average=self._average['Precision'],
                          num_classes=self._num_classes if self._average['Precision'] != 'micro' else num_classes,
                          multiclass=multiclass if self._average['Precision'] != 'none' else True),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'Recall': MultiinputWrapper(
                Recall(dist_sync_on_step=True,
                       average=self._average['Recall'],
                       num_classes=self._num_classes if self._average['Recall'] != 'micro' else num_classes,
                       multiclass=multiclass if self._average['Recall'] != 'none' else True),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'F1Score': MultiinputWrapper(
                F1Score(dist_sync_on_step=True,
                        average=self._average['F1Score'],
                        num_classes=self._num_classes if self._average['F1Score'] != 'micro' else num_classes,
                        multiclass=multiclass if self._average['F1Score'] != 'none' else True),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'ConfusionMatrix': MultiinputWrapper(
                ConfusionMatrix(dist_sync_on_step=True,
                                num_classes=self._num_classes,
                                normalize=None),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'AUROC': MultiinputWrapper(
                AUROC(dist_sync_on_step=True,
                      num_classes=num_classes),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'ROCCurve': MultiinputWrapper(
                ROC(dist_sync_on_step=True, num_classes=num_classes),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
            'PRCurve': MultiinputWrapper(
                PrecisionRecallCurve(dist_sync_on_step=True,
                                     num_classes=num_classes),
                self._outputs_key, self._targets_key,
                input_nodes=None, output_nodes=None,
            ),
        }

    @property
    def needs_graph(self):
        return self.classification_model.needs_graph

    @property
    def needs_heatmaps(self):
        return False

    @property
    def needs_confidence(self):
        return self.classification_model.needs_confidence

    @cached_property
    def class_labels(self):
        return self.trainer.datamodule.class_labels[self._targets_key]

    @classmethod
    def get_available_models(cls) -> Dict[str, Dict[str, torch.nn.Module]]:
        """
        Returns a dictionary with available/required models.
        """
        return {
            'classification': {
                "GConvLSTM": GConvLSTMModel,
                "DCRNN": DCRNNModel,
                "TGCN": TGCNModel,
                "GConvGRU": GConvGRUModel,
                "LSTM": LSTM,
                "GRU": GRU,
                "GCNBestPaper": GCNBestPaper,
                "GCNBestPaperTransformer": GCNBestPaperTransformer
            }
        }

    @classmethod
    def get_default_models(cls) -> Dict[str, torch.nn.Module]:
        """
        Returns a dictionary with default models.
        """
        return {
            'classification': LSTM,
        }

    @staticmethod
    def add_model_specific_args(parent_parser):
        """
        Add model specific commandline arguments.

        By default, this method adds parameters for projection layer and loss modes.
        If overriding, remember to call super().
        """
        parser = parent_parser.add_argument_group("Classification Module")
        parser.add_argument(
            '--classification_average',
            type=str,
            choices=['micro', 'macro', 'weighted', 'none', 'benchmark'],
            default='macro',
        )

        return parent_parser

    def configure_optimizers(self) -> Dict[str, Union[torch.optim.Optimizer, '_LRScheduler']]:
        return self.classification_model.configure_optimizers()

    @rank_zero_only
    def on_fit_start(self) -> None:
        initial_metrics = self._calculate_initial_metrics()
        self._update_hparams(initial_metrics)

    def _update_hparams(self, initial_metrics: Dict[str, float] = None):
        # We need to manually add the datamodule hparams,
        # because the merge is automatically handled only for initial_hparams
        # additionally, store info on train set size for easy access
        additional_config = {
            **self.trainer.datamodule.hparams,
            'train_set_size': getattr(
                self.trainer.datamodule.train_set,
                '__len__',
                lambda: self.trainer.limit_train_batches*self.trainer.datamodule.batch_size
            )(),
            'val_set_size': getattr(
                self.trainer.datamodule.val_set,
                '__len__',
                lambda: self.trainer.limit_val_batches*self.trainer.datamodule.batch_size
            )(),
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

    def _calculate_initial_metrics(self) -> Dict[str, float]:
        dl = self.trainer.datamodule.val_dataloader()
        if dl is None:
            return {}

        # get class counts
        class_counts = self.trainer.datamodule.class_counts
        if self._num_classes > 2:
            prevalent_class = torch.argmax(
                torch.Tensor([
                    class_counts['train'][self._targets_key][n]
                    for n in self.class_labels
                ])).item()
        else:
            # binary classification
            prevalent_class = 1

        initial_metrics = MetricCollection({
            **self.get_metrics(),
            **self.get_initial_metrics()
        }).to(self.device)

        if not len(initial_metrics):
            return {}

        for batch in dl:
            (inputs, targets, *_) = self._unwrap_batch(batch)
            d_targets = {k: v.to(self.device) for k, v in targets.items()}

            if self.classification_model.output_type == ClassificationModelOutputType.multiclass:
                one_hot = torch.nn.functional.one_hot(
                    torch.ones_like(d_targets[self._targets_key]) * prevalent_class,
                    num_classes=self._num_classes
                )

                if one_hot.ndim > 2:
                    one_hot = one_hot.transpose(one_hot.ndim-2, one_hot.ndim-1)

                initial_metrics.update({
                    self._outputs_key: one_hot.float(),
                }, d_targets)
            else:
                initial_metrics.update({
                    self._outputs_key: (torch.ones_like(d_targets[self._targets_key]) * prevalent_class).float(),
                }, d_targets)

        results = initial_metrics.compute()
        unwrapped = {
            k: v.item()
            for k, v in self._unwrap_nested_metrics(results, ['initial']).items()
            if isinstance(v, torch.Tensor) and v.numel() == 1
        }
        unwrapped.update({
            k: v
            for k, v in self._unwrap_nested_metrics(class_counts, ['counts']).items()
        })

        if len(unwrapped) > 0:
            print_metrics(unwrapped, 'Initial metrics:')

        return unwrapped

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

    def _log_curve(self, key: str, title: str, col_names: List[str], x: Tuple[torch.Tensor], y: Tuple[torch.Tensor], axis_names: List[str] = None):
        if not isinstance(self.trainer.loggers[0], WandbLogger):
            return

        # copied part of code from W&B, since we already have curve(s)
        # but not raw data to calculate it

        curves = {}

        if axis_names is None:
            axis_names = col_names

        if len(x) == len(self.class_labels):
            classes = enumerate(self.class_labels)
        else:
            classes = ((0, self.class_labels[1]), )
            x = x.unsqueeze(0)
            y = y.unsqueeze(0)

        for i, class_name in classes:
            x_class = x[i]
            y_class = y[i]

            x_class_len = x_class.shape[0]
            y_class_len = y_class.shape[0]

            samples = min(20, y_class_len)
            sample_y = []
            sample_x = []
            for k in range(samples):
                sample_x.append(x_class[int(x_class_len * k / samples)].item())
                sample_y.append(
                    y_class[int(y_class_len * k / samples)].item()
                )

            curves[class_name] = (sample_x, sample_y)

        data = [
            [class_name, round(x_s, 3), round(y_s, 3)]
            for class_name, vals in curves.items()
            for x_s, y_s in zip(*vals)
        ]

        self.trainer.loggers[0].experiment.log({
            key: wandb.plot_table(
                "wandb/area-under-curve/v0",
                wandb.Table(columns=["class"] + col_names, data=data),
                {"x": col_names[0], "y": col_names[1], "class": "class"},
                {
                    "title": title,
                    "x-axis-title": axis_names[0],
                    "y-axis-title": axis_names[1],
                },
            ),
            "trainer/global_step": self.trainer.global_step,
        })

    def _log_pr_curve(self, k, v):
        return self._log_curve(
            k,
            "Precision v. Recall",
            ["recall", "precision"],
            v[1], v[0],
            axis_names=["Recall", "Precision"],
        )

    def _log_roc_curve(self, k, v):
        return self._log_curve(
            k,
            "ROC",
            ["fpr", "tpr"],
            v[0], v[1],
            axis_names=["False Positive Rate", "True Positive Rate"],
        )

    def _log_confusion_matrix(self, k, v):
        if not isinstance(self.trainer.loggers[0], WandbLogger):
            return

        # copied part of code from W&B, since we already have confusion matrix
        # but not raw data to calculate it
        data = []
        for i in range(self._num_classes):
            for j in range(self._num_classes):
                data.append([self.class_labels[i], self.class_labels[j], v[i, j]])

        fields = {
            "Actual": "Actual",
            "Predicted": "Predicted",
            "nPredictions": "nPredictions",
        }

        self.trainer.loggers[0].experiment.log({
            k: wandb.plot_table(
                "wandb/confusion_matrix/v1",
                wandb.Table(columns=["Actual", "Predicted", "nPredictions"], data=data),
                fields,
                {"title": k},
            ),
            "trainer/global_step": self.trainer.global_step,
        })

    def _eval_step_end(self, outputs, stage):
        # update metrics
        self.metrics.update(outputs['preds'], outputs['targets'])

    # TODO: somehow this is not working
    def _on_eval_epoch_end(self):
        try:
            unwrapped_m = self._unwrap_nested_metrics(self.metrics.compute(), ['hp'])
            for k, v in unwrapped_m.items():
                # special W&B plots
                if not isinstance(self.trainer.loggers[0], TensorBoardLogger):
                    if k.endswith('ConfusionMatrix'):
                        self._log_confusion_matrix(k, v)
                    elif k.endswith('ROCCurve'):
                        # self._log_roc_curve(k, v)
                        pass
                    elif k.endswith('PRCurve'):
                        # self._log_pr_curve(k, v)
                        pass
                    else:
                        self.log(k, v)
                else:
                    self.log(k, v)
        except ValueError as e:
            # cannot compute metrics for this epoch (e.g. only one class is present in validation sanity check)
            pass

        self.metrics.reset()

    def on_validation_epoch_end(self) -> None:
        return self._on_eval_epoch_end()

    def on_test_epoch_end(self) -> None:
        return self._on_eval_epoch_end()

    def _unwrap_nested_metrics(self, items: Union[Dict, float], keys: List[str], zeros: bool = False, nans: bool = False):
        r = {}
        if hasattr(items, 'items'):
            for k, v in items.items():
                r.update(self._unwrap_nested_metrics(v, keys + [k], zeros, nans))
        else:
            if zeros:
                v = 0.0
            elif nans:
                v = float('nan')
            # for binary case
            elif keys[-1] in self._average and self._average[keys[-1]] == 'none' and self._num_classes == 2 and isinstance(items, torch.Tensor) and items.numel() == 2:
                v = items[1]  # positive class in binary classification
            else:
                v = items

            r['/'.join(keys)] = v
        return r

    def _step(self, batch, batch_idx, stage):
        (frames, targets, meta, edge_index, batch_vector) = self._unwrap_batch(batch)
        sliced = self._inner_step(frames, targets, edge_index, batch_vector)

        loss_dict = self._calculate_lossess(stage, len(frames), sliced, meta)

        self._log_videos(meta=meta, batch_idx=batch_idx, stage=stage, **sliced)

        return self._get_outputs(stage, len(frames), sliced, meta, loss_dict)

    def _inner_step(self, frames: torch.Tensor, targets: Dict[str, torch.Tensor], edge_index: torch.Tensor, batch_vector: torch.Tensor) -> Dict[str, Union[Dict, torch.Tensor]]:
        out = self.classification_model(frames, edge_index, batch_vector)

        out_slice = slice(None, None, None)
        if self.needs_graph and frames.shape[0] != out.shape[0]:
            out_slice = slice(-1, None, None)

        out = out[out_slice]
        target = targets[self._targets_key][out_slice]

        if out.ndim - 1 != target.ndim:
            target = target.squeeze(-1)

        return {
            'inputs': frames[out_slice],
            self._outputs_key: out,
            'targets': {
                **targets,
                self._targets_key: target,
            },
        }

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
                batch.batch,
            )

    def _get_outputs(self, stage, batch_size, sliced, meta, loss_dict):
        if self.criterion.__class__.__name__ in loss_dict:
            loss = loss_dict[self.criterion.__class__.__name__]
            self.log('{}_loss/primary'.format(stage), loss, batch_size=batch_size)
            return {
                'loss': loss,
                'preds': {
                    self._outputs_key: sliced[self._outputs_key].detach(),
                },
                'targets': sliced['targets'],
            }

        raise RuntimeError("Couldn't calculate any loss.")

    def _calculate_lossess(self, stage, batch_size, sliced, meta):
        # TODO: for now this hardcodes the loss to be the cross entropy loss
        loss_dict = {}

        criterion_targets = torch.atleast_1d(sliced['targets'][self._targets_key])
        if self.classification_model.output_type == ClassificationModelOutputType.binary:
            criterion_targets = criterion_targets.to(torch.float32)

        loss = self.criterion(
            sliced[self._outputs_key],
            criterion_targets,
        )

        if loss is not None and not torch.isnan(loss):
            loss_dict[self.criterion.__class__.__name__] = loss

        for k, v in loss_dict.items():
            self.log('{}_loss/{}'.format(stage, k), v, batch_size=batch_size)
        return loss_dict

    def _log_videos(self,
                    meta: torch.Tensor,
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
