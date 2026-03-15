from typing import Dict
import torch
from torchmetrics import MeanSquaredError
import torchmetrics
from pedestrians_video_2_carla.metrics.missing_joints_ratio import MissingJointsRatio
from pedestrians_video_2_carla.metrics.multiinput_wrapper import MultiinputWrapper
from pedestrians_video_2_carla.metrics.pck import PCK
from pedestrians_video_2_carla.modules.flow.base import LitBaseFlow

# available models
from pedestrians_video_2_carla.modules.movements.zero import ZeroMovements
from pedestrians_video_2_carla.modules.movements.linear import Linear
from pedestrians_video_2_carla.modules.movements.lstm import LSTM
from pedestrians_video_2_carla.modules.movements.linear_ae import LinearAE, LinearAE2D
from pedestrians_video_2_carla.modules.movements.seq2seq import Seq2Seq, Seq2SeqEmbeddings, Seq2SeqFlatEmbeddings, Seq2SeqResidualA, Seq2SeqResidualB, Seq2SeqResidualC
from pedestrians_video_2_carla.modules.movements.transformers import SimpleTransformer


class LitAutoencoderFlow(LitBaseFlow):
    @property
    def needs_graph(self):
        return self.movements_model.needs_graph

    @classmethod
    def get_available_models(cls) -> Dict[str, Dict[str, torch.nn.Module]]:
        """
        Returns a dictionary with available/required models.
        """
        return {
            'movements': {
                m.__name__: m
                for m in [
                    # For testing
                    ZeroMovements,
                    Linear,

                    # Universal (support MovementsModelOutputType param)
                    LinearAE,
                    LSTM,
                    Seq2Seq,
                    Seq2SeqEmbeddings,
                    Seq2SeqFlatEmbeddings,
                    Seq2SeqResidualA,
                    Seq2SeqResidualB,
                    Seq2SeqResidualC,

                    # For 2D pose autoencoding
                    LinearAE2D,
                    SimpleTransformer,
                ]
            }
        }

    @classmethod
    def get_default_models(cls) -> Dict[str, torch.nn.Module]:
        """
        Returns a dictionary with default models.
        """
        return {
            'movements': LSTM,
        }

    def get_initial_metrics(self) -> Dict[str, torchmetrics.Metric]:
        return {
            'MJR': MissingJointsRatio(
                dist_sync_on_step=True,
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes,
                # no missing joints masking for this metric, since it is supposed to calculate it
            ),
        }

    def get_metrics(self) -> Dict[str, torchmetrics.Metric]:
        return {
            'MSE': MultiinputWrapper(
                MeanSquaredError(dist_sync_on_step=True),
                self._outputs_key, self._outputs_key,
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes,
                mask_missing_joints=self.mask_missing_joints,
            ),
            'PCKhn@01': PCK(
                dist_sync_on_step=True,
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes,
                mask_missing_joints=self.mask_missing_joints,
                key=self._outputs_key,
                threshold=0.1,
                get_normalization_tensor='hn',
            ),
            'PCK@005': PCK(
                dist_sync_on_step=True,
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes,
                mask_missing_joints=self.mask_missing_joints,
                key=self._outputs_key,
                threshold=0.05,
                get_normalization_tensor='bbox',
            ),
        }

    def _inner_step(self, frames: torch.Tensor, targets: Dict[str, torch.Tensor], edge_index: torch.Tensor, batch_vector: torch.Tensor):
        pose_inputs = self.movements_model(
            frames,
            targets=targets if self.training and self.movements_model.needs_targets else None,
            edge_index=edge_index.to(
                self.device) if self.needs_graph else None,
            batch_vector=batch_vector.to(
                self.device) if self.needs_graph else None
        )

        return self._get_sliced_data(frames, targets, pose_inputs)

    def _get_sliced_data(self,
                         frames,
                         targets,
                         pose_inputs
                         ):
        eval_slice = (slice(None), self.movements_model.eval_slice)

        # get all inputs/outputs properly sliced
        sliced = {}

        # assumption: model output is in 'normalized' space if transform is not None
        # else it is in pixel space
        sliced[self._outputs_key] = self._fix_dimensions(
            pose_inputs)[eval_slice]

        sliced['inputs'] = self._fix_dimensions(frames)[eval_slice]
        sliced['targets'] = {k: self._fix_dimensions(
            v)[eval_slice[:v.ndim]] for k, v in targets.items()}

        return sliced
