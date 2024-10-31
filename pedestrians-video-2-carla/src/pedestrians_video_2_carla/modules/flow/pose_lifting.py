from typing import Dict
from pedestrians_video_2_carla.modules.trajectory.zero import ZeroTrajectory
import torch
from pedestrians_video_2_carla.metrics.fb.fb_mpjpe import FB_MPJPE
from pedestrians_video_2_carla.metrics.fb.fb_mpjve import FB_MPJVE
from pedestrians_video_2_carla.metrics.fb.fb_n_mpjpe import FB_N_MPJPE
from pedestrians_video_2_carla.metrics.fb.fb_pa_mpjpe import FB_PA_MPJPE
from pedestrians_video_2_carla.metrics.fb.fb_weighted_mpjpe import FB_WeightedMPJPE
from pedestrians_video_2_carla.metrics.mpjpe import MPJPE
from pedestrians_video_2_carla.metrics.mrpe import MRPE
from pedestrians_video_2_carla.modules.flow.base import LitBaseFlow
from pedestrians_video_2_carla.modules.layers.projection import ProjectionModule
from pedestrians_video_2_carla.utils.world import calculate_world_from_changes

# available models
from pedestrians_video_2_carla.modules.movements.zero import ZeroMovements
from pedestrians_video_2_carla.modules.movements.linear import Linear
from pedestrians_video_2_carla.modules.movements.lstm import LSTM
from pedestrians_video_2_carla.modules.movements.linear_ae import LinearAE, LinearAEResidual, LinearAEResidualLeaky
from pedestrians_video_2_carla.modules.movements.baseline_3d_pose import Baseline3DPose, Baseline3DPoseRot
from pedestrians_video_2_carla.modules.movements.seq2seq import Seq2Seq, Seq2SeqEmbeddings, Seq2SeqFlatEmbeddings, Seq2SeqResidualA, Seq2SeqResidualB, Seq2SeqResidualC
from pedestrians_video_2_carla.modules.movements.pose_former import PoseFormer, PoseFormerRot


class LitPoseLiftingFlow(LitBaseFlow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.projection = ProjectionModule(
            movements_output_type=self.movements_model.output_type,
            trajectory_output_type=self.trajectory_model.output_type,
        )

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

                    # For pose lifting
                    Baseline3DPose,
                    Baseline3DPoseRot,
                    LinearAEResidual,
                    LinearAEResidualLeaky,
                    PoseFormer,
                    PoseFormerRot,
                ]
            },
            'trajectory': {
                m.__name__: m
                for m in [
                    ZeroTrajectory
                ]
            }
        }

    @classmethod
    def get_default_models(cls) -> Dict[str, torch.nn.Module]:
        """
        Returns a dictionary with default models.
        """
        return {
            'trajectory': ZeroTrajectory,
            'movements': LSTM,
        }

    def get_metrics(self):
        return {
            'MPJPE':MPJPE(
                dist_sync_on_step=True,
                input_nodes=self.movements_model.input_nodes
            ),
            'MRPE':MRPE(
                dist_sync_on_step=True,
                input_nodes=self.movements_model.input_nodes,
                output_nodes=self.movements_model.output_nodes
            ),
            'FB_MPJPE':FB_MPJPE(dist_sync_on_step=True),
            # FB_WeightedMPJPE should be same as FB_MPJPE since we provide no weights:
            'FB_WeightedMPJPE':FB_WeightedMPJPE(dist_sync_on_step=True),
            'FB_PA_MPJPE':FB_PA_MPJPE(dist_sync_on_step=True),
            'FB_N_MPJPE':FB_N_MPJPE(dist_sync_on_step=True),
            'FB_MPJVE':FB_MPJVE(dist_sync_on_step=True),
        }

    def _get_crucial_keys(self):
        return [
            self._outputs_key,
            'relative_pose_loc',
            'relative_pose_rot',
            'absolute_pose_loc',
            'absolute_pose_rot',
            'world_loc',
            'world_rot',
        ]

    def _on_batch_start(self, batch, batch_idx):
        self.projection.on_batch_start(batch, batch_idx)

    def _inner_step(self, frames: torch.Tensor, targets: Dict[str, torch.Tensor], edge_index: torch.Tensor = None, batch_vector: torch.Tensor = None):
        pose_inputs = self.movements_model(
            frames,
            targets if self.training and self.movements_model.needs_targets else None,
            edge_index=edge_index.to(
                self.device) if self.movements_model.needs_graph else None,
            batch_vector=batch_vector.to(
                self.device) if self.movements_model.needs_graph else None
        )

        world_loc_inputs, world_rot_inputs = self.trajectory_model(
            frames,
            targets if self.training and self.trajectory_model.needs_targets else None
        )

        projection_outputs = self.projection(
            pose_inputs,
            world_loc_inputs,
            world_rot_inputs
        )

        return self._get_sliced_data(frames, targets,
                                     pose_inputs, world_loc_inputs, world_rot_inputs,
                                     projection_outputs)

    def _get_sliced_data(self,
                         frames,
                         targets,
                         pose_inputs,
                         world_loc_inputs,
                         world_rot_inputs,
                         projection_outputs
                         ):
        # TODO: this should take into account both movements and trajectory models
        eval_slice = (slice(None), self.movements_model.eval_slice)

        # unpack projection outputs
        (projection_2d, projection_outputs_dict) = projection_outputs

        # get all inputs/outputs properly sliced
        sliced = {}

        sliced['pose_inputs'] = tuple([v[eval_slice] for v in pose_inputs]) if isinstance(
            pose_inputs, tuple) else pose_inputs[eval_slice]
        sliced['projection_2d'] = projection_2d[eval_slice]

        dm = self.trainer.datamodule
        if dm.transform_callable is not None:
            sliced['projection_2d_transformed'] = dm.transform_callable(
                projection_2d[eval_slice])

        sliced['world_loc_inputs'] = world_loc_inputs[eval_slice]
        sliced['world_rot_inputs'] = world_rot_inputs[eval_slice]
        sliced['inputs'] = frames[eval_slice]
        sliced['targets'] = {k: v[eval_slice] for k, v in targets.items()}

        keys_of_intrest = list(
            set(list(projection_outputs_dict.keys()) + self._crucial_keys))
        for k in keys_of_intrest:
            if k not in sliced:
                sliced[k] = projection_outputs_dict[k][eval_slice] if k in projection_outputs_dict and projection_outputs_dict[k] is not None else None

        # sometimes we need absolute target world loc/rot, which is not saved in data
        # so we need to compute it here and then slice appropriately
        # caveat - dataset needst to provide those targets in the first place
        try:
            target_world_loc, target_world_rot = calculate_world_from_changes(
                projection_2d.shape, projection_2d.device,
                targets['world_loc_changes'], targets['world_rot_changes']
            )
            sliced['targets']['world_loc'] = target_world_loc[eval_slice]
            sliced['targets']['world_rot'] = target_world_rot[eval_slice]
        except KeyError:
            pass
        return sliced
