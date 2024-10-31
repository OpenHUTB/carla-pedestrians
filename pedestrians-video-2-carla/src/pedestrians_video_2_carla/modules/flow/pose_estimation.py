from typing import Dict, Tuple
import numpy as np
import torch
import torchmetrics
from pedestrians_video_2_carla.modules.flow.autoencoder import LitAutoencoderFlow
from pedestrians_video_2_carla.modules.flow.output_types import PoseEstimationModelOutputType

# available models
from pedestrians_video_2_carla.modules.pose_estimation.unipose.unipose_lstm import UniPoseLSTM
from pedestrians_video_2_carla.modules.pose_estimation.transformers.avpedestrian_pose_transformer import AvPedestrianPoseTransformer
from pedestrians_video_2_carla.modules.pose_estimation.regular.p0 import P0

from pedestrians_video_2_carla.modules.pose_estimation.linear import Linear
from pedestrians_video_2_carla.utils.unravel_index import unravel_index


class LitPoseEstimationFlow(LitAutoencoderFlow):
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
                    Linear,

                    # For pose estimation
                    UniPoseLSTM,

                    # transformer
                    AvPedestrianPoseTransformer,

                    # regular
                    P0

                ]
            }
        }

    @classmethod
    def get_default_models(cls) -> Dict[str, torch.nn.Module]:
        """
        Returns a dictionary with default models.
        """
        return {
            'movements': UniPoseLSTM,
        }

    def get_initial_metrics(self) -> Dict[str, torchmetrics.Metric]:
        return {}

    def _calculate_initial_metrics(self) -> Dict[str, float]:
        return {}

    def _get_sliced_data(self,
                         frames,
                         targets,
                         pose_inputs
                         ):
        if self.movements_model.output_type == PoseEstimationModelOutputType.heatmaps:
            eval_slice = (slice(None), self.movements_model.eval_slice)

            # get all inputs/outputs properly sliced
            sliced = {}
            heatmaps = pose_inputs

            sliced['heatmaps'] = self._fix_dimensions(heatmaps)[eval_slice]

            # assumption: model output is in 'normalized' space if transform is not None
            # else it is in pixel space
            projection_2d = self._keypoints_from_heatmaps(
                heatmaps, frames.shape[-2:])  # pixel space

            sliced['projection_2d_confidence'] = self._fix_dimensions(projection_2d)[
                eval_slice]
            sliced['projection_2d'] = self._fix_dimensions(
                projection_2d[..., :2])[eval_slice]

            dm = self.trainer.datamodule
            if dm.transform_callable is not None:
                sliced['projection_2d_transformed'] = self._fix_dimensions(dm.transform_callable(
                    projection_2d[..., :2]))[eval_slice]

            sliced['inputs'] = self._fix_dimensions(frames)[eval_slice]
            sliced['targets'] = {k: self._fix_dimensions(
                v)[eval_slice[:v.ndim]] for k, v in targets.items()}

            # TODO: since the heatmaps after the model are smaller than in the input,
            # we need to resize the targets to match the actual output
            # here we're reusing the model's pool_center layer, which is
            # responsible for the resizing of the initial centermap to correct size
            # but this is very hackish and should be resolved in the future
            with torch.no_grad():
                # since center pooling won't accept a batch of sequences,
                # we're doing batch*sequence and then back
                h = sliced['targets']['heatmaps']
                rh = torch.nn.functional.avg_pool2d(
                    h.view((-1, *h.shape[2:])),
                    kernel_size=9,
                    stride=8,
                    padding=1
                )
                rh = rh.view((h.shape[0], h.shape[1], *rh.shape[1:]))
                sliced['targets']['heatmaps'] = rh

            return sliced
        else:
            return super()._get_sliced_data(frames, targets, pose_inputs)

    def _keypoints_from_heatmaps(self, heatmaps: torch.Tensor, bbox_size: Tuple[int, int]) -> torch.Tensor:
        b, l, p, h, w = heatmaps.shape
        (bbox_width, bbox_height) = bbox_size
        (sw, sh) = bbox_width / w, bbox_height / h

        keypoints = torch.zeros(
            (b, l, p-1, 3), dtype=torch.float32, device=heatmaps.device)

        # TODO: there's probably a better way to do this
        for bi in range(b):
            for li in range(l):
                for pi in range(p-1):
                    m = heatmaps[bi, li, pi+1]
                    c = m.max()
                    if c > 0:
                        h, w = unravel_index(m.argmax(), m.shape, as_tuple=True)

                        keypoints[bi, li, pi, 0] = w * sw
                        keypoints[bi, li, pi, 1] = h * sh
                        keypoints[bi, li, pi, 2] = c

        return keypoints
