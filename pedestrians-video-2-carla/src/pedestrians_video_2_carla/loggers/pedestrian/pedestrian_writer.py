import copy
import os
from typing import Any, Callable, Dict, Iterator, List, Tuple, Type

import numpy as np
import torch
import torchvision
from pedestrians_scenarios.karma.renderers.points_renderer import \
    PointsRenderer
from pedestrians_scenarios.karma.renderers.renderer import Renderer
from pedestrians_scenarios.karma.renderers.source_videos_renderer import \
    SourceVideosRenderer
from pedestrians_video_2_carla.data.base.skeleton import Skeleton
from pedestrians_video_2_carla.renderers.carla_renderer import CarlaRenderer
from pedestrians_video_2_carla.renderers.smpl_renderer import SMPLRenderer
from pedestrians_video_2_carla.transforms.pose.augmentation.augment_pose import AugmentPose
from pedestrians_video_2_carla.transforms.pose.normalization.denormalizer import DeNormalizer, Extractor
from pedestrians_video_2_carla.transforms.pose.normalization.hips_neck_extractor import HipsNeckExtractor
from pedestrians_video_2_carla.transforms.pose.normalization.reference_skeletons_denormalizer import \
    ReferenceSkeletonsDeNormalizer
from torch import Tensor
from tqdm.auto import tqdm

from .enums import MergingMethod, PedestrianRenderers


class PedestrianWriter(object):
    def __init__(self,
                 log_dir: str,
                 renderers: List[PedestrianRenderers],
                 input_nodes: Type[Skeleton],
                 output_nodes: Type[Skeleton] = None,
                 reduced_log_every_n_steps: int = 500,
                 fps: float = 30.0,
                 max_videos: int = 10,
                 merging_method: MergingMethod = MergingMethod.square,
                 source_videos_dir: str = None,
                 body_model_dir: str = None,
                 source_videos_overlay_skeletons: bool = False,
                 source_videos_overlay_bboxes: bool = False,
                 source_videos_overlay_classes: bool = False,
                 **kwargs) -> None:
        self._log_dir = log_dir

        self._reduced_log_every_n_steps = reduced_log_every_n_steps
        self._fps = fps
        self._max_videos = max_videos

        if self._max_videos > 0:
            self.__videos_slice = slice(0, self._max_videos)
        else:
            self.__videos_slice = slice(None)

        self._used_renderers: List[PedestrianRenderers] = renderers[:]
        if len(self._used_renderers) < 3 and merging_method == MergingMethod.square:
            merging_method = MergingMethod.horizontal
        self._merging_method = merging_method

        if self._merging_method == MergingMethod.square:
            # find how many videos we need per row and column
            self._video_columns = int(np.ceil(np.sqrt(len(self._used_renderers))))
            self._video_rows = int(
                np.ceil(len(self._used_renderers) / self._video_columns))

            # pad with zero renderers
            self._used_renderers += [PedestrianRenderers.zeros] * (self._video_columns *
                                                                   self._video_rows - len(self._used_renderers))
        elif self._merging_method == MergingMethod.vertical:
            self._video_columns = 1
            self._video_rows = len(self._used_renderers)
        else:
            self._video_columns = len(self._used_renderers)
            self._video_rows = 1

        # TODO: fix writer requiring explicit input/output nodes from command line
        self._input_nodes = input_nodes
        self._output_nodes = output_nodes

        self.__denormalize_input = ReferenceSkeletonsDeNormalizer(
            extractor=HipsNeckExtractor(input_nodes=self._input_nodes,)
        )
        self.__deaugment_input = AugmentPose(
            nodes=self._input_nodes, flip=True, rotate=True).invert

        if self._output_nodes is not None:
            self.__denormalize_output = ReferenceSkeletonsDeNormalizer(
                extractor=HipsNeckExtractor(input_nodes=self._output_nodes,)
            )
            self.__deaugment_output = AugmentPose(
                nodes=self._output_nodes, flip=True, rotate=True).invert

        # actual renderers
        zeros_renderer = Renderer()
        self.__renderers: Dict[PedestrianRenderers, Renderer] = {
            PedestrianRenderers.zeros: zeros_renderer,
            PedestrianRenderers.source_videos: SourceVideosRenderer(
                data_dir=source_videos_dir,
                overlay_skeletons=source_videos_overlay_skeletons,
                overlay_bboxes=source_videos_overlay_bboxes,
                overlay_labels=source_videos_overlay_classes,
            ) if PedestrianRenderers.source_videos in self._used_renderers else zeros_renderer,
            PedestrianRenderers.source_carla: CarlaRenderer(
                fps=self._fps
            ) if PedestrianRenderers.source_carla in self._used_renderers else zeros_renderer,
            PedestrianRenderers.input_points: PointsRenderer(
                input_nodes=self._input_nodes
            ) if PedestrianRenderers.input_points in self._used_renderers else zeros_renderer,
            PedestrianRenderers.target_points: PointsRenderer(
                input_nodes=self._input_nodes
            ) if PedestrianRenderers.target_points in self._used_renderers else zeros_renderer,
            PedestrianRenderers.projection_points: PointsRenderer(
                input_nodes=self._output_nodes
            ) if PedestrianRenderers.projection_points in self._used_renderers and self._output_nodes is not None else zeros_renderer,
            PedestrianRenderers.carla: CarlaRenderer(
                fps=self._fps
            ) if PedestrianRenderers.carla in self._used_renderers else zeros_renderer,
            PedestrianRenderers.smpl: SMPLRenderer(
                body_model_dir=body_model_dir
            ) if PedestrianRenderers.smpl in self._used_renderers else zeros_renderer
        }

    @torch.no_grad()
    def log_videos(self,
                   meta: Dict[str, Tensor],
                   step: int,
                   batch_idx: int,
                   stage: str,
                   vid_callback: Callable = None,
                   force: bool = False,
                   # data that is passed from various inputs/outputs:
                   inputs: Tensor = None,
                   targets: Dict[str, Tensor] = None,
                   projection_2d: Tensor = None,
                   projection_2d_transformed: Tensor = None,
                   relative_pose_loc: Tensor = None,
                   relative_pose_rot: Tensor = None,
                   world_loc: Tensor = None,
                   world_rot: Tensor = None,
                   **kwargs) -> None:
        if step % self._reduced_log_every_n_steps != 0 and not force:
            return

        # TODO: render videos in background so rendering is not blocking the main thread

        for vid_idx, (vid, meta) in tqdm(enumerate(self._render(
                inputs[self.__videos_slice],
                {k: v[self.__videos_slice] for k, v in targets.items()},
                {k: v[self.__videos_slice] for k, v in meta.items()},
                projection_2d[self.__videos_slice] if projection_2d is not None else None,
                projection_2d_transformed[self.__videos_slice] if projection_2d_transformed is not None else None,
                relative_pose_loc[self.__videos_slice] if relative_pose_loc is not None else None,
                relative_pose_rot[self.__videos_slice] if relative_pose_rot is not None else None,
                world_loc[self.__videos_slice] if world_loc is not None else None,
                world_rot[self.__videos_slice] if world_rot is not None else None,
                batch_idx)), desc="Rendering clips", total=self._max_videos, leave=False):
            video_dir = os.path.join(self._log_dir, stage, meta.get(
                'set_name', ''), meta['video_id'])
            os.makedirs(video_dir, exist_ok=True)

            torchvision.io.write_video(
                os.path.join(video_dir,
                             '{pedestrian_id}-{clip_id:0>2d}-step={step:0>4d}.mp4'.format(
                                 **meta,
                                 step=step
                             )),
                vid,
                fps=self._fps
            )

            if vid_callback is not None:
                vid_callback(vid, vid_idx, self._fps, stage, meta)

    @torch.no_grad()
    def _render(self,
                frames: Tensor,
                targets: Dict[str, Tensor],
                meta: Dict[str, List[Any]],
                projection_2d: Tensor,
                projection_2d_transformed: Tensor,
                relative_pose_loc: Tensor,
                relative_pose_rot: Tensor,
                world_loc: Tensor,
                world_rot: Tensor,
                batch_idx: int
                ) -> Iterator[Tuple[Tensor, Tuple[str, str, int]]]:
        """
        Prepares video data. **It doesn't save anything!**

        :param frames: Input frames
        :type frames: Tensor
        :param targets: Target data
        :type targets: Dict[str, Tensor]
        :param meta: Meta data for each clips
        :type meta: Dict[str, List[Any]]
        :param projection_2d: Output of the projection layer.
        :type projection_2d: Tensor
        :param projection_2d_transformed: Output of the projection layer in the transformation space.
        :type projection_2d_transformed: Tensor
        :param relative_pose_loc: Output from the .forward converted to relative pose locations. May be None.
        :type relative_pose_loc: Tensor
        :param relative_pose_rot: Output from the .forward converted to absolute pose rotations.
        :type relative_pose_rot: Tensor
        :param world_loc: Output from the .forward converted to world locations. Get it from projection layer.
        :type world_loc: Tensor
        :param world_rot: Output from the .forward converted to world rotations. May be None.
        :type world_rot: Tensor
        :param batch_idx: Batch index
        :type batch_idx: int
        :return: List of videos and metadata
        :rtype: Tuple[List[Tensor], Tuple[str]]
        """

        # TODO: handle world_loc and world_rot in carla renderers
        # TODO: denormalization should be aware of the camera position if possible
        if 'projection_2d_transformed' in targets:
            denormalized_input_projection = self.__denormalize_input.from_projection(
                frames, meta)
            denormalized_target_projection = self.__denormalize_input.from_projection(
                targets['projection_2d_transformed'], meta)
        else:
            denormalized_input_projection = self.__denormalize_input.from_projection(
                frames, meta, autonormalize=True)
            denormalized_target_projection = self.__denormalize_input.from_projection(
                targets['projection_2d'], meta, autonormalize=True)

        if projection_2d_transformed is not None:
            denormalized_output_projection = self.__denormalize_output.from_projection(
                projection_2d_transformed, meta)
        elif projection_2d is not None:
            denormalized_output_projection = self.__denormalize_output.from_projection(
                projection_2d, meta, autonormalize=True)
        else:
            denormalized_output_projection = None

        output_videos = []

        if PedestrianRenderers.source_videos in self._used_renderers:
            source_videos_meta, source_videos_bboxes = self._prepare_overlays(
                targets, meta, projection_2d, projection_2d_transformed)

        render = {
            PedestrianRenderers.zeros: lambda: self.__renderers[PedestrianRenderers.zeros].render(
                frames
            ),
            PedestrianRenderers.source_videos: lambda: self.__renderers[PedestrianRenderers.source_videos].render(
                meta=source_videos_meta,
                bboxes=source_videos_bboxes
            ),
            PedestrianRenderers.source_carla: lambda: self.__renderers[PedestrianRenderers.source_carla].render(
                targets['relative_pose_loc'], targets['relative_pose_rot'],
                targets['world_loc'], targets['world_rot'],
                meta
            ),
            PedestrianRenderers.smpl: lambda: self.__renderers[PedestrianRenderers.smpl].render(
                targets['amass_body_pose'], meta
            ),
            PedestrianRenderers.input_points: lambda: self.__renderers[PedestrianRenderers.input_points].render(
                denormalized_input_projection
            ),
            PedestrianRenderers.target_points: lambda: self.__renderers[PedestrianRenderers.target_points].render(
                denormalized_target_projection
            ),
            PedestrianRenderers.projection_points: lambda: self.__renderers[PedestrianRenderers.projection_points].render(
                denormalized_output_projection
            ),
            PedestrianRenderers.carla: lambda: self.__renderers[PedestrianRenderers.carla].render(
                relative_pose_loc, relative_pose_rot,
                world_loc, world_rot,
                meta
            )
        }

        for renderer_type in self._used_renderers:
            output_videos.append(render[renderer_type]())

        for vid_idx, vids in enumerate(zip(*output_videos)):
            # TODO: generating each item in vids should be done in parallel somehow
            # although the output_videos itself contains generators...

            merged_rows = []
            # for each row in the output
            for row in range(self._video_rows):
                merged_rows.append(np.concatenate(
                    vids[row * self._video_columns:(row + 1)*self._video_columns],
                    axis=2)
                )
            merged_vid = torch.tensor(
                np.concatenate(merged_rows, axis=1),
            )

            vid_meta = {
                'video_id': 'video_{:0>2d}_{:0>2d}'.format(
                    batch_idx,
                    vid_idx
                ),
                'pedestrian_id': '{}_{}'.format(
                    meta['age'][vid_idx] if 'age' in meta else 'adult',
                    meta['gender'][vid_idx] if 'gender' in meta else 'female'
                ),
                'clip_id': 0
            }
            vid_meta.update({
                k: v[vid_idx]
                for k, v in meta.items()
                if k != 'skeletons'
            })
            yield merged_vid, vid_meta

    def _prepare_overlays(self, targets, orig_meta, projection_2d, projection_2d_transformed):
        meta = {k: v.clone() if hasattr(v, 'clone') else copy.deepcopy(v)
                for k, v in orig_meta.items()}

        if self.__renderers[PedestrianRenderers.source_videos].overlay_labels:
            # TODO: get this from classification_targets_key
            # meta['labels'] = {'crossing': meta['crossing']}
            pass

        deaugmented_input_pose, new_targets = self.__deaugment_input(
            pose=targets['projection_2d'],
            targets=targets,
            meta=meta,
        )
        bboxes = new_targets['bboxes']

        if not self.__renderers[PedestrianRenderers.source_videos].overlay_skeletons:
            return meta, bboxes

        # convert projection_2d to something that can be rendered
        skeletons = [
            {
                'type': self._input_nodes,
                # red; TODO: make it configurable
                'color': (255, 0, 0),
                'keypoints': deaugmented_input_pose.cpu().numpy()
            }
        ]

        if self._output_nodes is not None:
            if projection_2d_transformed is not None and 'projection_2d_shift' in targets and 'projection_2d_scale' in targets:
                output_denormalizer = DeNormalizer()
                denormalized_output_pose = output_denormalizer(
                    projection_2d_transformed[..., :2],
                    targets['projection_2d_scale'],
                    targets['projection_2d_shift']
                )
                deaugmented_output_pose, _ = self.__deaugment_output(
                    pose=denormalized_output_pose,
                    targets=targets,
                    meta=meta,
                )
                skeletons.append({
                    'type': self._output_nodes,
                    # green; TODO: make it configurable
                    'color': (0, 255, 0),
                    'keypoints': deaugmented_output_pose.cpu().numpy()
                })
            elif projection_2d is not None:
                deaugmented_output_pose, _ = self.__deaugment_output(
                    pose=projection_2d[..., :2],
                    targets=targets,
                    meta=meta,
                )
                skeletons.append({
                    'type': self._output_nodes,
                    # blue; TODO: make it configurable
                    'color': (0, 0, 255),
                    'keypoints': deaugmented_output_pose.cpu().numpy()
                })

        meta['skeletons'] = skeletons

        return meta, bboxes
