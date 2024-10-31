
import glob
import logging
import os
from typing import Any, Dict, Tuple
from PIL import Image
import numpy as np
import pims
import torch
import matplotlib

from pedestrians_video_2_carla.utils.gaussian_kernel import gaussian_kernel
from pedestrians_video_2_carla.transforms.video.video_to_resnet import VideoToResNet
from pedestrians_video_2_carla.utils.argparse import boolean


class VideoMixin:
    """
    Mixin that returns raw video frame instead of the projection_2d as the input.
    """

    def __init__(
        self,
        source_videos_dir: str,
        frames_target_size: int = 368,
        frames_bbox_crop: bool = False,
        frames_bbox_margin: float = 0.2,
        needs_heatmaps: bool = False,
        heatmaps_sigma: int = 1,
        **kwargs
    ):
        super().__init__(**kwargs)

        augment_args = [v for k, v in kwargs.items() if k.startswith('augment_')]
        if any(augment_args):
            raise ValueError("VideoMixin does not support augment_* args.")

        self.source_videos_dir = source_videos_dir
        self.target_size = frames_target_size
        self.video_transform = VideoToResNet(target_size=self.target_size)
        self.bbox_crop = frames_bbox_crop
        self.bbox_margin = frames_bbox_margin

        # TODO: heatmaps should be a separate mixin
        self.needs_heatmaps = needs_heatmaps
        self.sigma = heatmaps_sigma

        if kwargs.get('skip_metadata', False):
            raise ValueError("VideoMixin does not support skip_metadata")

    @staticmethod
    def add_cli_args(parser):
        parser.add_argument(
            "--heatmaps_sigma",
            type=int,
            default=1,
            help="""
                Gaussian kernel sigma for heatmaps.
            """
        )

        parser.add_argument(
            "--frames_target_size",
            type=int,
            default=368,
            help="""
                Size of the image inserted into the network.
            """
        )
        parser.add_argument(
            "--frames_bbox_crop",
            type=boolean,
            default=False,
            help="""
                Should image be cropped to square centered on bbox?
            """
        )
        parser.add_argument(
            "--frames_bbox_margin",
            type=float,
            default=0.2,
            help="""
                Margin around bbox to crop.
            """
        )
        return parser

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, Dict[str, torch.Tensor], Dict[str, Any]]:
        (_, targets, meta) = super().__getitem__(idx)

        clip_frames, original_shape, shifts = self._get_clip_frames(
            meta, targets['bboxes'])

        if self.needs_heatmaps:
            self._add_heatmaps_to_targets(
                targets, clip_frames.shape, original_shape, shifts)

            # debug
            # self.__debug_heatmaps(targets, clip_frames)

        return (clip_frames, targets, meta)

    def _get_clip_frames(self, meta: Dict[str, Any], bboxes: torch.Tensor = None) -> torch.Tensor:
        """
        Returns the clip frames.
        """

        set_name = meta.get('set_name', '')
        video_id = meta['video_id']
        pedestrian_id = meta['pedestrian_id']
        clip_id = meta['clip_id']
        start_frame = meta['start_frame']
        end_frame = meta['end_frame']

        paths = glob.glob(os.path.join(
            self.source_videos_dir, set_name, '{}.*'.format(os.path.splitext(video_id)[0])))

        if len(paths) != 1:
            # this shouldn't happen
            logging.getLogger(__name__).warn(
                "Clip extraction failed for {}, {}, {}".format(
                    video_id,
                    pedestrian_id,
                    clip_id))
            return torch.zeros((end_frame - start_frame, 3, self.target_size, self.target_size))

        with pims.PyAVReaderIndexed(paths[0]) as video:
            clip = video[start_frame:end_frame]
            clip_length = len(clip)

            assert clip_length == end_frame - start_frame, "Clip length mismatch"

            clip_frames = np.array(clip)

        if self.bbox_crop:
            clip_frames, shifts = VideoMixin.crop_bbox(
                clip_frames, bboxes.numpy(), bbox_margin=self.bbox_margin, target_size=self.target_size)
        else:
            shifts = torch.zeros((clip_length, 2), dtype=int)

        return self.video_transform(clip_frames), clip_frames.shape[1:3], torch.from_numpy(shifts)

    @staticmethod
    def crop_bbox(clip_frames: np.ndarray, bboxes: np.ndarray, bbox_margin: float = 0.2, target_size: int = 368):
        """
        Crop the clip frames to the bounding box + margin.
        """
        clip_length, clip_height, clip_width, _ = clip_frames.shape
        canvas_size = ((
            bboxes[:, 1] - bboxes[:, 0]).max() * (1 + 2*bbox_margin)).astype(int)
        canvas_size = max(canvas_size, target_size)
        half_size = canvas_size // 2

        canvas = np.zeros((clip_length, canvas_size, canvas_size, 3), dtype=np.uint8)
        centers = (bboxes.mean(axis=-2) + 0.5).round().astype(int)
        shifts = np.zeros((clip_length, 2), dtype=int)

        for idx in range(clip_length):
            shifts[idx] = VideoMixin.extract_bbox_from_frame(canvas[idx], clip_frames[idx],
                                                             (half_size, half_size),
                                                             centers[idx],
                                                             (clip_width, clip_height),
                                                             )

        return canvas, shifts

    @staticmethod
    def extract_bbox_from_frame(canvas: np.ndarray, frame: np.ndarray, half_size: Tuple[int, int], bbox_center: Tuple[int, int], clip_size: Tuple[int, int]):
        (half_width, half_height) = half_size
        (x_center, y_center) = bbox_center
        (clip_width, clip_height) = clip_size

        frame_x_min = int(max(0, x_center-half_width))
        frame_x_max = int(min(clip_width, x_center+half_width))
        frame_y_min = int(max(0, y_center-half_height))
        frame_y_max = int(min(clip_height, y_center+half_height))
        frame_width = frame_x_max - frame_x_min
        frame_height = frame_y_max - frame_y_min
        canvas_x_shift = max(0, half_width-x_center)
        canvas_y_shift = max(0, half_height-y_center)
        canvas[canvas_y_shift:canvas_y_shift+frame_height, canvas_x_shift:canvas_x_shift +
               frame_width] = frame[frame_y_min:frame_y_max, frame_x_min:frame_x_max]

        return frame_x_min - canvas_x_shift, frame_y_min - canvas_y_shift

    def _add_heatmaps_to_targets(
        self,
        targets: Dict[str, torch.Tensor],
        transformed_shape: Tuple[int, int, int, int],  # (T, C, H, W)
        original_shape: Tuple[int, int],  # (H, W)
        shifts: torch.Tensor,  # (T, 2)
    ):
        projection_2d = targets['projection_2d']

        # add heatmaps
        clip_length, _, clip_height, clip_width = transformed_shape
        heatmaps = torch.zeros((clip_length, self.num_input_joints + 1,
                               clip_height, clip_width), dtype=torch.float32)

        for i in range(clip_length):
            heatmaps[i, :, :, :] = self._get_heatmap(
                projection_2d[i], (clip_height, clip_width), original_shape, shifts[i])

        targets['heatmaps'] = heatmaps

    def _get_heatmap(self, projection_2d: torch.Tensor, clip_size: Tuple[int, int], original_size: Tuple[int, int], shift: torch.Tensor) -> torch.Tensor:
        clip_height, clip_width = clip_size
        original_height, original_width = original_size
        scale = torch.tensor([clip_width / original_width,
                             clip_height / original_height]).float()

        heatmap = torch.zeros(
            (self.num_input_joints + 1, clip_height, clip_width), dtype=torch.float32)

        scaled_keypoints = ((projection_2d - shift) * scale).round().int()
        for i in range(self.num_input_joints):
            heatmap[i+1, :, :] = gaussian_kernel(clip_width, clip_height,
                                                 scaled_keypoints[i][0].item(),
                                                 scaled_keypoints[i][1].item(),
                                                 self.sigma)

        heatmap[0, :, :] = torch.neg(
            torch.max(heatmap[1:, :, :], dim=0)[0]) + 1.0  # for background

        return heatmap

    def __debug_heatmaps(self, targets, clip_frames):
        im = Image.fromarray(np.clip((clip_frames.numpy()[0].transpose(1, 2, 0) * np.array([0.229, 0.224, 0.225]) + np.array(
            [0.485, 0.456, 0.406])) * 255.0, 0, 255).astype(np.uint8))
        im.save('/outputs/im.png')

        cm = matplotlib.cm.get_cmap('jet')
        heatmaps = targets['heatmaps'].numpy()[0]

        for i, heat in enumerate(heatmaps):
            heatmap = Image.fromarray((255.0*cm(heat)[..., :3]).astype(np.uint8))
            im_heat = Image.blend(im, heatmap, 0.8)
            im_heat.save('/outputs/im_heat'+str(i)+'.png')
