import glob
import logging
import math
import os
import warnings
from typing import Iterable, List, Dict, Any
import matplotlib

import numpy as np
import pims
import av
from PIL import Image, ImageDraw, ImageFont
from .renderer import Renderer
from .points_renderer import PointsRenderer
from tqdm.auto import trange


class SourceVideosRenderer(Renderer):
    labels_font = ImageFont.load_default()

    def __init__(
        self,
        data_dir: str,
        overlay_skeletons: bool = False,
        center_bboxes: bool = True,
        overlay_bboxes: bool = False,
        overlay_labels: bool = True,
        **kwargs
    ) -> None:
        super().__init__(**kwargs)

        self.__data_dir = data_dir
        self.__overlay_skeletons = overlay_skeletons
        self.__overlay_bboxes = overlay_bboxes
        self.__center_bboxes = center_bboxes
        self.__overlay_labels = overlay_labels

        if self.overlay_labels:
            system_fonts = matplotlib.font_manager.findSystemFonts(
                fontpaths=None, fontext='ttf')
            if len(system_fonts):
                SourceVideosRenderer.labels_font = ImageFont.truetype(
                    system_fonts[0], size=24)

    @property
    def overlay_skeletons(self) -> bool:
        return self.__overlay_skeletons

    @property
    def overlay_bboxes(self) -> bool:
        return self.__overlay_bboxes

    @property
    def overlay_labels(self) -> bool:
        return self.__overlay_labels

    def render(self, meta: List[Dict[str, Any]], bboxes: Iterable[np.ndarray] = None, eval_slice: slice = slice(None), **kwargs) -> List[np.ndarray]:
        warnings.filterwarnings("ignore", category=DeprecationWarning)

        rendered_videos = len(meta['video_id'])
        start_offset = eval_slice.start if eval_slice.start is not None else 0
        stop_offset = int(meta['end_frame'][0] - meta['start_frame'][0] -
                          eval_slice.stop) if eval_slice.stop is not None else 0

        for clip_idx in range(rendered_videos):
            video = self.render_clip(
                meta['set_name'][clip_idx] if 'set_name' in meta else '',
                meta['video_id'][clip_idx],
                meta['pedestrian_id'][clip_idx],
                meta['clip_id'][clip_idx],
                meta['start_frame'][clip_idx] + start_offset,
                meta['end_frame'][clip_idx] - stop_offset,
                bboxes[clip_idx] if bboxes is not None else None,
                [{
                    'keypoints': sk['keypoints'][clip_idx],
                    'color': sk['color'],
                    'type': sk['type']
                } for sk in meta['skeletons']] if 'skeletons' in meta else None,
                {
                    k: v[clip_idx]
                    for k, v in meta['labels'].items()
                } if 'labels' in meta else None
            )
            yield video

    def render_clip(self, set_name, video_id, pedestrian_id, clip_id, start_frame, end_frame, bboxes=None, skeletons=None, labels=None):
        (canvas_width, canvas_height) = self.image_size
        half_width = int(math.floor(canvas_width / 2))
        half_height = int(math.floor(canvas_height / 2))
        canvas = np.zeros((end_frame - start_frame, canvas_height,
                          canvas_width, 3), dtype=np.uint8)

        paths = glob.glob(os.path.join(
            self.__data_dir, set_name, '{}.*'.format(os.path.splitext(video_id)[0])))
        try:
            assert len(paths) == 1
            with pims.PyAVReaderIndexed(paths[0]) as video:
                clip = video[start_frame:end_frame]
                (clip_height, clip_width, _) = clip.frame_shape

                if bboxes is None or not self.__center_bboxes:
                    centers = np.array([(clip_width/2, clip_height/2)] *
                                       (end_frame - start_frame)).round().astype(np.int)
                elif isinstance(bboxes, np.ndarray):
                    centers = (bboxes.mean(axis=-2) + 0.5).round().astype(np.int)
                else:
                    centers = (bboxes.mean(dim=-2) +
                               0.5).round().cpu().numpy().astype(int)

                for idx in trange(len(clip), desc='Frame', leave=False):
                    self.render_frame(canvas[idx], clip[idx],
                                      (half_width, half_height),
                                      centers[idx],
                                      (clip_width, clip_height),
                                      skeletons=[{
                                          'keypoints': np.array(sk['keypoints'][idx], np.int32),
                                          'color': sk['color'],
                                          'type': sk['type']
                                      } for sk in skeletons] if skeletons is not None and self.overlay_skeletons else None,
                                      bbox=bboxes[idx] if bboxes is not None and self.overlay_bboxes else None,
                                      labels={k: (v if isinstance(v, str) else v[idx]) for k, v in labels.items(
                                      )} if labels is not None and self.overlay_labels else None,
                                      )
        except AssertionError:
            # no video or multiple candidates - skip
            logging.getLogger(__name__).warn(
                "Clip extraction failed for {}, {}, {}, {}".format(set_name, video_id, pedestrian_id, clip_id))

        return canvas

    @ staticmethod
    def render_frame(canvas, clip, frame_half_size, bbox_center, clip_size, skeletons=None, bbox=None, labels=None):
        (half_width, half_height) = frame_half_size
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
               frame_width] = clip[frame_y_min:frame_y_max, frame_x_min:frame_x_max]

        if skeletons is not None:
            for skeleton in skeletons:
                SourceVideosRenderer.overlay_skeleton(
                    canvas, skeleton, (canvas_x_shift-frame_x_min, canvas_y_shift-frame_y_min))

        if bbox is not None:
            SourceVideosRenderer.overlay_bbox(
                canvas, bbox, (canvas_x_shift-frame_x_min, canvas_y_shift-frame_y_min))

        if labels is not None:
            SourceVideosRenderer.overlay_labels(
                canvas, labels)

        return canvas

    @staticmethod
    def overlay_bbox(canvas, bbox, shift=(0, 0)):
        ((x_min, y_min), (x_max, y_max)) = bbox
        (x_min, y_min, x_max, y_max) = (
            x_min+shift[0], y_min+shift[1], x_max+shift[0], y_max+shift[1])

        img, draw, end = SourceVideosRenderer.get_img_draw(canvas)
        draw.rectangle((x_min, y_min, x_max, y_max), outline=(0, 0, 255, 255)[:end])

        canvas[:] = np.array(img)

    @staticmethod
    def get_img_draw(canvas):
        no_channels = canvas.shape[-1]
        has_alpha = no_channels == 4
        img = Image.fromarray(canvas, 'RGBA' if has_alpha else 'RGB')
        draw = ImageDraw.Draw(img, 'RGBA' if has_alpha else 'RGB')

        return img, draw, no_channels

    @staticmethod
    def overlay_skeleton(canvas, skeleton, shift=(0, 0)):
        keypoints = skeleton['keypoints']
        skeleton_type = skeleton['type']
        color = skeleton['color']

        shifted_points = keypoints + np.array(shift)

        canvas[:] = PointsRenderer.draw_projection_points(
            canvas,
            shifted_points,
            skeleton_type,
            color_values=[color]*len(skeleton_type) if color is not None else None,
            lines=True
        )

    @staticmethod
    def overlay_labels(canvas, labels):
        img, draw, end = SourceVideosRenderer.get_img_draw(canvas)

        text_to_draw = '\n'.join([f'{k}: {v}' for k, v in labels.items()])
        draw.text(
            (5, 5),
            text_to_draw,
            fill=(0, 0, 0, 255)[:end],
            stroke_fill=(255, 255, 255, 255)[:end],
            stroke_width=1,
            font=SourceVideosRenderer.labels_font,
            align='left')

        canvas[:] = np.array(img)

    def save(self, frames: np.ndarray, name: str = 'out', outputs_dir: str = None, fps: int = 30) -> None:
        if outputs_dir is None:
            outputs_dir = os.path.join(os.getcwd(), 'source_videos_renderer')
        os.makedirs(outputs_dir, exist_ok=True)

        output_filename = os.path.join(outputs_dir, name + '.mp4')

        with av.open(output_filename, mode="w") as container:
            stream = container.add_stream('libx264', rate=fps)
            stream.width = self.image_size[0]
            stream.height = self.image_size[1]
            stream.pix_fmt = "yuv420p"
            stream.options = {}

            for img in frames:
                frame = av.VideoFrame.from_ndarray(img, format="rgb24")
                frame.pict_type = "NONE"

                for packet in stream.encode(frame):
                    container.mux(packet)

            for packet in stream.encode():
                container.mux(packet)
