import os
from typing import List, Type, Union

import numpy as np
from ..pose.skeleton import Skeleton, CARLA_SKELETON
from .renderer import Renderer
from PIL import Image, ImageDraw


class PointsRenderer(Renderer):
    def __init__(self, input_nodes: Type[Skeleton] = CARLA_SKELETON, **kwargs) -> None:
        super().__init__(**kwargs)
        self._input_nodes = input_nodes

    def render(self, frames: Union['Tensor', np.ndarray], **kwargs) -> List[np.ndarray]:
        rendered_videos = len(frames)

        if isinstance(frames, np.ndarray):
            cpu_frames = frames
        else:
            cpu_frames = frames.cpu().numpy()

        cpu_frames = cpu_frames[..., 0:2].round().astype(np.int)

        for clip_idx in range(rendered_videos):
            video = self.render_clip(cpu_frames[clip_idx])
            yield video

    def render_clip(self, clip: np.ndarray) -> np.ndarray:
        video = []

        for frame in clip:
            frame = self.render_frame(frame)
            video.append(frame)

        return self.alpha_behavior(np.stack(video))

    def render_frame(self, frame: np.ndarray) -> np.ndarray:
        canvas = np.zeros(
            (self.image_size[1], self.image_size[0], 4), np.uint8)

        rgba_frame = self.draw_projection_points(
            canvas, frame, self._input_nodes)

        return rgba_frame

    def save(self,
             frame: np.ndarray,
             name: Union[str, int] = 'reference',
             outputs_dir: str = None,
             ):
        """
        Saves the image.
        """
        if outputs_dir is None:
            outputs_dir = os.path.join(os.getcwd(), 'points_renderer')
        os.makedirs(outputs_dir, exist_ok=True)

        img = Image.fromarray(frame, 'RGBA')
        img.save(os.path.join(outputs_dir, '{:s}.png'.format("{:06d}_pose".format(name)
                                                             if isinstance(name, int) else name)), 'PNG')

    @staticmethod
    def draw_projection_points(canvas, points, skeleton, color_values=None, lines=False):
        """
        Draws the points on the copy of the canvas.
        """
        rounded_points = np.round(points).astype(int)

        end = canvas.shape[-1]
        has_alpha = end == 4
        img = Image.fromarray(canvas, 'RGBA' if has_alpha else 'RGB')
        draw = ImageDraw.Draw(img, 'RGBA' if has_alpha else 'RGB')

        if color_values is None:
            skeleton_colors = skeleton.get_colors()
            # ensure colors are in the same order as joints
            color_values = [skeleton_colors[k] for k in skeleton]

        # if we know that skeleton has root point, we can draw it
        root_point = skeleton.get_root_point() if skeleton is not None else None
        root_point = root_point.value if isinstance(
            root_point, Skeleton) else root_point
        if root_point is not None:
            draw.rectangle(
                [tuple(rounded_points[0] - 2), tuple(rounded_points[0] + 2)],
                fill=color_values[0][:end],
                outline=None
            )

        for idx, point in enumerate(rounded_points):
            if idx == root_point:
                continue
            draw.ellipse(
                [tuple(point - 2), tuple(point + 2)],
                fill=color_values[idx][:end],
                outline=None
            )

        if lines and hasattr(skeleton, 'get_edges'):
            height, width = canvas.shape[:2]
            edges = skeleton.get_edges()
            for edge in edges:
                line_start = tuple(rounded_points[edge[0].value])
                line_end = tuple(rounded_points[edge[1].value])
                # skip line if either one of the points is not visible
                if not (line_start[0] > 0 and line_start[1] > 0 and line_end[0] > 0 and line_end[1] > 0
                        and line_start[0] < width and line_start[1] < height and line_end[0] < width and line_end[1] < height):
                    continue
                draw.line(
                    [line_start, line_end],
                    fill=color_values[edge[0].value][:end],
                    width=2
                )

        return np.array(img)
