import os
from typing import Iterable, List, Union
from .renderer import Renderer
import numpy as np


class LidarRenderer(Renderer):
    def __init__(
        self,
        **kwargs
    ) -> None:
        super().__init__(**kwargs)

    def render(self, frames: Union['Tensor', np.ndarray], **kwargs) -> List[np.ndarray]:
        if isinstance(frames, np.ndarray):
            cpu_frames = frames
        else:
            cpu_frames = frames.cpu().numpy()

        rendered_videos = len(cpu_frames)

        for clip_idx in range(rendered_videos):
            video = self.render_clip(cpu_frames[clip_idx])
            yield video

    def render_clip(self, clip: Iterable[np.ndarray]) -> np.ndarray:
        points = np.concatenate(clip, axis=0)
        min_ts = max(points['t'].min() - 1, 0)
        max_ts = points['t'].max() + 1

        # adjust timestamps to start from 1
        points['t'] -= min_ts

        # tmp sanity check - max timestamp should be less than clip length * (1000 / fps)
        assert max_ts < len(clip) * (1000 / 30.0) + 1

        # sort in ascending order by timestamp
        points.sort(axis=0, order=['t', 'x', 'y'])
        return points

    def save(self, frames: np.ndarray, name: str = 'out', outputs_dir: str = None) -> None:
        if outputs_dir is None:
            outputs_dir = os.path.join(os.getcwd(), 'lidar_renderer')

        return np.save(os.path.join(outputs_dir, name), frames, allow_pickle=False)
