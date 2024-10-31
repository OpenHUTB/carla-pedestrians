import os
from typing import Iterable, Tuple, Union
import numpy as np
from .segmentation_renderer import SegmentationRenderer
from .dvs_palette import DYNAMIC_VISION_SENSOR_PALETTE
from PIL import Image
from enum import Flag, auto


class DVSOutputFormat(Flag):
    img_frame = auto()
    events = auto()


class DVSRenderer(SegmentationRenderer):
    def __init__(
        self,
        palette=DYNAMIC_VISION_SENSOR_PALETTE,
        mode: DVSOutputFormat = DVSOutputFormat.img_frame | DVSOutputFormat.events,
        **kwargs
    ) -> None:
        super().__init__(palette=palette, **kwargs)

        self._mode = mode

    def render_clip(self, clip: Iterable[np.ndarray]) -> np.ndarray:
        img_frame = None
        events_stream = None

        if DVSOutputFormat.img_frame in self._mode:
            frames = np.zeros(
                (len(clip), self.image_size[1], self.image_size[0]),
                dtype=np.uint8
            )
            for frame_idx, events in enumerate(clip):
                frames[frame_idx, events[:]['y'], events[:]['x']
                       ] = events[:]['pol'] + 1  # +1 to avoid 0
            img_frame = super().render_clip(frames)

        events = np.concatenate(clip, axis=0)
        min_ts = max(events['t'].min() - 1, 0)

        # adjust timestamps to start from 1
        events['t'] -= min_ts

        if DVSOutputFormat.events in self._mode:
            # sort in ascending order by timestamp
            events.sort(axis=0, order=['t', 'x', 'y'])
            events_stream = events

        return (img_frame, events_stream)

    def save(self, frames: Tuple[Iterable[Image.Image], np.ndarray], name: str = 'out', outputs_dir: str = None, fps: int = 30) -> None:
        if outputs_dir is None:
            outputs_dir = os.path.join(os.getcwd(), 'dvs_renderer')

        (img_frame, events_stream) = frames

        if events_stream is not None:
            np.save(os.path.join(outputs_dir, name), events_stream, allow_pickle=False)

        if img_frame is not None:
            super().save(img_frame, name=name, outputs_dir=outputs_dir, fps=fps)

