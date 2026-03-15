import os
from typing import Iterable, List, Union

import numpy as np

from .city_scapes_palette import CITY_SCAPES_PALETTE
from .renderer import Renderer
from PIL import Image, ImagePalette


class SegmentationRenderer(Renderer):
    def __init__(self, palette=CITY_SCAPES_PALETTE, **kwargs) -> None:
        super().__init__(**kwargs)

        self._palette = ImagePalette.ImagePalette(mode='RGB', palette=palette)
        self._bits = int(np.ceil(np.log2(len(palette) // 3)))

    def render(self, frames: Union['Tensor', np.ndarray], **kwargs) -> List[np.ndarray]:
        if isinstance(frames, np.ndarray):
            cpu_frames = frames
        else:
            cpu_frames = frames.cpu().numpy()

        cpu_frames = cpu_frames.round().astype(np.uint8)
        rendered_videos = len(cpu_frames)

        for clip_idx in range(rendered_videos):
            video = self.render_clip(cpu_frames[clip_idx])
            yield video

    def render_clip(self, clip: np.ndarray) -> np.ndarray:
        sequence = []
        for frame in clip:
            img = Image.fromarray(frame, mode='P')
            img.putpalette(self._palette)

            sequence.append(img)
        return sequence

    def save(self, frames: Iterable[Image.Image], name: str = 'out', outputs_dir: str = None, fps: int = 30) -> None:
        if outputs_dir is None:
            outputs_dir = os.path.join(os.getcwd(), 'segmentation_renderer')
        os.makedirs(outputs_dir, exist_ok=True)

        frames[0].save(
            os.path.join(outputs_dir, f'{name}.apng'),
            format='PNG',
            save_all=True,
            append_images=frames[1:],
            duration=1000 / fps,
            loop=1,
            compress_level=9,
            bits=self._bits,
        )
