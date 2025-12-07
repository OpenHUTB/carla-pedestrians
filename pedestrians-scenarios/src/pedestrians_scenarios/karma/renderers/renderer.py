from enum import Enum
from typing import List, Tuple, Union

import numpy as np


class AlphaBehavior(Enum):
    drop = 0
    blend = 1
    keep = 2


class Renderer(object):
    def __init__(self, image_size: Tuple[int, int] = (800, 600), alpha: AlphaBehavior = AlphaBehavior.drop, **kwargs) -> None:
        self._alpha = alpha

        # public to allow runtime overriding
        self.image_size = image_size

    def render(self, frames: Union['Tensor', np.ndarray], **kwargs) -> List[np.ndarray]:
        """
        Renders black clip and repeats it N times in the output.
        Number of channels depends on the defined alpha behavior.

        :param frames: Batch of sequence data (N, L, ...)
        :type frames: Tensor
        :return: List with size N of black clips (L, height, width, channels)
        :rtype: List[np.ndarray]
        """
        rendered_videos = len(frames)
        for _ in range(rendered_videos):
            yield self.alpha_behavior(np.zeros((frames.shape[1], self.image_size[1], self.image_size[0], 4), dtype=np.uint8))

    def alpha_behavior(self, rgba_frames: np.ndarray) -> np.ndarray:
        if self._alpha == AlphaBehavior.drop:
            return rgba_frames[..., 0:3].astype(np.uint8)
        elif self._alpha == AlphaBehavior.blend:
            return ((rgba_frames[..., 0:3] * 255.0) / rgba_frames[..., 3:4]).round().astype(np.uint8)
        # keep
        return rgba_frames
