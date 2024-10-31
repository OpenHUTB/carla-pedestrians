from typing import Union
import numpy as np
import torch
from torchvision.transforms.functional import equalize, normalize, resize


class VideoToResNet:
    def __init__(self, target_size: int = 368, **kwargs):
        self._target_size = target_size

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}(target_size={self._target_size})'

    def __call__(self, clip: Union[torch.Tensor, np.ndarray]) -> torch.Tensor:
        """
        Converts a video sequence to ResNet-compatible input.

        :param clip: A tensor of shape (T, H, W, C) where T is the number of frames, C is the number of channels,
                H is the height and W is the width. The tensor is assumed to be in the range [0, 255] (dtype=uint8).
        :type clip: Union[torch.Tensor, np.ndarray]

        :return: A tensor of shape (T, C, sH, sW) where T is the number of frames, C is the number of channels,
                sH and sW are such the smaller size equals target size and the larger size is scaled
                appropriately to maintain the aspect ratio (see torchvision.transforms.functional.resize).
                The tensor is in the range [0, 1].
        """
        # convert to tensor if necessary
        if not isinstance(clip, torch.Tensor):
            clip = torch.from_numpy(clip)

        # (T, H, W, C) -> (T, C, H, W)
        clip = clip.permute((0, 3, 1, 2))

        # equalize histogram of the images
        clip = equalize(clip)

        # resize if needed
        clip_length, _, clip_height, clip_width = clip.shape
        if clip_height > self._target_size or clip_width > self._target_size:
            scaled_canvas = []
            for idx in range(clip_length):
                resized = resize(
                    clip[idx],
                    self._target_size,
                    antialias=True,
                )

                scaled_canvas.append(resized)
            clip = torch.stack(scaled_canvas)

        # normalize
        clip = clip.div(255.0)
        clip = normalize(
            clip, [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])

        return clip.float()
