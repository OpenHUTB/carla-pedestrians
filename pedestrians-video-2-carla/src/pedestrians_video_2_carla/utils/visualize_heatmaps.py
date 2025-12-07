import os
import matplotlib
import numpy as np
import torch
from PIL import Image
from pedestrians_video_2_carla.data import DEFAULT_ROOT, OUTPUTS_BASE


def visualize_heatmaps(
    clip_frames: torch.Tensor,
    clip_heatmaps: torch.Tensor,
    clip_idx: int = 0,
    frame_idx: int = 0,
    colormap_name: str = 'jet',
    output_path: str = os.path.join(DEFAULT_ROOT, OUTPUTS_BASE, 'heatmaps.png'),
) -> None:
    """
    Visualize the heatmaps.

    :param clip_frames: (B, T, C, H, W) tensor of the clip frames
    :param clip_heatmaps: (B, T, P, H, W) tensor of the heatmaps, where P is the number of joints + 1 for the background
    """

    # get the numpy arrays
    frame = clip_frames[clip_idx, frame_idx].detach().cpu().numpy()
    heatmaps = clip_heatmaps[clip_idx, frame_idx].detach().cpu().numpy()

    images = []

    # convert frame to PIL Image; reverse ImageNet normalization
    im = Image.fromarray(np.clip((frame.transpose(1, 2, 0) * np.array([0.229, 0.224, 0.225]) + np.array(
        [0.485, 0.456, 0.406])) * 255.0, 0, 255).astype(np.uint8))
    images.append(im)

    cm = matplotlib.cm.get_cmap(colormap_name)

    # convert heatmaps to PIL Image; move background to the end
    for heat in (heatmaps[1:] + heatmaps[0:1]):
        heatmap = Image.fromarray((255.0*cm(heat)[..., :3]).astype(np.uint8))
        im_heat = Image.blend(im, heatmap, 0.8)
        images.append(im_heat)

    # combine images
    im_combined = Image.fromarray(np.concatenate(
        [np.array(im) for im in images], axis=0))
    im_combined.save(output_path)
