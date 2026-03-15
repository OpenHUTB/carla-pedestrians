import numpy as np
import torch


def gaussian_kernel(size_w, size_h, center_x, center_y, sigma):
    # copied from third_party/unipose/utils/penn_action_data.py
    # + clipping added
    gridy, gridx = np.mgrid[0:size_h, 0:size_w]
    D2 = (gridx - center_x) ** 2 + (gridy - center_y) ** 2
    gmap = np.exp(-D2 / 2.0 / sigma / sigma)
    gmap[gmap > 1] = 1
    gmap[gmap < 0.0099] = 0

    return torch.from_numpy(gmap).unsqueeze(0).float()  # (1, H, W)
