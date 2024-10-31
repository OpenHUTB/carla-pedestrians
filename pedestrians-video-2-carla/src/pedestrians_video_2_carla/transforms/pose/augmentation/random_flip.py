from typing import Type
import torch

from pedestrians_video_2_carla.data.base.skeleton import Skeleton
from pedestrians_video_2_carla.utils.tensors import atleast_4d, get_missing_joints_mask


class RandomFlip:
    def __init__(self, nodes: Type[Skeleton], generator=None, prob=0.5):
        self.nodes = nodes
        self.flip_mask = self.nodes.get_flip_mask()
        self.prob = prob
        self.generator = generator

    def __call__(self, pose: torch.Tensor, centers: torch.Tensor = None, bboxes: torch.Tensor = None, clip_size: torch.Tensor = None, is_flipped: torch.Tensor = None):
        """
        Randomly flips the pose in X (left/right) dimension.
        **IT MODIFIES THE INPUTS (pose, bboxes & centers) IN PLACE**

        :param pose: (B,L,P,2|3) 2D pose with or without confidence or 3D pose.
        :type pose: torch.Tensor
        :param centers: (B,L,1+) centers of the bounding boxes. If not provided, zeros are used.
        :type centers: torch.Tensor
        :param bboxes: (B,L,2,2) bounding boxes with top-left and bottom-right coordinates. If not provided, bboxes **and centers** are not updated.
        :type bboxes: torch.Tensor
        :param clip_size: (B,2) size of the video clip. If not provided or zeros, bboxes and centers are not updated.
        :type clip_size: torch.Tensor
        :param is_flipped: A tensor of shape (B,) indicating whether the pose should be flipped or not. If not provided, it will be randomized.
        :type is_flipped: torch.Tensor

        :return: (B,) is_flipped tensor.
        :rtype: torch.Tensor
        """
        if is_flipped is None:
            is_flipped = torch.rand(
                (pose.shape[0],), device=pose.device, generator=self.generator) < self.prob

        if not is_flipped.any():
            return is_flipped

        if centers is None:
            centers = torch.zeros(
                (*pose.shape[:1], 1), device=pose.device, dtype=pose.dtype)

        # remember undetected joints
        pose_mask = ~get_missing_joints_mask(pose)

        flip_mul = torch.ones_like(pose)
        flip_mul[..., 0] *= -1.0

        pose[is_flipped] = atleast_4d(pose[is_flipped][..., self.flip_mask, :])
        pose[is_flipped, ..., 0] = pose[is_flipped, ..., 0].sub_(
            centers[is_flipped, ..., 0]).mul_(flip_mul[is_flipped, ..., 0])

        # shift the bounding box & centers
        # so it reflects where flipped skeleton would be
        # if it was extracted from the flipped image
        if bboxes is not None and clip_size is not None and torch.all(clip_size):
            half_clip_widths = (clip_size[is_flipped, 0] / 2.0)[:, None, None]
            bboxes[is_flipped, ..., 0] = bboxes[is_flipped, ..., 0].sub_(
                half_clip_widths).mul_(-1.0).add_(half_clip_widths)
            bboxes[is_flipped, ..., 0] = torch.flip(
                bboxes[is_flipped, ..., 0], dims=(-1,))
            centers[is_flipped, ..., 0] = bboxes[is_flipped].mean(
                dim=-2, keepdim=True)[..., 0]

        pose[is_flipped, ..., 0] = pose[is_flipped, ..., 0].add_(
            centers[is_flipped, ..., 0])

        # restore undetected joints
        pose[pose_mask] = 0.0

        return is_flipped

    def __repr__(self):
        return self.__class__.__name__ + '(nodes={}, prob={})'.format(self.nodes.__name__, self.prob)
