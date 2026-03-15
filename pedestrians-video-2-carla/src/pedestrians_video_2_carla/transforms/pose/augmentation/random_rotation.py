import torch

from pedestrians_video_2_carla.utils.tensors import get_bboxes, get_missing_joints_mask


class RandomRotation:
    def __init__(self, generator=None, max_rotation_angle: float = 10.0):
        self.max_rotation_angle = max_rotation_angle
        self.generator = generator

    def __call__(self, pose: torch.Tensor, centers: torch.Tensor = None, bboxes: torch.Tensor = None, rotation: torch.Tensor = None):
        """
        Randomly rotates the pose in Z (clockwise/counter-clockwise) dimension.
        **IT MODIFIES THE INPUTS (pose, bboxes & centers) IN PLACE**

        :param pose: (B,L,P,2|3) 2D pose with or without confidence or 3D pose.
        :type pose: torch.Tensor
        :param centers: (B,L,1+) centers of the bounding boxes. If not provided, bboxes are calculated from pose (if needed) and their centers are used.
        :type centers: torch.Tensor, optional
        :param bboxes: (B,L,2,2) bounding boxes with top-left and bottom-right coordinates. If not provided, bboxes are not updated.
        :type bboxes: torch.Tensor, optional
        :param rotation: A tensor of shape (B,) with angles (in degrees) of rotation. If not provided, it will be randomized.
        :type rotation: torch.Tensor, optional
        :return: (B,) rotation tensor.
        :rtype: torch.Tensor
        """
        if rotation is None:
            rotation = (torch.rand(pose.shape[:-3], generator=self.generator,
                                   dtype=pose.dtype, device=pose.device) * 2 - 1) * self.max_rotation_angle

        if centers is None:
            pose_bboxes = get_bboxes(pose) if bboxes is None else bboxes
            centers = pose_bboxes.mean(dim=-2, keepdim=True)

        # remember undetected joints
        pose_mask = ~get_missing_joints_mask(pose)

        rotation_radians = torch.deg2rad(rotation)
        rotation_cos = torch.cos(rotation_radians)
        rotation_sin = torch.sin(rotation_radians)
        rotation_matrix = torch.stack((
            torch.stack((rotation_cos, -rotation_sin)),
            torch.stack((rotation_sin, rotation_cos)))
        ).permute(2, 0, 1).unsqueeze_(1)

        pose[..., :2] = pose[..., :2].sub_(
            centers).matmul(rotation_matrix).add_(centers)

        # restore undetected joints
        pose[pose_mask] = 0.0

        if bboxes is not None:
            other_corners = bboxes.clone()
            other_corners[..., 1, 1] = bboxes[..., 0, 1]
            other_corners[..., 0, 1] = bboxes[..., 1, 1]
            all_corners = torch.cat((bboxes, other_corners), dim=-2)
            all_corners = all_corners.sub_(centers).matmul(
                rotation_matrix).add_(centers)
            minimums, _ = all_corners.min(dim=-2)
            maximums, _ = all_corners.max(dim=-2)
            new_bboxes = torch.stack((minimums, maximums), dim=-2)
            bboxes[:] = new_bboxes

        return rotation

    def __repr__(self):
        return self.__class__.__name__ + '(max_rotation_angle={})'.format(self.max_rotation_angle)