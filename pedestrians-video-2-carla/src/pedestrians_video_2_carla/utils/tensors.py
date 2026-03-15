import torch


def eye_batch(batch_size, joints, device=torch.device('cpu'), dtype=torch.float32, size=3):
    """
    Returns a batch of identity matrices for the given batch size and number of joints.
    """
    return torch.eye(size, device=device, dtype=dtype).reshape(
        (1, 1, size, size)).repeat((batch_size, joints, 1, 1))


def get_bboxes(sample, near_zero=1e-5) -> torch.Tensor:
    """
    Returns the bounding boxes for the given sample of Skeleton points.
    """
    missing_points_mask = torch.all(sample[..., 0:2] < near_zero, dim=-1)
    detected_points_min = sample.clone()
    detected_points_min[missing_points_mask] = torch.ones_like(
        detected_points_min[missing_points_mask]) * float('inf')
    minimums, _ = detected_points_min.min(dim=-2)
    detected_points_max = sample.clone()
    detected_points_max[missing_points_mask] = torch.ones_like(
        detected_points_max[missing_points_mask]) * float('-inf')
    maximums, _ = detected_points_max.max(dim=-2)

    return torch.stack((minimums, maximums), dim=-2)


def get_missing_joints_mask(common_gt, hips=None, input_indices=None):
    mask = torch.all(common_gt != 0, dim=-1)  # missing joints are 'perfect' zeros

    # do not mask hips joint if it exists
    if hips is not None:
        if isinstance(input_indices, slice):
            common_hips_idx = hips.value
        else:
            common_hips_idx = input_indices.index(hips.value)
        mask[..., common_hips_idx] = 1

    return mask


def nan_to_zero(sample: torch.Tensor) -> torch.Tensor:
    if getattr(torch, 'nan_to_num', False):
        sample = torch.nan_to_num(
            sample, nan=0, posinf=0, neginf=0)
    else:
        sample = torch.where(torch.isnan(
            sample), torch.tensor(0.0, device=sample.device), sample)
        sample = torch.where(torch.isinf(
            sample), torch.tensor(0.0, device=sample.device), sample)

    return sample


def atleast_4d(sample: torch.Tensor) -> torch.Tensor:
    if sample.ndim < 4:
        shape_4d = (None, None, None, None, *((slice(None),)*sample.ndim))[-4:]
        sample = sample[shape_4d]
    return sample
