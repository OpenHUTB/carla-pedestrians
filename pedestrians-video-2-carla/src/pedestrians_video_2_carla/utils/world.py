from typing import Tuple
import torch
from torch import Tensor


def zero_world_loc(shape: Tuple, device: torch.device) -> Tensor:
    return torch.zeros((*shape, 3), device=device)


def zero_world_rot(shape: Tuple, device: torch.device) -> Tensor:
    ones = tuple([1] * len(shape))
    return torch.eye(3, device=device).reshape(
        (*ones, 3, 3)).repeat((*shape, 1, 1))


def calculate_world_from_changes(
    shape: Tuple,
    device: torch.device,
    world_loc_change_batch: Tensor = None,
    world_rot_change_batch: Tensor = None,
    initial_world_loc: Tensor = None,
    initial_world_rot: Tensor = None,
) -> Tuple[Tensor, Tensor]:
    batch_size, clip_length, *_ = shape

    if initial_world_loc is None:
        initial_world_loc = zero_world_loc((batch_size,), device=device)

    if initial_world_rot is None:
        initial_world_rot = zero_world_rot((batch_size,), device=device)

    if world_loc_change_batch is None and world_rot_change_batch is None:
        # bail out if no changes
        return (
            initial_world_loc.unsqueeze(1).repeat(1, clip_length, 1),
            initial_world_rot.unsqueeze(1).repeat(1, clip_length, 1, 1),
        )

    if world_loc_change_batch is None:
        world_loc_change_batch = zero_world_loc((batch_size, clip_length),
                                                device=device)  # no world loc change

    if world_rot_change_batch is None:
        world_rot_change_batch = zero_world_rot((batch_size, clip_length),
                                                device=device)  # no world rot change

    world_loc = torch.empty(
        (batch_size, clip_length+1, *world_loc_change_batch.shape[2:]), device=device)
    world_rot = torch.empty(
        (batch_size, clip_length+1, *world_rot_change_batch.shape[2:]), device=device)

    world_loc[:, 0] = initial_world_loc
    world_rot[:, 0] = initial_world_rot

    # for every frame in clip
    for i in range(clip_length):
        world_rot[:, i+1] = torch.bmm(
            world_rot[:, i],
            world_rot_change_batch[:, i]
        )
        world_loc[:, i+1] = world_loc[:, i] + world_loc_change_batch[:, i]

    return world_loc[:, 1:], world_rot[:, 1:]
