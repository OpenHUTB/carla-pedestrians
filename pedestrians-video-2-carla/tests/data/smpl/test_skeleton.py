from pedestrians_video_2_carla.data.smpl.skeleton import SMPL_SKELETON
import torch
import math


def test_mappings_are_reversible():
    smpl_len = len(SMPL_SKELETON)
    random_radians = (torch.rand((4, smpl_len*3)) - 0.5) * math.pi

    mapped = SMPL_SKELETON.map_from_original(random_radians)

    assert mapped.shape == (4, smpl_len, 3)

    reversed = SMPL_SKELETON.map_to_original(mapped)

    assert reversed.shape == (4, smpl_len*3)
    assert torch.allclose(random_radians, reversed)
