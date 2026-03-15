from pedestrians_video_2_carla.data.carla.reference import get_absolute_tensors, get_projections
from pedestrians_video_2_carla.transforms.reference_skeletons import ReferenceSkeletonsDenormalize
import torch


def test_reference_skeletons_denormalization_identity(device):
    denormalizer = ReferenceSkeletonsDenormalize()

    abs_reference = get_absolute_tensors(device, as_dict=True)

    abs_tensor = torch.stack(tuple([v[0] for v in abs_reference.values()]), dim=0)
    meta = {
        'age': [],
        'gender': []
    }
    for (age, gender) in abs_reference.keys():
        meta['age'].append(age)
        meta['gender'].append(gender)

    denormalized = denormalizer.from_abs(abs_tensor, meta, autonormalize=True)

    assert torch.allclose(abs_tensor, denormalized), "Abs poses are not equal"

    abs_tensor_scaled = abs_tensor * torch.rand((1), device=device)
    denormalized_scaled = denormalizer.from_abs(
        abs_tensor_scaled, meta, autonormalize=True)

    assert torch.allclose(
        abs_tensor,
        denormalized_scaled,
        rtol=1e-4,
        atol=1e-4
    ), "Abs poses are not equal when input is scaled"

    projections = get_projections(device)

    denormalized_projection = denormalizer.from_projection(
        projections, meta, autonormalize=True)
    assert torch.allclose(
        projections, denormalized_projection), "Projections are not equal"

    projections_scaled = projections.clone()
    projections_scaled[..., 0:2] = projections[..., 0:2] * \
        torch.rand((1), device=device)
    denormalized_projection_scaled = denormalizer.from_projection(
        projections_scaled, meta, autonormalize=True)

    assert torch.allclose(
        projections,
        denormalized_projection_scaled,
        rtol=1e-4,
        atol=1e-4
    ), "Projections are not equal when input is scaled"
