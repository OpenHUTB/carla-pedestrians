import torch

from pedestrians_video_2_carla.transforms.bbox import BBoxExtractor


def test_bbox_extractor():
    extractor = BBoxExtractor(input_nodes=None)

    points = torch.tensor([
        (
            (0.0, 0.0),
            (100.0, 100.0),
            (100.0, 400.0),
            (500.0, 100.0),
            (500.0, 400.0),
        ),
        (
            (300.0, 250.0),
            (100.0, 100.0),
            (100.0, 400.0),
            (500.0, 100.0),
            (500.0, 400.0),
        ),
        (
            (300.0, 250.0),
            (200.0, 150.0),
            (200.0, 350.0),
            (400.0, 150.0),
            (400.0, 350.0),
        )
    ])

    (shift, scale) = extractor.get_shift_scale(points)

    assert torch.allclose(shift, torch.tensor(
        [[300.0, 250.0], [300.0, 250.0], [300.0, 250.0]]))
    assert torch.allclose(scale, torch.tensor([150.0, 150.0, 100.0]))
