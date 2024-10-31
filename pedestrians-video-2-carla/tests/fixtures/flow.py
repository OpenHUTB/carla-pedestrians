import os
import pytest

from pedestrians_video_2_carla.loss import LossModes
from pedestrians_video_2_carla.modules.movements.movements import MovementsModelOutputType
from pedestrians_video_2_carla.modules.movements import MOVEMENTS_MODELS
from pedestrians_video_2_carla.modules.trajectory import TRAJECTORY_MODELS


@pytest.fixture(scope="session")
def test_root_dir():
    """
    Create a directory for the test logs.
    """
    import tempfile
    root_dir = tempfile.mkdtemp()
    yield root_dir
    import shutil
    shutil.rmtree(root_dir)


@pytest.fixture(scope="session")
def test_data_dir():
    return os.path.join(os.path.dirname(__file__), '..', 'test_data')


@pytest.fixture(params=list(LossModes.__members__.keys()))
def loss_mode(request, movements_output_type):
    supported = {
        MovementsModelOutputType.pose_changes: list(LossModes),
        MovementsModelOutputType.absolute_loc_rot: [
            LossModes.common_loc_2d,
            LossModes.loc_3d,
            LossModes.rot_3d,
            LossModes.loc_2d_3d,
            LossModes.loc_2d_loc_rot_3d,
            LossModes.weighted_loc_2d_loc_rot_3d
        ],
        MovementsModelOutputType.absolute_loc: [
            LossModes.common_loc_2d,
            LossModes.loc_3d,
            LossModes.loc_2d_3d,
        ],
        MovementsModelOutputType.relative_rot: [
            LossModes.common_loc_2d,
            LossModes.loc_3d,
            LossModes.rot_3d,
            LossModes.loc_2d_3d,
            LossModes.loc_2d_loc_rot_3d,
            LossModes.weighted_loc_2d_loc_rot_3d
        ],
        MovementsModelOutputType.pose_2d: [
            LossModes.common_loc_2d
        ]
    }[MovementsModelOutputType[movements_output_type]]
    if LossModes[request.param] not in supported:
        pytest.skip("Loss mode {} not supported for projection type {}".format(
            request.param, movements_output_type))
    return request.param


# those renderers should always be available
@pytest.fixture(params=['input_points', 'projection_points', 'none'])
def renderer(request):
    return request.param


@pytest.fixture(params=list(set(MovementsModelOutputType.__members__.keys()) - {'pose_2d'}))
def movements_output_type(request):
    return request.param


# all models should be able to run with default settings
# TODO: searate autoencoder-only flow models from pose-lifting-only models
@pytest.fixture(params=list(set(MOVEMENTS_MODELS.keys()) - {'LinearAE2D', 'GNNLinearAutoencoder', 'VariationalGcn'}))
def movements_model_name(request):
    return request.param


# all models should be able to run with default settings
@pytest.fixture(params=TRAJECTORY_MODELS.keys())
def trajectory_model_name(request):
    return request.param
