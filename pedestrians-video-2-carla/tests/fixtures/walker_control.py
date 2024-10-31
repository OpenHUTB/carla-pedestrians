import os
import pytest
from pedestrians_video_2_carla.data.carla.utils import load, yaml_to_pose_dict

from pedestrians_video_2_carla.walker_control.controlled_pedestrian import ControlledPedestrian
from pedestrians_video_2_carla.walker_control.pose import Pose
from pedestrians_video_2_carla.walker_control.p3d_pose import P3dPose
from pedestrians_video_2_carla.walker_control.pose_projection import PoseProjection
from pedestrians_video_2_carla.walker_control.p3d_pose_projection import P3dPoseProjection


@pytest.fixture(params=[Pose, P3dPose])
def pose_cls(request):
    return request.param


@pytest.fixture(params=[PoseProjection, P3dPoseProjection])
def pose_projection_cls(request):
    return request.param


@pytest.fixture
def pedestrian(pose_cls, device):
    """
    Returns unbound ControlledPedestrian
    """
    return ControlledPedestrian(None, 'adult', 'female', reference_pose=pose_cls, device=device)


@pytest.fixture(scope="session")
def carla_world():
    try:
        from pedestrians_video_2_carla.carla_utils.destroy import destroy_client_and_world
        from pedestrians_video_2_carla.carla_utils.setup import setup_client_and_world
    except ImportError:
        pytest.skip(
            "`carla` package not found. Functionality depending on it will not work!")
    try:
        client, world = setup_client_and_world()
        yield world
        destroy_client_and_world(client, world, {})
    except RuntimeError as e:
        pytest.skip("Could not connect to CARLA. Original error: {}".format(e))


@pytest.fixture
def carla_pedestrian(carla_world, pose_cls, device):
    """
    Returns ControlledPedestrian bound to specific CARLA world instance
    """
    return ControlledPedestrian(carla_world, 'adult', 'female', reference_pose=pose_cls, device=device)


@pytest.fixture
def absolute_pose():
    unreal_abs_pose = load('sk_female_absolute.yaml')
    return yaml_to_pose_dict(unreal_abs_pose['transforms'], is_abs=True)[0]


@pytest.fixture
def relative_pose():
    unreal_rel_pose = load('sk_female_relative.yaml')
    return yaml_to_pose_dict(unreal_rel_pose['transforms'])[0]


@pytest.fixture
def reference_pose(pose_cls, relative_pose, device):
    p = pose_cls(device=device)
    p.relative = relative_pose
    return p
