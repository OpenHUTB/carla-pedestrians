from typing import List

try:
    import carla
except ModuleNotFoundError:
    from . import mock_carla as carla
    import warnings
    warnings.warn("Using mock carla.", ImportWarning)


def convert_vector2d_to_list(vector2d: 'carla.Vector2D') -> List[float]:
    return [
        vector2d.x,
        vector2d.y,
    ]


def convert_list_to_vector2d(vector2d_flat: List[float]) -> 'carla.Vector2D':
    return carla.Vector2D(x=float(vector2d_flat[0]), y=float(vector2d_flat[1]))


def convert_vector3d_to_list(vector3d: 'carla.Vector3D') -> List[float]:
    return [
        vector3d.x,
        vector3d.y,
        vector3d.z,
    ]


def convert_list_to_vector3d(vector3d_flat: List[float]) -> 'carla.Vector3D':
    return carla.Vector3D(x=vector3d_flat[0], y=vector3d_flat[1], z=vector3d_flat[2])


def convert_transform_to_list(transform: 'carla.Transform') -> List[float]:
    return [
        transform.location.x,
        transform.location.y,
        transform.location.z,
        transform.rotation.pitch,
        transform.rotation.yaw,
        transform.rotation.roll,
    ]


def convert_list_to_transform(transform_flat: List[float]) -> 'carla.Transform':
    return carla.Transform(
        carla.Location(x=transform_flat[0], y=transform_flat[1], z=transform_flat[2]),
        carla.Rotation(pitch=transform_flat[3],
                       yaw=transform_flat[4], roll=transform_flat[5])
    )
