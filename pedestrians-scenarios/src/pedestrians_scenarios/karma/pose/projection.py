from typing import Union
from pedestrians_scenarios.karma.pose.pose_dict import PoseDict, Pose2DDict


import carla
import cameratransform as ct
from pedestrians_scenarios.karma.cameras import Camera as KarmaCamera
from pedestrians_scenarios.karma.pose.skeleton import CARLA_SKELETON


def project_pose(world_pose: PoseDict, camera_transform: carla.Transform, km_camera: Union[KarmaCamera, carla.Sensor]) -> Pose2DDict:
    """
    Project a 3D world pose into a 2D pose.
    """

    # Convert current camera transform from CARLA coordinates
    # to cameratransform coordinates
    pos_x = camera_transform.location.x
    pos_y = -camera_transform.location.y
    elevation = camera_transform.location.z
    heading_deg = 90 + camera_transform.rotation.yaw
    tilt_deg = 90 + camera_transform.rotation.pitch
    roll_deg = -camera_transform.rotation.roll

    camera_ct = ct.Camera(
        ct.RectilinearProjection(
            image_width_px=int(km_camera.attributes['image_size_x']),
            image_height_px=int(km_camera.attributes['image_size_y']),
            view_x_deg=float(km_camera.attributes['fov']),
            sensor_width_mm=float(km_camera.attributes['lens_x_size'])*1000,
            sensor_height_mm=float(km_camera.attributes['lens_y_size'])*1000
        ),
        ct.SpatialOrientation(
            pos_x_m=pos_x,
            pos_y_m=pos_y,
            elevation_m=elevation,
            heading_deg=heading_deg,
            tilt_deg=tilt_deg,
            roll_deg=roll_deg
        )
    )

    # get the 2D pose
    projection = camera_ct.imageFromSpace([
        (bone.location.x, -bone.location.y, bone.location.z)
        for bone in world_pose.values()
    ], hide_backpoints=True)

    return {
        k: carla.Vector2D(x=v[0], y=v[1])
        for k, v in zip(CARLA_SKELETON, projection)
    }
