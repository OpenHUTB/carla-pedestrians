import numpy as np
import warnings

try:
    import carla
except (ImportError, ModuleNotFoundError) as e:
    import pedestrians_video_2_carla.carla_utils.mock_carla as carla
    warnings.warn("Using mock carla.", source=e)


def test_unbound_transform(pedestrian):
    pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ))

    assert np.isclose(pedestrian.transform.location.x, 0.5)
    assert np.isclose(pedestrian.transform.location.y, 0.5)
    assert np.isclose(pedestrian.transform.location.z, 0)
    assert np.isclose(pedestrian.transform.rotation.pitch, 0)
    assert np.isclose(pedestrian.transform.rotation.yaw, -30)
    assert np.isclose(pedestrian.transform.rotation.roll, 0)

    pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ))

    assert np.isclose(pedestrian.transform.location.x, 1)
    assert np.isclose(pedestrian.transform.location.y, 1)
    assert np.isclose(pedestrian.transform.location.z, 0)
    assert np.isclose(pedestrian.transform.rotation.pitch, 0)
    assert np.isclose(pedestrian.transform.rotation.yaw, -60)
    assert np.isclose(pedestrian.transform.rotation.roll, 0)


def test_unbound_world_transform(pedestrian):
    pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ))

    assert np.isclose(pedestrian.world_transform.location.x, 0.5)
    assert np.isclose(pedestrian.world_transform.location.y, 0.5)
    assert np.isclose(pedestrian.world_transform.location.z, 0)
    assert np.isclose(pedestrian.world_transform.rotation.pitch, 0)
    assert np.isclose(pedestrian.world_transform.rotation.yaw, -30)
    assert np.isclose(pedestrian.world_transform.rotation.roll, 0)

    pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ))

    assert np.isclose(pedestrian.world_transform.location.x, 1)
    assert np.isclose(pedestrian.world_transform.location.y, 1)
    assert np.isclose(pedestrian.world_transform.location.z, 0)
    assert np.isclose(pedestrian.world_transform.rotation.pitch, 0)
    assert np.isclose(pedestrian.world_transform.rotation.yaw, -60)
    assert np.isclose(pedestrian.world_transform.rotation.roll, 0)


def test_bound_transform(carla_pedestrian):
    carla_pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ), True)

    assert np.isclose(carla_pedestrian.transform.location.x, 0.5)
    assert np.isclose(carla_pedestrian.transform.location.y, 0.5)
    # we cannot test Z due to uneven world surface
    assert np.isclose(carla_pedestrian.transform.rotation.pitch, 0)
    assert np.isclose(carla_pedestrian.transform.rotation.yaw, -30)
    assert np.isclose(carla_pedestrian.transform.rotation.roll, 0)

    carla_pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ), True)

    assert np.isclose(carla_pedestrian.transform.location.x, 1)
    assert np.isclose(carla_pedestrian.transform.location.y, 1)
    # we cannot test Z due to uneven world surface
    assert np.isclose(carla_pedestrian.transform.rotation.pitch, 0)
    assert np.isclose(carla_pedestrian.transform.rotation.yaw, -60)
    assert np.isclose(carla_pedestrian.transform.rotation.roll, 0)


def test_bound_world_transform(carla_pedestrian):
    carla_pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ), True)

    assert np.isclose(carla_pedestrian.world_transform.location.x,
                      carla_pedestrian.initial_transform.location.x + 0.5)
    assert np.isclose(carla_pedestrian.world_transform.location.y,
                      carla_pedestrian.initial_transform.location.y + 0.5)
    # we cannot test Z due to uneven world surface
    assert np.isclose(carla_pedestrian.world_transform.rotation.pitch,
                      carla_pedestrian.initial_transform.rotation.pitch)
    assert np.isclose(carla_pedestrian.world_transform.rotation.yaw,
                      carla_pedestrian.initial_transform.rotation.yaw + -30)
    assert np.isclose(carla_pedestrian.world_transform.rotation.roll,
                      carla_pedestrian.initial_transform.rotation.roll)

    carla_pedestrian.teleport_by(carla.Transform(
        location=carla.Location(
            x=0.5,
            y=0.5
        ),
        rotation=carla.Rotation(
            yaw=-30
        )
    ), True)

    assert np.isclose(carla_pedestrian.world_transform.location.x,
                      carla_pedestrian.initial_transform.location.x + 1)
    assert np.isclose(carla_pedestrian.world_transform.location.y,
                      carla_pedestrian.initial_transform.location.y + 1)
    # we cannot test Z due to uneven world surface
    assert np.isclose(carla_pedestrian.world_transform.rotation.pitch,
                      carla_pedestrian.initial_transform.rotation.pitch)
    assert np.isclose(carla_pedestrian.world_transform.rotation.yaw,
                      carla_pedestrian.initial_transform.rotation.yaw + -60)
    assert np.isclose(carla_pedestrian.world_transform.rotation.roll,
                      carla_pedestrian.initial_transform.rotation.roll)


def test_carla_rendering(carla_world, carla_pedestrian, test_outputs_dir):
    import random
    import os
    from queue import Queue, Empty
    from collections import OrderedDict
    from pedestrians_video_2_carla.carla_utils.setup import setup_camera
    from pedestrians_video_2_carla.walker_control.pose_projection import PoseProjection
    from pedestrians_scenarios.karma.renderers.points_renderer import PointsRenderer

    sensor_dict = OrderedDict()
    camera_queue = Queue()

    sensor_dict['camera_rgb'] = setup_camera(
        carla_world, camera_queue, carla_pedestrian
    )

    base_projection = PoseProjection(carla_pedestrian, sensor_dict['camera_rgb'])
    renderer = PointsRenderer()

    ticks = 0
    while ticks < 10:
        w_frame = carla_world.tick()

        try:
            sensor_data = camera_queue.get(True, 1.0)
            sensor_data.save_to_disk(
                os.path.join(test_outputs_dir, '{:06d}.png'.format(sensor_data.frame))
            )
            # TODO: since pedestrian was adjusted to have hips at 0,0,0,
            # we need to adjust the projection to have the same offset
            assert False
            points = base_projection.current_pose_to_points()
            rendered_points = renderer.render_frame(points)
            renderer.save(rendered_points, '{:06d}'.format(sensor_data.frame), test_outputs_dir)
            ticks += 1
        except Empty:
            print("Some sensor information is missed in frame {:06d}".format(w_frame))

        # teleport/rotate pedestrian a bit to see if teleport_by is working
        carla_pedestrian.teleport_by(carla.Transform(
            location=carla.Location(
                x=random.random()-0.5,
                y=random.random()-0.5
            ),
            rotation=carla.Rotation(
                yaw=random.random()*60-30
            )
        ))

        # apply some movement to the left arm to see apply_movement in action
        carla_pedestrian.update_pose({
            'crl_arm__L': carla.Rotation(yaw=-random.random()*15),
            'crl_foreArm__L': carla.Rotation(pitch=-random.random()*15)
        })


def test_if_pose_and_carla_bone_transform_match():
    assert False
