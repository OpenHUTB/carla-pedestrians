import os

import numpy as np
import torch
from pedestrians_scenarios.karma.renderers.points_renderer import \
    PointsRenderer
from pedestrians_video_2_carla.data import DATASETS_BASE
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.smpl.skeleton import (_ORIG_SMPL_SKELETON,
                                                          SMPL_SKELETON)
from pedestrians_video_2_carla.data.smpl.smpl_dataset import SMPLDataset
from pedestrians_video_2_carla.utils.world import (zero_world_loc,
                                                   zero_world_rot)
from PIL import Image


def test_convert_smpl_to_carla(test_data_dir, test_outputs_dir, device):
    # data_dir & set_filepath are not used in this test, but are required by the SMPLDataset class
    amass_dataset = SMPLDataset(
        data_dir=os.path.join(DATASETS_BASE, 'AMASS'),
        set_filepath=os.path.join(test_data_dir, 'AMASSDataModule',
                                  'subsets', '21b78507376adbe8edc2253d6edf8cda', 'train.csv'),
        points=CARLA_SKELETON,
        device=device
    )

    models_to_test = [
        ('adult', 'female'),
        ('adult', 'male')
    ]

    joints_moves_to_test = np.array((
        # roll, pitch, yaw
        (0.0, 0.0, 90.0),
        (0.0, 90.0, 0.0),
        (90.0, 0.0, 0.0),
    ))

    # reference + each joint * joints_moves_to_test + cumulative example
    batch_size = amass_dataset.smpl_nodes_len * len(joints_moves_to_test) + 2

    # prepare moves batch
    # -----------------------------------------------------------------------------
    moves = torch.tensor(
        np.deg2rad(joints_moves_to_test),
        dtype=torch.float32,
        device=device)

    smpl_pose = torch.zeros((batch_size, amass_dataset.smpl_nodes_len, 3),
                            dtype=torch.float32, device=device)
    names = ['reference']

    for i in range(amass_dataset.smpl_nodes_len):
        for a in range(3):
            smpl_pose[i*3+a+1, i] = moves[a].clone()
            names.append('{}_{}'.format(
                _ORIG_SMPL_SKELETON(i).name, joints_moves_to_test[a]))

    names.append('left_arm_cumulative')
    smpl_pose[batch_size-1, _ORIG_SMPL_SKELETON.L_Shoulder.value] = moves[0].clone()
    smpl_pose[batch_size-1, _ORIG_SMPL_SKELETON.L_Elbow.value] = moves[0].clone()

    # moves batch ready
    # -----------------------------------------------------------------------------

    outputs_dir = os.path.join(test_outputs_dir, 'projections')
    os.makedirs(outputs_dir, exist_ok=True)

    smpl_renderer = PointsRenderer(SMPL_SKELETON)
    carla_renderer = PointsRenderer(CARLA_SKELETON)

    world_loc = zero_world_loc((batch_size,), device)
    world_rot = zero_world_rot((batch_size,), device)

    for (age, gender) in models_to_test:
        modifications = [[] for _ in range(batch_size)]

        pose_body = smpl_pose.reshape((batch_size, -1))
        _, _, smpl_abs, _, smpl_proj, smpl_pp = amass_dataset.get_clip_projection(
            pose_body,
            SMPL_SKELETON,
            age,
            gender,
            world_loc,
            world_rot
        )
        _, _, carla_abs, _, carla_proj, carla_pp = amass_dataset.get_clip_projection(
            pose_body,
            CARLA_SKELETON,
            age,
            gender,
            world_loc,
            world_rot
        )

        # take into account the shift in elevation between SMPL and CARLA coordinates
        for perspective in (
            ((0.0, 3.1, 0), (0.0, 3.1, 1.2)),
            ((0.01, 0.0, 1.9), (0.01, 0.0, 3.1)),
            ((2.2, 2.2, 0), (2.2, 2.2, 1.2)),
            None
        ):
            for i in range(batch_size):
                carla_canvas = carla_renderer.render_frame(carla_proj[i, :, :2].cpu().numpy())
                smpl_canvas = smpl_renderer.render_frame(smpl_proj[i, :, :2].cpu().numpy())

                modifications[i].append(np.concatenate(
                    (smpl_canvas, carla_canvas), axis=1))

                if perspective is not None:
                    smpl_pp.update_camera(perspective[0])
                    carla_pp.update_camera(perspective[1])

                    smpl_proj = smpl_pp(smpl_abs, world_loc, world_rot)
                    carla_proj = carla_pp(carla_abs, world_loc, world_rot)

        for mi, rows in enumerate(modifications):
            full = np.concatenate(rows, axis=0)
            img = Image.fromarray(full, 'RGBA')
            img.save(os.path.join(outputs_dir,
                                  f'full_pose_{gender}_{names[mi]}.png'), 'PNG')

        # And now manually check if the rendered poses look correct ;)
