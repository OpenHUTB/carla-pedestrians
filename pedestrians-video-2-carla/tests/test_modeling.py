"""
Sanity checks to see if the overall flow is working.
"""
from pedestrians_video_2_carla.data import DATASETS_BASE, DEFAULT_ROOT, OUTPUTS_BASE
from pedestrians_video_2_carla.modeling import main
import os
import shutil
import glob


def test_flow(test_root_dir, loss_mode, movements_output_type):
    """
    Test the overall flow using Linear model.
    """
    main([
        "--data_module_name=Carla2D3D",
        "--movements_model_name=Linear",
        "--batch_size=2",
        "--val_set_size=2",
        "--test_set_size=2",
        "--num_workers=0",
        "--clip_length=32",
        "--input_nodes=CARLA_SKELETON",
        "--output_nodes=CARLA_SKELETON",
        "--max_epochs=1",
        "--limit_val_batches=1",
        "--limit_train_batches=1",
        "--loss_modes",
        loss_mode,
        "--renderers",
        "none",
        "--movements_output_type={}".format(movements_output_type),
        "--root_dir={}".format(test_root_dir),
    ])


def test_flow_needs_confidence(test_root_dir):
    """
    Test the basic flow using Linear model with needs_confidence flag enabled.
    """
    main([
        "--data_module_name=Carla2D3D",
        "--movements_model_name=Linear",
        "--batch_size=2",
        "--val_set_size=2",
        "--test_set_size=2",
        "--num_workers=0",
        "--clip_length=32",
        "--input_nodes=CARLA_SKELETON",
        "--output_nodes=CARLA_SKELETON",
        "--max_epochs=1",
        "--limit_val_batches=1",
        "--limit_train_batches=1",
        "--loss_modes",
        "common_loc_2d",
        "--renderers",
        "none",
        "--movements_output_type=pose_changes",
        "--needs_confidence",
        "--root_dir={}".format(test_root_dir),
    ])


def test_renderer(test_root_dir, renderer):
    """
    Test the renderers using Linear model.
    """
    experiment_dir = main([
        "--data_module_name=Carla2D3D",
        "--movements_model_name=Linear",
        "--batch_size=2",
        "--val_set_size=2",
        "--test_set_size=2",
        "--num_workers=0",
        "--clip_length=32",
        "--input_nodes=CARLA_SKELETON",
        "--output_nodes=CARLA_SKELETON",
        "--max_epochs=1",
        "--limit_val_batches=1",
        "--limit_train_batches=1",
        "--loss_modes",
        "common_loc_2d",
        "--renderers",
        renderer,
        "--root_dir={}".format(test_root_dir),
        "--seed=0",
    ])

    # assert the experiments log dir exists
    assert os.path.exists(experiment_dir), 'Experiment logs dir was not created'

    # assert no video files were created
    if renderer == 'none':
        assert not os.path.exists(os.path.join(
            experiment_dir, "videos")), 'Videos dir was created'


def test_source_videos_jaad(test_root_dir, test_data_dir):
    """
    Test the source videos rendering using JAADOpenPoseDataModule.
    """
    # JAADOpenPoseDataModule will look for the subsets in the tmp directory
    # and fail if it can't find the required files.
    shutil.copytree(
        os.path.join(test_data_dir, 'JAADOpenPoseDataModule'),
        os.path.join(test_root_dir, OUTPUTS_BASE, 'JAADOpenPoseDataModule'),
        dirs_exist_ok=True
    )

    # We're not going to include the videos in the repo, so optionally provide the path
    source_videos_dir = os.getenv('JAAD_SOURCE_VIDEOS_DIR', os.path.join(
        DEFAULT_ROOT, DATASETS_BASE, 'JAAD', 'videos'))

    experiment_dir = main([
        "--data_module_name=JAADOpenPose",
        "--movements_model_name=Linear",
        "--batch_size=8",
        "--num_workers=0",
        "--clip_length=32",
        "--input_nodes=BODY_25_SKELETON",
        "--output_nodes=CARLA_SKELETON",
        "--max_epochs=0",
        "--limit_train_batches=0",
        "--limit_val_batches=1",
        "--loss_modes=common_loc_2d",
        "--max_videos=4",
        "--renderers",
        "source_videos",
        "--source_videos_dir={}".format(source_videos_dir),
        "--root_dir={}".format(test_root_dir),
    ])

    video_dir = os.path.join(experiment_dir, "videos", "val")

    assert os.path.exists(video_dir), 'Videos dir was not created'

    videos = glob.glob(os.path.join(video_dir, '**', '*.mp4'))

    assert len(videos) == 4, 'Video files were not created'


def test_pose_lifting(test_root_dir, movements_model_name, trajectory_model_name):
    """
    Test the overall flow using specified models.
    """
    main([
        "--data_module_name=Carla2D3D",
        "--flow=pose_lifting",
        "--movements_model_name={}".format(movements_model_name),
        "--trajectory_model_name={}".format(trajectory_model_name),
        "--batch_size=2",
        "--val_set_size=2",
        "--test_set_size=2",
        "--num_workers=0",
        "--clip_length=32",
        "--input_nodes=CARLA_SKELETON",
        "--output_nodes=CARLA_SKELETON",
        "--max_epochs=1",
        "--limit_val_batches=1",
        "--limit_train_batches=1",
        "--loss_modes",
        "common_loc_2d",
        "--renderers",
        "none",
        "--root_dir={}".format(test_root_dir),
    ])


def test_weighted_loss(test_root_dir):
    """
    Test the overall flow using Linear model.
    """
    main([
        "--data_module_name=Carla2D3D",
        "--movements_model_name=Linear",
        "--batch_size=2",
        "--val_set_size=2",
        "--test_set_size=2",
        "--num_workers=0",
        "--clip_length=32",
        "--input_nodes=CARLA_SKELETON",
        "--output_nodes=CARLA_SKELETON",
        "--max_epochs=1",
        "--limit_val_batches=1",
        "--limit_train_batches=1",
        "--loss_modes",
        'weighted_loc_2d_loc_rot_3d',
        "--loss_weights",
        "common_loc_2d=1.0",
        "loc_3d=1.0",
        "rot_3d=3.0",
        "--renderers",
        "none",
        "--movements_output_type=absolute_loc_rot",
        "--root_dir={}".format(test_root_dir),
    ])
