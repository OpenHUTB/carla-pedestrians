from pedestrians_video_2_carla.data.mixed.mixed_datamodule import MixedDataModule
from pedestrians_video_2_carla.data.carla.datamodules.carla_recorded_datamodule import CarlaRecordedDataModule
from pedestrians_video_2_carla.data.smpl.amass_datamodule import AMASSDataModule
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.smpl.skeleton import SMPL_SKELETON
from pedestrians_video_2_carla.utils.argparse import flat_args_as_list_arg


class CarlaRecAMASSDataModule(MixedDataModule):
    data_modules = [
        CarlaRecordedDataModule,
        AMASSDataModule
    ]
    # default mixing proportions
    train_proportions = [0.5, 0.5]
    val_proportions = [0.5, 0.5]
    test_proportions = [0.5, 0.5]

    def __init__(self, **kwargs):
        carla_missing_joint_probabilities = flat_args_as_list_arg(kwargs, 'missing_joint_probabilities', True)
        
        amass_missing_joint_probabilities = MixedDataModule._map_missing_joint_probabilities(
            carla_missing_joint_probabilities,
            CARLA_SKELETON,
            SMPL_SKELETON
        )

        super().__init__({
            CarlaRecordedDataModule: {
                'data_nodes': CARLA_SKELETON,
                'input_nodes': CARLA_SKELETON,
                'missing_joint_probabilities': carla_missing_joint_probabilities,
                'classification_targets_key': 'frame.pedestrian.is_crossing'
            },
            AMASSDataModule: {
                'data_nodes': SMPL_SKELETON,
                'input_nodes': CARLA_SKELETON,
                'missing_joint_probabilities': amass_missing_joint_probabilities,
            }
        }, **kwargs)
