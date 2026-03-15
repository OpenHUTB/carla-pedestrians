import os
from ..constants import (JAAD_USECOLS,
                         JAAD_DIR)
from .yorku_openpose_datamodule import YorkUOpenPoseDataModule


class JAADOpenPoseDataModule(YorkUOpenPoseDataModule):
    def __init__(self,
                 sample_type='beh',
                 **kwargs
                 ):
        self.sample_type = sample_type

        super().__init__(
            dataset_dirname=JAAD_DIR,
            cross_label='crossing',
            data_filepath=os.path.join(JAAD_DIR, 'annotations.csv'),
            video_index=['video'],
            pedestrian_index=['id'],
            clips_index=['clip', 'frame'],
            df_usecols=JAAD_USECOLS,
            df_filters={'beh': [True]} if sample_type == 'beh' else None,
            converters={
                'crossing': lambda x: x == '1',
            },
            **kwargs
        )

    @property
    def settings(self):
        return {
            **super().settings,
            'sample_type': self.sample_type,
        }

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        YorkUOpenPoseDataModule.add_subclass_specific_args(parent_parser)
        parser = parent_parser.add_argument_group('JAADBenchmark Data Module')
        parser.add_argument('--sample_type', type=str,
                            choices=['beh', 'all'],
                            default='beh')

        return parent_parser
