import os
from typing import Dict, List, Literal

from .yorku_benchmark_datamodule import YorkUBenchmarkDataModule

from ..constants import JAAD_DIR, JAAD_USECOLS


class JAADBenchmarkDataModule(YorkUBenchmarkDataModule):
    def __init__(self,
                 jaad_data_variant='default',
                 sample_type='beh',
                 **kwargs
                 ):
        self.jaad_data_variant = jaad_data_variant
        self.sample_type = sample_type

        super().__init__(
            dataset_dirname=JAAD_DIR,
            pose_pickles_dir=os.path.join(JAAD_DIR, 'poses'),
            data_filepath=os.path.join(JAAD_DIR, 'annotations.csv'),
            video_index=['video'],
            pedestrian_index=['id'],
            clips_index=['clip', 'frame'],
            df_usecols=JAAD_USECOLS,
            df_filters={'beh': [True]} if sample_type == 'beh' else None,
            **kwargs
        )

        self._splits_dir = os.path.join(
            self.datasets_dir, JAAD_DIR, 'split_ids', self.jaad_data_variant)

    @property
    def settings(self):
        return {
            **super().settings,
            'jaad_data_variant': self.jaad_data_variant,
            'sample_type': self.sample_type,
        }

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        YorkUBenchmarkDataModule.add_subclass_specific_args(parent_parser)
        parser = parent_parser.add_argument_group('JAADBenchmark Data Module')
        parser.add_argument('--jaad_data_variant', type=str,
                            choices=['default', 'high_visibility', 'all_videos'],
                            default='default')
        parser.add_argument('--sample_type', type=str,
                            choices=['beh', 'all'],
                            default='beh')

        # update default settings
        parser.set_defaults(
            clip_length=16,
            clip_offset=3,
        )

        return parent_parser

    def _get_splits(self) -> Dict[Literal['train', 'val', 'test'], List[str]]:
        """
        Get the splits for the dataset.
        """
        splits = {}

        for name in ['train', 'val', 'test']:
            splits[name] = open(os.path.join(self._splits_dir,
                                             f'{name}.txt'), 'r').read().splitlines()

        return splits
