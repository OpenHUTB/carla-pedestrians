import os
import pickle
from typing import Dict, List, Literal, Tuple
import numpy as np
import pandas
from tqdm.auto import tqdm
from pedestrians_video_2_carla.data.base.mixins.datamodule.benchmark_datamodule_mixin import BenchmarkDataModuleMixin

from pedestrians_video_2_carla.data.openpose.skeleton import BODY_25_SKELETON, COCO_SKELETON

from .yorku_openpose_datamodule import YorkUOpenPoseDataModule


class YorkUBenchmarkDataModule(BenchmarkDataModuleMixin, YorkUOpenPoseDataModule):
    """
    Datamodule that attempts to follow the train/val/test split & labeling conventions as set in:

    ```
    @inproceedings{kotseruba2021benchmark,
        title={{Benchmark for Evaluating Pedestrian Action Prediction}},
        author={Kotseruba, Iuliia and Rasouli, Amir and Tsotsos, John K},
        booktitle={Proceedings of the IEEE Winter Conference on Applications of Computer Vision (WACV)},
        pages={1258--1268},
        year={2021}
    }
    ```

    https://github.com/ykotseruba/PedestrianActionBenchmark
    """

    def __init__(self,
                 pose_pickles_dir: str,
                 pose_data: str = 'pickle',
                 **kwargs
                 ):
        self.pose_data = pose_data

        # update the 'data_nodes' kwargs **IN PLACE** so that it is correct when determining input_nodes
        kwargs['data_nodes'] = COCO_SKELETON if self.pose_data == 'pickle' else BODY_25_SKELETON

        super().__init__(**kwargs)

        self._pose_pickles_dir = os.path.join(self.datasets_dir, pose_pickles_dir)

        if self.pose_data == 'pickle':
            self._extract_additional_data = self._extract_additional_data_pickle

    @property
    def settings(self):
        return {
            **super().settings,
            'pose_data': self.pose_data,
        }

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        parser = parent_parser.add_argument_group('YorkUBenchmark Data Module')
        parser = BenchmarkDataModuleMixin.add_cli_args(parser)
        parser.add_argument('--pose_data', type=str, choices=['pickle', 'json'], default='pickle',
                            help='''Type of pose data to use.
                                    "pickle" are data provided in https://github.com/ykotseruba/PedestrianActionBenchmark,
                                    "json" are our OpenPose JSON files. Default is "pickle".
                                    Remember to set data_nodes to the correct skeleton (BODY_25_SKELETON for "json").
                            ''')

        # update default settings
        parser.set_defaults(
            data_nodes=COCO_SKELETON,
        )

        return parent_parser

    def _extract_additional_data_pickle(self, clips: List[pandas.DataFrame]):
        """
        Extract skeleton data from keypoint files. This potentially modifies data in place!

        :param clips: List of DataFrames
        :type clips: List[DataFrame]
        """
        pose_data = {}
        for file in os.listdir(self._pose_pickles_dir):
            with open(os.path.join(self._pose_pickles_dir, file), 'rb') as fid:
                set_name = os.path.splitext(file)[0].split('_')[1]
                try:
                    data = pickle.load(fid)
                except:
                    data = pickle.load(fid, encoding='bytes')
                pose_data[set_name] = data

        updated_clips = []
        for clip in tqdm(clips, desc='Extracting skeleton data', leave=False):
            pedestrian_info = clip.reset_index().sort_values('frame')

            set_name = pedestrian_info.iloc[0]['set_name'] if 'set_name' in pedestrian_info.columns else 'set01'
            video_id = pedestrian_info.iloc[0]['video']
            pedestrian_id = pedestrian_info.iloc[0]['id']
            start_frame = pedestrian_info.iloc[0]['frame']
            stop_frame = pedestrian_info.iloc[-1]['frame'] + 1

            # get the pose data for this clip
            for i, f in enumerate(range(start_frame, stop_frame, 1)):
                ped_frame_id = f'{f:05d}_{pedestrian_id}'
                try:
                    frame_pose_data = np.array(
                        pose_data[set_name][video_id][ped_frame_id]).reshape(-1, 2)  # COCO_SKELETON
                    # TODO: this pose data is normalized - how to convert back to pixels for display?
                except KeyError:
                    frame_pose_data = np.zeros((len(self.data_nodes), 2))
                pedestrian_info.at[pedestrian_info.index[i],
                                   'keypoints'] = frame_pose_data.tolist()

            updated_clips.append(pedestrian_info)

        return updated_clips

    def _get_splits(self) -> Dict[Literal['train', 'val', 'test'], List[str]]:
        """
        Get the splits for the dataset.
        """
        raise NotImplementedError()

    def _split_and_save_clips(self, clips):
        """
        Split the clips into train, val, and test clips based on the predefined split lists.
        """
        set_size = {}
        clips = pandas.concat(clips).set_index(self.full_index)
        clips.sort_index(inplace=True)

        splits = self._get_splits()
        for name, split_list in tqdm(splits.items(), desc='Saving clips', leave=False):
            mask = clips.index.get_level_values(self.video_index[0]).isin(split_list)
            clips_set = clips[mask]

            set_size[name] = self._process_clips_set(name, clips_set)

        return set_size
