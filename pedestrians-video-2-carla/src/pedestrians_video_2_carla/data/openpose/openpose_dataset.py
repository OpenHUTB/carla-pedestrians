from typing import Dict, Union, Type
import torch
from pedestrians_video_2_carla.data.base.base_dataset import BaseDataset
from pedestrians_video_2_carla.data.openpose.skeleton import BODY_25_SKELETON, COCO_SKELETON


class OpenPoseDataset(BaseDataset):
    def __init__(self,
                 set_filepath,
                 data_nodes: Union[Type[BODY_25_SKELETON],
                                   Type[COCO_SKELETON]] = BODY_25_SKELETON,
                 **kwargs
                 ) -> None:

        if kwargs.get('strong_points', 0) != 1 and (
            sum(kwargs.get('missing_joint_probabilities', [])) != 0 or
            kwargs.get('noise', 'zero') != 'zero'
        ):
            raise ValueError(
                'strong_points should be 1 if artificial missing joints and/or noise are requested.')

        super().__init__(set_filepath=set_filepath, data_nodes=data_nodes, **kwargs)

    def _get_targets(self, idx: int, *args, **kwargs) -> Dict[str, torch.Tensor]:
        targets = super()._get_targets(idx, *args, **kwargs)

        targets['bboxes'] = torch.from_numpy(self.set_file['targets/bboxes'][idx])

        return targets
