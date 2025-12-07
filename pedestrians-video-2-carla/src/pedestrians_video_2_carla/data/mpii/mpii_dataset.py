from typing import Dict, Union, Type
import torch
from pedestrians_video_2_carla.data.base.base_dataset import BaseDataset
from .skeleton import MPII_SKELETON


class MPIIDataset(BaseDataset):
    def __init__(self,
                 set_filepath,
                 data_nodes: Type[MPII_SKELETON] = MPII_SKELETON,
                 **kwargs
                 ) -> None:

        super().__init__(set_filepath=set_filepath, data_nodes=data_nodes, **kwargs)
