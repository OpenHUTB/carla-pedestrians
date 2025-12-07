from typing import Dict, Iterable, Tuple
from importlib_metadata import metadata
import torch

from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON


try:
    from torch_geometric.data import Data, Batch

    class PedestrianData(Data):
        def __cat_dim__(self, key, value, *args, **kwargs):
            if key.startswith('meta'):
                return None
            else:
                return super().__cat_dim__(key, value, *args, **kwargs)

except ImportError:
    Data=None
    Batch=None
    class PedestrianData:
        def __init__(self, *args, **kwargs):
            raise NotImplementedError()


class GraphMixin:
    def __init__(self,
                 return_graph: bool = False,
                 **kwargs
                 ):
        super().__init__(**kwargs)
        self.return_graph = return_graph

        if self.return_graph:
            self.edge_index = self.input_nodes.get_edge_index()
            self.clip_length = kwargs.get('clip_length', 30)

    def process_graph(self, out: Tuple[torch.Tensor, Dict[str, torch.Tensor], Dict[str, Iterable]]) -> torch.Tensor:
        if not self.return_graph:
            return out

        projection_2d, targets, meta = out

        if self.clip_length == 1:
            return PedestrianData(
                x=projection_2d,
                edge_index=self.edge_index,
                **{
                    f'targets_{k}': v for k, v in targets.items()
                },
                meta=meta
            )

        # for temporal data - this is torch geometric temporal compatible format
        # i.e. they are using batch dimension as time dimension
        graph_batch = Batch.from_data_list([PedestrianData(
            x=projection_2d[i],
            edge_index=self.edge_index,
            **{
                f'targets_{k}': v[i] for k, v in targets.items() if len(v.shape) and v.shape[0] == self.clip_length
            }
        ) for i in range(self.clip_length)])

        # 'weird' targets
        for k, v in targets.items():
            if not len(v.shape) or v.shape[0] != self.clip_length:
                graph_batch[f'targets_{k}'] = v

        graph_batch.meta = meta

        return graph_batch
