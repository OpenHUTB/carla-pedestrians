import copy
import logging
from typing import Any, Dict, Iterable, Optional, Tuple, Type
import numpy as np
from pedestrians_video_2_carla.data.base.skeleton import Skeleton
from torch.utils.data import Dataset, IterableDataset
from pedestrians_video_2_carla.data.base.mixins.dataset.confidence_mixin import ConfidenceMixin
from pedestrians_video_2_carla.data.base.mixins.dataset.graph_mixin import GraphMixin
from pedestrians_video_2_carla.data.base.mixins.dataset.projection_2d_mixin import Projection2DMixin
import torch
import h5py

from pedestrians_video_2_carla.data.base.skeleton import get_common_indices


class TorchDataset(Dataset):
    # needed to allow for multiple inheritance
    def __init__(self, *args, **kwargs) -> None:
        super().__init__()


class TorchIterableDataset(IterableDataset):
    # needed to allow for multiple inheritance
    def __init__(self, *args, **kwargs) -> None:
        super().__init__()


class BaseDataset(Projection2DMixin, ConfidenceMixin, GraphMixin, TorchDataset):
    def __init__(
        self,
        set_filepath,
        data_nodes: Type[Skeleton],
        input_nodes: Type[Skeleton] = None,
        skip_metadata: bool = False,
        class_labels: Dict[str, Iterable[str]] = None,
        is_training: bool = False,
        **kwargs
    ) -> None:
        """
        Initializes the dataset using the given filepath.

        :param set_filepath: Path to the set file
        :type set_filepath: str
        :param data_nodes: Nodes as present in the data
        :type data_nodes: Type[Skeleton]
        :param input_nodes: Nodes as expected by the model. Defaults to data_nodes.
        :type input_nodes: Type[Skeleton]
        :param skip_metadata: Whether to skip the metadata (default: False). Skipping metadata loading speeds up the dataset creation.
        :type skip_metadata: bool
        :param class_labels: Labels for classification tasks
        :type class_labels: Dict[str, Iterable[str]]
        """
        self.data_nodes = data_nodes
        self.input_nodes = input_nodes if input_nodes is not None else data_nodes
        self.num_input_joints = len(self.input_nodes)
        self.num_data_joints = len(self.data_nodes)

        super().__init__(**kwargs)

        self._set_filepath = set_filepath
        self._skip_metadata = skip_metadata
        self._is_training = is_training

        self._load_data()

        self.input_indices, self.data_indices = get_common_indices(
            input_nodes=self.data_nodes,
            output_nodes=self.input_nodes
        )
        self.class_labels = class_labels or {}

        # cache cross classification labels
        # this needs to be done on labels that are already decoded in meta
        # since encoded meta can differ between train/val/test depending
        # on what kind of sample is first in the subset
        self.class_ints = {}
        for key in self.class_labels.keys():
            if key in self.meta[0]:
                self.class_ints[key] = self._encode_labels(
                    key, [self.meta[i][key] for i in range(len(self.meta))])

    def _load_data(self, ignore_metadata=False) -> None:
        self.set_file = h5py.File(self._set_filepath, 'r', driver='core')

        self.projection_2d = self.set_file['projection_2d']

        if not ignore_metadata:
            if self._skip_metadata:
                self.meta = [{}] * len(self)
            else:
                self.meta = self._decode_meta(self.set_file['meta'])

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            if k in ['set_file', 'projection_2d']:
                setattr(result, k, None)
            else:
                setattr(result, k, copy.deepcopy(v, memo))

        # get a new h5df file handle
        result._load_data(ignore_metadata=True)

        return result

    def _decode_meta_value(self, value: Any, meta_item: Any, key: str) -> Any:
        if 'labels' in meta_item.attrs:
            return meta_item.attrs['labels'][value].decode("latin-1")

        if isinstance(value, np.floating):
            return float(value)

        if isinstance(value, np.integer):
            return int(value)

        if value.dtype.kind == 'S':
            return value.decode("latin-1")

        return value

    def _decode_meta(self, meta):
        logging.getLogger(__name__).debug(
            'Decoding meta for {}...'.format(self.set_file.filename))

        out = [{
            k: self._decode_meta_value(v[idx], meta[k], k)
            for k, v in meta.items()
        } for idx in range(len(self))]

        logging.getLogger(__name__).debug('Meta decoding done.')

        return out

    def __len__(self):
        return len(self.projection_2d)

    def _get_raw_projection_2d(self, idx: int) -> torch.Tensor:
        """
        Returns the raw 2D projection of the clip.

        :param idx: Clip index
        :type idx: int
        """
        return torch.from_numpy(self.projection_2d[idx]), {}

    def _get_targets(self, idx: int, raw_projection_2d: torch.Tensor, intermediate_outputs: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        return {
            k: v[idx:idx+1] for k, v in self.class_ints.items()
        }

    def _get_meta(self, idx: int) -> Dict[str, Any]:
        return self.meta[idx]

    def _get_common_tensor(self, data_item):
        # map data nodes to input nodes expected by the model
        # assumption: data_tensor.shape[1] is num_data_joints
        # TODO: some other data may accidentally have the same size as num_data_joints, it would be incorrectly mapped
        if not isinstance(data_item, torch.Tensor) or len(data_item.shape) < 2 or data_item.shape[1] != self.num_data_joints:
            return data_item

        input_tensor = torch.zeros(
            (data_item.shape[0], self.num_input_joints, *data_item.shape[2:]), dtype=data_item.dtype, device=data_item.device)
        input_tensor[:, self.input_indices] = data_item[:, self.data_indices]

        return input_tensor

    def _map_nodes(self, projection_2d: torch.Tensor, projection_targets: Dict[str, torch.Tensor], targets: Dict[str, torch.Tensor]) -> Tuple[torch.Tensor, Dict[str, torch.Tensor], Dict[str, torch.Tensor]]:
        """
        Maps the input nodes to the data nodes.

        :param projection_2d: 2D projection
        :type projection_2d: torch.Tensor
        :param projection_targets: 2D projection targets
        :type projection_targets: Dict[str, torch.Tensor]
        :param targets: targets
        :type targets: Dict[str, torch.Tensor]
        """
        # map data nodes to input nodes expected by the model
        return (
            self._get_common_tensor(projection_2d),
            {
                k: self._get_common_tensor(v)
                for k, v in projection_targets.items()
            },
            {
                k: self._get_common_tensor(v)
                for k, v in targets.items()
            }
        )

    def _encode_labels(self, key: str, labels: Iterable[str]) -> torch.Tensor:
        """
        Helper method to convert string labels into numeric classes.

        :param key: field name
        :type key: str
        :param labels: list/tuple/etc. of labels
        :type labels: Iterable[str]
        :return: Labels as integers
        :rtype: torch.Tensor
        """
        return torch.Tensor([self.class_labels[key].index(label) for label in labels]).long()

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, Dict[str, torch.Tensor], Dict[str, Any]]:
        """
        Returns a single clip.

        :param idx: Clip index
        :type idx: int
        """
        try:
            raw_projection_2d, intermediate_outputs = self._get_raw_projection_2d(idx)
            targets = self._get_targets(idx, raw_projection_2d, intermediate_outputs)
            meta = self._get_meta(idx)

            projection_2d, projection_targets = self.process_projection_2d(
                raw_projection_2d, targets, meta)
            projection_2d = self.process_confidence(projection_2d)

            if self.data_nodes != self.input_nodes:
                projection_2d, projection_targets, targets = self._map_nodes(
                    projection_2d, projection_targets, targets)

            out = (projection_2d, {
                **targets,
                **projection_targets,
            }, meta)

            return self.process_graph(out)
        except IndexError as e:
            raise IndexError(
                'Index {} out of range for dataset of length {}'.format(idx, len(self))) from e
