import os
from typing import Any, Dict, Iterable, List, Literal, Optional, Type, Union
from pedestrians_scenarios.karma.pose.skeleton import CARLA_SKELETON
from pytorch_lightning import LightningDataModule
from pedestrians_video_2_carla.data.base.skeleton import Skeleton, get_common_indices
from pedestrians_video_2_carla.data.base.base_datamodule import BaseDataModule
from pedestrians_video_2_carla.data.mixed.mixed_dataset import MixedDataset
import numpy as np
import yaml

try:
    from yaml import CDumper as Dumper, CLoader as Loader
except ImportError:
    from yaml import Dumper, Loader


class MixedDataModule(LightningDataModule):
    data_modules: Iterable[BaseDataModule] = []

    # default mixing proportions; this should be overridden by subclasses
    train_proportions: Iterable[float] = []
    val_proportions: Iterable[float] = []
    test_proportions: Iterable[float] = []

    def __init__(
        self,
        data_modules_kwargs: Dict[Type[BaseDataModule], Dict[str, Any]],
        data_modules: List[Type[BaseDataModule]] = None,
        train_proportions: List[float] = None,
        val_proportions: List[float] = None,
        test_proportions: List[float] = None,
        subsets_dir: Union[List[str], str] = None,
        mappings: List[Dict[str, str]] = None,
        **kwargs
    ):
        all_data_modules = self.data_modules + (data_modules or [])
        assert len(all_data_modules) > 1, 'At least 2 data modules are required'

        super().__init__()

        # basic compatibility check
        uses_projection_mixin = [dm_cls.uses_projection_mixin()
                                 for dm_cls in all_data_modules]
        assert all(uses_projection_mixin) or not any(
            uses_projection_mixin), 'All data modules must use projection mixin or none of them can.'
        self._uses_projection_mixin = all(uses_projection_mixin)

        if kwargs.get('flow', 'pose_lifting') == 'classification':
            uses_classification_mixin = [dm_cls.uses_classification_mixin()
                                         for dm_cls in all_data_modules]
            assert all(uses_classification_mixin) or not any(
                uses_classification_mixin), 'All data modules must use cross mixin or none of them can.'
            self._uses_classification_mixin = all(uses_classification_mixin)

        self._skip_metadata = kwargs.get('skip_metadata', False)

        subsets_dirs = [None] * len(all_data_modules)
        if isinstance(subsets_dir, str):
            subsets_dirs[0] = subsets_dir
        elif isinstance(subsets_dir, list):
            assert len(subsets_dir) == len(all_data_modules)
            subsets_dirs = subsets_dir

        self.predict_set_name = None
        self.predict_set = None

        self._mappings = mappings

        self._data_modules: List[BaseDataModule] = [
            dm_cls(
                **{
                    **kwargs,
                    **data_modules_kwargs.get(dm_cls, {}),
                    'subsets_dir': subsets_dir,
                }
            ) for dm_cls, subsets_dir in zip(all_data_modules, subsets_dirs)
        ]

        for dm in self._data_modules:
            self.hparams.update(dm.hparams)

        self.requested_train_proportions = self._validate_proportions(
            train_proportions or self.train_proportions)
        self.requested_val_proportions = self._validate_proportions(
            val_proportions or self.val_proportions)
        self.requested_test_proportions = self._validate_proportions(
            test_proportions or self.test_proportions)

        self.hparams['train_proportions'] = self.requested_train_proportions
        self.hparams['val_proportions'] = self.requested_val_proportions
        self.hparams['test_proportions'] = self.requested_test_proportions
        self.hparams['mixed_datasets'] = [
            dm.__class__.__name__ for dm in self._data_modules]
        # explicitly set this to avoid confusion
        self.hparams['data_module_name'] = self.__class__.__name__
        self.hparams['data_nodes'] = 'Mixed'

        self.train_set = None
        self.val_set = None
        self.test_set = None

    @staticmethod
    def _map_missing_joint_probabilities(probabilities: List, input_nodes: Skeleton, output_nodes: Skeleton) -> List:
        """Helper method for mapping missing joint probabilities to the output skeleton.
        This should be used by subclasses when & as needed.

        :param probabilities: List of missing joint probabilities.
        :type probabilities: List
        :param input_nodes: Skeleton for which the probabilities are given.
        :type input_nodes: Skeleton
        :param output_nodes: Skeleton to which the probabilities should be mapped.
        :type output_nodes: Skeleton
        :return: Probabilities mapped to the output skeleton.
        :rtype: List
        """
        if len(probabilities) > 1:
            missing_joint_probabilities = np.array(probabilities)

            # what probability to use for joints that are not in the input_nodes?
            mean_missing_joint_probability = np.mean(missing_joint_probabilities)

            output_indices, input_indices = get_common_indices(
                input_nodes, output_nodes)
            mapped_missing_joint_probabilities = np.ones(
                len(output_nodes)) * mean_missing_joint_probability
            mapped_missing_joint_probabilities[output_indices] = missing_joint_probabilities[input_indices]

            return mapped_missing_joint_probabilities.tolist()
        else:
            return probabilities[:]

    @classmethod
    def uses_infinite_train_set(cls):
        # Mixing infinite datasets is not supported
        return False

    @property
    def subsets_dir(self) -> List[str]:
        return [dm.subsets_dir for dm in self._data_modules]

    @property
    def class_labels(self) -> List[str]:
        # assumption - all datamodules share labels
        return self._data_modules[0].class_labels

    @property
    def class_counts(self) -> Dict[Literal['train', 'val', 'test'], Dict[str, Dict[str, int]]]:
        class_counts = self._data_modules[0].class_counts
        mapped_keys = list(self._mappings.keys())

        for dm in self._data_modules[1:]:
            for set_name in dm.class_counts.keys():
                for cls_name, cls_values in dm.class_counts[set_name].items():
                    if cls_name not in class_counts[set_name]:
                        if cls_name in mapped_keys:
                            cls_name = self._mappings[cls_name]
                        else:
                            continue
                    for cls_value, cls_count in cls_values.items():
                        class_counts[set_name][cls_name][cls_value] += cls_count
        return class_counts

    def _validate_proportions(self, proportions: Iterable[float]) -> None:
        assert len(proportions) == len(
            self._data_modules), 'Proportions must be specified for each data module.'
        assert (all(0 <= p <= 1 for p in proportions) and sum(proportions)
                == 1) or all((p == 0 or p == -1) for p in proportions)
        return proportions

    @classmethod
    def add_data_specific_args(cls, parent_parser):
        parent_parser = BaseDataModule.add_data_specific_args(
            parent_parser, add_projection_2d_args=True, add_classification_args=True)

        for dm_cls in cls.data_modules:
            dm_cls: Type[BaseDataModule]
            parent_parser = dm_cls.add_subclass_specific_args(parent_parser)

        # mixing params
        parser = parent_parser.add_argument_group('Mixed Data Module')
        parser.add_argument(
            '--train_proportions',
            type=float,
            nargs='+',
            default=cls.train_proportions,
            help='Proportions of data to use for training. Must sum to 1 OR be a mix of -1s (all available data) and 0s (do not use this one). Order should be the same as data_modules.'
        )
        parser.add_argument(
            '--val_proportions',
            type=float,
            nargs='+',
            default=cls.val_proportions,
            help='Proportions of data to use for validation. Must sum to 1 OR be a mix of -1s (all available data) and 0s (do not use this one). Order should be the same as data_modules.'
        )
        parser.add_argument(
            '--test_proportions',
            type=float,
            nargs='+',
            default=cls.test_proportions,
            help='Proportions of data to use for testing. Must sum to 1 OR be a mix of -1s (all available data) and 0s (do not use this one). Order should be the same as data_modules.'
        )

        # set common input_nodes
        parser.set_defaults(
            input_nodes=CARLA_SKELETON,
        )

        return parent_parser

    def prepare_data(self) -> None:
        # TODO: run this in parallel?
        for dm in self._data_modules:
            dm.prepare_data()

    def setup(self, stage: Optional[str] = None) -> None:
        # TODO: run this in parallel?
        for dm in self._data_modules:
            dm.setup(stage)

        if stage == "fit" or stage is None:
            self.train_set = MixedDataset([
                dm.train_set for dm in self._data_modules
            ],
                skip_metadata=self._skip_metadata,
                proportions=self.requested_train_proportions,
                mappings=self._mappings,
            )
            self.val_set = MixedDataset([
                dm.val_set for dm in self._data_modules
            ],
                skip_metadata=self._skip_metadata,
                proportions=self.requested_val_proportions,
                mappings=self._mappings,
            )

            self.hparams['train_set_sizes'] = tuple(np.diff(
                self.train_set.cumulative_sizes, prepend=0))
            self.hparams['val_set_sizes'] = tuple(np.diff(
                self.val_set.cumulative_sizes, prepend=0))

        if stage == "test" or stage is None:
            self.test_set = MixedDataset([
                dm.test_set for dm in self._data_modules
            ],
                skip_metadata=self._skip_metadata,
                proportions=self.requested_test_proportions,
                mappings=self._mappings,
            )
            self.hparams['test_set_sizes'] = tuple(np.diff(
                self.test_set.cumulative_sizes, prepend=0))

    def choose_predict_set(self, set_name: str) -> None:
        for dm in self._data_modules:
            dm.choose_predict_set(set_name)

        self.predict_set = MixedDataset([
            dm.predict_set for dm in self._data_modules
        ],
            skip_metadata=self._skip_metadata,
            proportions=self.requested_test_proportions,
            mappings=self._mappings,
        )
        self.predict_set_name = set_name

    def train_dataloader(self):
        return self._data_modules[0].get_dataloader(self.train_set, shuffle=True, persistent_workers=True)

    def val_dataloader(self):
        return self._data_modules[0].get_dataloader(self.val_set)

    def test_dataloader(self):
        return self._data_modules[0].get_dataloader(self.test_set)

    def predict_dataloader(self):
        return self._data_modules[0].get_dataloader(self.predict_set)

    def save_predictions(self, *args, **kwargs) -> None:
        # assumption: data was converted to the same format as the first data module
        predictions_output_dir = self._data_modules[0].save_predictions(
            *args, outputs_dm=self.__class__.__name__, **kwargs)

        # but update counts in dparams file
        dparams_path = os.path.join(predictions_output_dir, 'dparams.yaml')

        with open(dparams_path, 'r') as f:
            settings = yaml.load(f, Loader=Loader)

        if 'class_counts' in settings:
            settings['class_counts'] = self.class_counts
        if 'train_set_size' in settings and self.train_set is not None:
            settings['train_set_size'] = self.train_set.cumulative_sizes[-1]
        if 'val_set_size' in settings and self.val_set is not None:
            settings['val_set_size'] = self.val_set.cumulative_sizes[-1]
        if 'test_set_size' in settings and self.test_set is not None:
            settings['test_set_size'] = self.test_set.cumulative_sizes[-1]

        settings['original_datamodule'] = self.__class__.__name__

        with open(dparams_path, 'w') as f:
            yaml.dump(settings, f, Dumper=Dumper)
