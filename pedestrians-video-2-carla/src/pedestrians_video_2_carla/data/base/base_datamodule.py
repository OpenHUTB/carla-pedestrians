import copy
import hashlib
import os
import shutil
from typing import Any, Callable, Dict, Iterable, List, Literal, Optional, Tuple, Type, Union

import h5py
import numpy as np
from pedestrians_video_2_carla.data.base.mixins.datamodule.classification_datamodule_mixin import ClassificationDataModuleMixin
import torch.multiprocessing
import yaml
from pedestrians_video_2_carla.data import (DATASETS_BASE, DEFAULT_ROOT,
                                            OUTPUTS_BASE, SUBSETS_BASE)
from pedestrians_video_2_carla.data.base.mixins.dataset.projection_2d_mixin import \
    Projection2DMixin
from pedestrians_video_2_carla.data.base.skeleton import (
    Skeleton, get_skeleton_type_by_name)
from pedestrians_video_2_carla.transforms.pose.normalization.bbox_extractor import BBoxExtractor
from pedestrians_video_2_carla.transforms.pose.normalization.hips_neck_extractor import HipsNeckExtractor
from pedestrians_video_2_carla.transforms.pose.normalization.hips_neck_bbox_fallback_extractor import \
    HipsNeckBBoxFallbackExtractor
from pedestrians_video_2_carla.transforms.pose.normalization import Normalizer, DeNormalizer
from pytorch_lightning import LightningDataModule
from torch.utils.data import DataLoader

from pedestrians_video_2_carla.utils.term import TERM_COLORS, TERM_CONTROLS

try:
    from torch_geometric.loader import DataLoader as GraphDataLoader
except ImportError:
    GraphDataLoader = None

from tqdm.auto import tqdm
import itertools

from .base_transforms import BaseTransforms

try:
    from yaml import CDumper as Dumper, CLoader as Loader
except ImportError:
    from yaml import Dumper, Loader


sharing_strategy = "file_system"
torch.multiprocessing.set_sharing_strategy(sharing_strategy)


def set_worker_sharing_strategy(worker_id: int) -> None:
    torch.multiprocessing.set_sharing_strategy(sharing_strategy)


class BaseDataModule(LightningDataModule):
    def __init__(self,
                 data_nodes: Type[Skeleton],
                 input_nodes: Type[Skeleton] = None,
                 root_dir: Optional[str] = DEFAULT_ROOT,
                 clip_length: Optional[int] = 30,
                 batch_size: Optional[int] = 64,
                 num_workers: Optional[int] = os.cpu_count() // 4,
                 transform: Optional[Union[BaseTransforms, Callable]
                                     ] = BaseTransforms.hips_neck_bbox,
                 return_graph: bool = False,
                 val_set_frac: Optional[float] = 0.2,
                 test_set_frac: Optional[float] = 0.2,
                 predict_sets: List[str] = None,
                 subsets_dir: Optional[str] = None,
                 source_videos_dir: Optional[str] = None,
                 outputs_dir: Optional[str] = None,
                 min_video_length: Optional[int] = None,
                 **kwargs):
        super().__init__()

        self.outputs_dir = outputs_dir if outputs_dir is not None else os.path.join(
            root_dir, OUTPUTS_BASE, self.__class__.__name__)
        self.datasets_dir = os.path.join(root_dir, DATASETS_BASE)

        self.source_videos_dir = source_videos_dir
        if self.source_videos_dir is not None and not os.path.isabs(self.source_videos_dir):
            self.source_videos_dir = os.path.join(
                self.datasets_dir, self.source_videos_dir)

        self.clip_length = clip_length
        self.min_video_length = min_video_length if min_video_length is not None else clip_length
        self.batch_size = batch_size
        self.num_workers = num_workers
        self.data_nodes = data_nodes
        self.input_nodes = input_nodes if input_nodes is not None else data_nodes
        self.return_graph = return_graph
        self.val_set_frac = val_set_frac
        self.test_set_frac = test_set_frac
        self.kwargs = kwargs

        # used for debugging; will save additional setting value
        # to prevent mixing up of 'debugging' data with real data
        # this is NOT saved automatically to settings, it should be saved in subclasses if needed
        self._fast_dev_run = kwargs.get('fast_dev_run', False) != False

        if self.uses_clip_offset():
            self.clip_offset = kwargs.get('clip_offset', None)
            if self.clip_offset is None:
                self.clip_offset = self.clip_length
            assert self.clip_offset > 0, 'clip_offset must be greater than 0'
        else:
            self.clip_offset = None

        if self.return_graph:
            assert self.clip_length == 1 or self.batch_size == 1, 'Either clip_length or batch_size must be 1 for GNNs.'

        self.train_set = None
        self.val_set = None
        self.test_set = None
        self._set_size = {}

        self._predict_set_names = predict_sets if predict_sets is not None else []
        self._predict_sets = {}
        self.predict_set_name = None
        self.predict_set = None

        self.transform, self.transform_callable = self._setup_data_transform(transform)

        self._settings_digest = self._calculate_settings_digest()
        if subsets_dir is None:
            self._subsets_dir = os.path.join(
                self.outputs_dir, SUBSETS_BASE, self._settings_digest)
        else:
            self._subsets_dir = subsets_dir

        print(f'{TERM_CONTROLS.BOLD}Subsets dir: {TERM_COLORS.CYAN}{self._subsets_dir}{TERM_CONTROLS.ENDC} ')

        # each dataset may provide its own classification labels
        # they are saved in a settings file during the subsets creation
        # TODO: move this to ClassificationDataModuleMixin
        self._class_labels = None
        self._class_counts = {
            'train': {},
            'val': {},
            'test': {},
        }

        self._needs_preparation = False
        # only try to generate subsets if dir doesn't exist or is empty
        if (not os.path.exists(self._subsets_dir) or len(os.listdir(self._subsets_dir)) == 0):
            self._needs_preparation = True
            os.makedirs(self._subsets_dir, exist_ok=True)
        else:
            # we already have datasset prepared for this combination of settings
            # so only retrieve and store classification labels info/counts
            self._load_set_info()

        self.save_hyperparameters({
            **self.settings,
            **self.additional_hparams
        })

    @property
    def settings(self):
        return {
            'data_module_name': self.__class__.__name__,
            'clip_length': self.clip_length,
            'clip_offset': self.clip_offset,
            'data_nodes': self.data_nodes.__name__,
        }

    @property
    def additional_hparams(self):
        return {
            'batch_size': self.batch_size,
            'num_workers': self.num_workers,
            'transform': self.transform,
            'settings_digest': self._settings_digest,
            'subsets_dir': self._subsets_dir,
            **(Projection2DMixin.extract_hparams(self.kwargs) if self.uses_projection_mixin() else {})
        }

    @property
    def subsets_dir(self) -> str:
        return self._subsets_dir

    def _calculate_settings_digest(self):
        # for digest, we always want to have alphabetic order of keys
        settings = {k: self.settings[k] for k in sorted(self.settings.keys())}

        return hashlib.md5('-'.join(['{}={}'.format(k, str(s))
                                     for k, s in settings.items()]).encode()).hexdigest()

    def save_settings(self):
        """
        Saves the subset settings to file.
        """
        with open(os.path.join(self._subsets_dir, 'dparams.yaml'), 'w') as f:
            settings = copy.deepcopy(self.settings)

            settings.update(**{f'{k}_set_size': v for k, v in self._set_size.items()})

            if self.class_labels is not None:
                settings['class_labels'] = self.class_labels
            if self._class_counts is not None:
                settings['class_counts'] = self._class_counts

            yaml.dump(settings, f, Dumper=Dumper)

    def _setup_data_transform(self, transform: Union[BaseTransforms, Callable]):
        return (transform, {
            BaseTransforms.none: None,
            BaseTransforms.hips_neck: Normalizer(HipsNeckExtractor(self.data_nodes)),
            BaseTransforms.bbox: Normalizer(BBoxExtractor(self.data_nodes)),
            BaseTransforms.hips_neck_bbox: Normalizer(HipsNeckBBoxFallbackExtractor(self.data_nodes)),
            BaseTransforms.user_defined: transform
        }[transform]) if isinstance(transform, BaseTransforms) else (BaseTransforms.user_defined, transform)

    @classmethod
    def uses_clip_offset(cls):
        return True

    @classmethod
    def uses_projection_mixin(cls):
        return False

    @classmethod
    def uses_classification_mixin(cls):
        return False

    @classmethod
    def uses_infinite_train_set(cls):
        return False

    @classmethod
    def add_data_specific_args(cls, parent_parser, add_projection_2d_args=False, add_classification_args=False):
        parser = parent_parser.add_argument_group('Base DataModule')
        parser.add_argument(
            '--data_nodes',
            type=get_skeleton_type_by_name,
            default=None,
            help='Skeleton type to use for data nodes. If not specified, the default skeleton type for the dataset will be used.'
        )
        parser.add_argument(
            "--clip_length",
            metavar='NUM_FRAMES',
            type=int,
            default=30,
            help="Length of the clips."
        )
        if cls.uses_clip_offset():
            parser.add_argument(
                '--clip_offset',
                metavar='NUM_FRAMES',
                help='''
                    Number of frames to shift from the BEGINNING of the last clip.
                    Example: clip_length=30 and clip_offset=10 means that there will be
                    20 frames overlap between subsequent clips.
                    ''',
                type=int,
                default=None
            )
        parser.add_argument(
            "--batch_size",
            type=int,
            default=64,
            help="Batch size."
        )
        parser.add_argument(
            "--num_workers",
            type=int,
            default=os.cpu_count() // 4,
            help="Number of workers for the data loader."
        )
        parser.add_argument(
            "--transform",
            dest="transform",
            help="""
                Use one of predefined transforms: {}.
                Defaults to hips_neck.
                """.format(
                set(BaseTransforms.__members__.keys())),
            metavar="TRANSFORM",
            default=BaseTransforms.hips_neck_bbox,
            choices=list(set(BaseTransforms) - {BaseTransforms.user_defined}),
            type=BaseTransforms.__getitem__
        )
        parser.add_argument(
            '--skip_metadata',
            help="If True, metadata will not be loaded from the dataset.",
            default=False,
            action='store_true'
        )
        if cls.uses_projection_mixin() or add_projection_2d_args:
            Projection2DMixin.add_cli_args(parser)
        if cls.uses_classification_mixin() or add_classification_args:
            ClassificationDataModuleMixin.add_cli_args(parser)

        parent_parser = cls.add_subclass_specific_args(parent_parser)

        # for prediction only
        parser.add_argument(
            "--predict_sets",
            dest="predict_sets",
            help="Which sets ('train', 'val', 'test') to use in 'predict' mode. Has no effect otherwise.",
            default=[],
            nargs="+",
            type=str
        )

        # overrides subsets generation
        parser.add_argument(
            "--subsets_dir",
            dest="subsets_dir",
            help="Directory to use for subsets. If specified, will use subsets as-is.",
            default=None,
            type=str
        )

        # force override outputs dir path (useful when raw data is in some kind of "slow" persistent storage,
        # but runtime data needs to be elsewhere
        parser.add_argument(
            "--outputs_dir",
            dest="outputs_dir",
            help="Force different output dir for subsets",
            default=None,
            type=str
        )

        # input nodes are handled in the model hyperparameters
        return parent_parser

    @classmethod
    def add_subclass_specific_args(cls, parent_parser):
        """
        This method is used to add data-specific arguments to the parser in the subclasses.
        Please use this method instead of overriding add_data_specific_args, especially if you
        want to potentially use MixedDataModule.
        """
        return parent_parser

    def get_dataloader(self, dataset, shuffle=False, persistent_workers=False):
        if dataset is None:  # needed for overfit_batches
            return None

        pin_memory = self.kwargs.get('gpus', None) is not None
        persistent_workers = persistent_workers if self.num_workers > 1 else False
        if self.return_graph:
            # for GNNs we need to use the torch_geometric DataLoader
            return GraphDataLoader(
                dataset=dataset,
                batch_size=self.batch_size,
                num_workers=self.num_workers,
                pin_memory=pin_memory,
                persistent_workers=persistent_workers,
                shuffle=shuffle,
                worker_init_fn=set_worker_sharing_strategy
            )
        return DataLoader(
            dataset=dataset,
            batch_size=self.batch_size,
            num_workers=self.num_workers,
            pin_memory=pin_memory,
            persistent_workers=persistent_workers,
            shuffle=shuffle,
            worker_init_fn=set_worker_sharing_strategy
        )

    def train_dataloader(self):
        return self.get_dataloader(self.train_set, shuffle=True, persistent_workers=True)

    def predict_dataloader(self):
        return self.get_dataloader(self.predict_set)

    def val_dataloader(self):
        return self.get_dataloader(self.val_set)

    def test_dataloader(self):
        return self.get_dataloader(self.test_set)

    def _read_data(self) -> Any:
        raise NotImplementedError()

    def _clean_filter_sort_data(self, data: Any) -> Any:
        return data

    def _get_frame_counts(self, data: Any) -> Any:
        return None

    def _extract_clips(self, data: Any) -> Iterable[Any]:
        raise NotImplementedError()

    def _extract_additional_data(self, clips: Iterable[Any]) -> Iterable[Any]:
        return clips

    def _clean_filter_sort_clips(self, clips: Iterable[Any]) -> Iterable[Any]:
        """
        Handles the filtering and sorting of the clips. This is also where the
        self._class_labels & self._class_counts dicts should be set if dataset provides them.
        """
        return clips

    def _split_and_save_clips(self, clips: Iterable[Any]) -> Dict[str, int]:
        """
        This method is used to split the clips into train, val and test sets.
        It should return a dictionary containing the number of clips in each set.

        The implementation should use self.val_set_frac and self.test_set_frac
        to get the desired split proportions. The _save_subset method is provided
        to facilitate actual saving of the clips to HDF5 files.

        :param clips: List (or other iterable) of clips to split.
        :type clips: Iterable[Any]
        :return: A dictionary containing the number of clips in each set.
        :rtype: Dict[str, int]
        """
        raise NotImplementedError()

    def _load_set_info(self):
        with open(os.path.join(self._subsets_dir, 'dparams.yaml'), 'r') as f:
            settings = yaml.load(f, Loader=Loader)
            if 'class_labels' in settings:
                self._class_labels = settings['class_labels']
            if 'class_counts' in settings:
                self._class_counts = settings['class_counts']
            if 'train_set_size' in settings:
                self._set_size['train'] = settings['train_set_size']
            if 'val_set_size' in settings:
                self._set_size['val'] = settings['val_set_size']
            if 'test_set_size' in settings:
                self._set_size['test'] = settings['test_set_size']

    @property
    def class_labels(self) -> Dict[str, List[str]]:
        return self._class_labels

    @property
    def class_counts(self) -> Dict[Literal['train', 'val', 'test'], Dict[str, Dict[str, int]]]:
        return self._class_counts

    def prepare_data(self) -> None:
        # this is only called on one GPU, do not use self.something assignments

        if not self._needs_preparation:
            return

        # initial preparation
        progress_bar = tqdm(
            total=6, desc=f'Generating {self.__class__.__name__} subsets')
        loaded_data = self._read_data()
        progress_bar.update(1)
        filtered_data = self._clean_filter_sort_data(loaded_data)
        progress_bar.update(1)

        # gather clips
        clips = self._extract_clips(filtered_data)
        progress_bar.update(1)
        updated_clips = self._extract_additional_data(clips)
        progress_bar.update(1)

        # post-processing
        filtered_clips = self._clean_filter_sort_clips(updated_clips)
        progress_bar.update(1)

        # save data
        self._set_size = self._split_and_save_clips(filtered_clips)
        progress_bar.update(1)

        # save subset settings
        self.save_settings()
        progress_bar.close()

        # in case the method is called second time
        self._needs_preparation = False

    def _save_subset(self, name, projection_2d, targets, meta, save_dir=None):
        """
        This method is used to save the subset of data as HDF5 file that follows common format.
        """
        if save_dir is None:
            save_dir = self._subsets_dir
        with h5py.File(os.path.join(save_dir, "{}.hdf5".format(name)), "w") as f:
            f.create_dataset("projection_2d", data=projection_2d,
                             chunks=(1, *projection_2d.shape[1:]))

            for k, v in targets.items():
                f.create_dataset(f"targets/{k}", data=v,
                                 chunks=(1, *v.shape[1:]))

            for k, v in meta.items():
                if isinstance(v, np.ndarray) and v.dtype.kind != 'U':
                    f.create_dataset(f"meta/{k}", data=v,
                                     chunks=(1, *v.shape[1:]))
                else:
                    unique = list(set(v))
                    encoded_unique = [
                        str(s).encode("latin-1") for s in unique
                    ]
                    max_label_length = max(len(s) for s in encoded_unique)
                    labels = np.array(encoded_unique, dtype=h5py.string_dtype(
                        'ascii', max_label_length))
                    if labels.nbytes < 64 * 1024:
                        # if labels are less than 64Kb, we can save them as attributes
                        mapping = {s: i for i, s in enumerate(unique)}
                        mapped_v = np.array([mapping[s] for s in v], dtype=np.uint16)
                        f.create_dataset("meta/{}".format(k), data=mapped_v)
                        f["meta/{}".format(k)].attrs["labels"] = labels
                    else:
                        # otherwise we need to save the values as dataset
                        encoded_v = [str(s).encode("latin-1") for s in v]
                        max_v_length = max(len(s) for s in encoded_v)
                        np_v = np.array(encoded_v, dtype=h5py.string_dtype(
                            'ascii', max_v_length))
                        f.create_dataset("meta/{}".format(k), data=np_v)

        return len(projection_2d)

    def _get_dataset_creator(self) -> Callable:
        raise NotImplementedError()

    def setup(self, stage: Optional[str] = None) -> None:
        """
        This method is used to setup the data module.
        """
        set_ext = 'hdf5'
        dataset_creator = self._get_dataset_creator()
        common_kwargs = {
            'data_nodes': self.data_nodes,
            'input_nodes': self.input_nodes,
            'transform': self.transform_callable,
            'return_graph': self.return_graph,
            'clip_length': self.clip_length,
            'class_labels': self.class_labels,
            'source_videos_dir': self.source_videos_dir,
            **self.kwargs
        }

        if stage == "fit" or stage is None:
            self.train_set = dataset_creator(
                set_filepath=os.path.join(self._subsets_dir, f'train.{set_ext}'),
                is_training=True,
                **common_kwargs,
            )
            self.val_set = dataset_creator(
                set_filepath=os.path.join(self._subsets_dir, f'val.{set_ext}'),
                **common_kwargs,
            )

        if stage == "test" or stage is None:
            self.test_set = dataset_creator(
                set_filepath=os.path.join(self._subsets_dir, f'test.{set_ext}'),
                **common_kwargs,
            )

        if stage == "predict":
            self._predict_sets = {
                name: dataset_creator(
                    set_filepath=os.path.join(self._subsets_dir, f'{name}.{set_ext}'),
                    **common_kwargs,
                )
                for name in self._predict_set_names
            }

    def choose_predict_set(self, set_name: str) -> None:
        self.predict_set = self._predict_sets[set_name]
        self.predict_set_name = set_name

    def save_predictions(self, run_id, outputs: Iterable[Tuple[Dict, Dict]], crucial_keys: List[str], outputs_key: str, outputs_dm: str = None) -> str:
        """
        Saves predictions from the model so that they can be used as input (dataset) for the next model.
        """

        if outputs_dm is None:
            base_outputs_dir = f"{self.outputs_dir}Predictions"
        else:
            base_outputs_dir = os.path.realpath(os.path.join(
                self.outputs_dir, "..", f"{outputs_dm}Predictions"))

        predictions_output_dir = os.path.join(
            base_outputs_dir, SUBSETS_BASE, self._settings_digest, run_id)

        if not os.path.exists(predictions_output_dir):
            os.makedirs(predictions_output_dir)
        shutil.copy(os.path.join(self._subsets_dir, "dparams.yaml"),
                    predictions_output_dir)

        print(f"Saving {self.predict_set_name} predictions to {predictions_output_dir}.")

        # what to save?
        meta_keys = list(outputs[0][1].keys())
        targets_keys = set(outputs[0][0]['targets'].keys()).union(set(crucial_keys))
        targets_keys = list(
            filter(lambda k: not k.startswith('projection_2d_'), targets_keys))

        # extract relevant data
        projections_2d = []
        targets = {
            k: [] for k in targets_keys
        }
        meta = {
            k: [] for k in meta_keys
        }
        for sliced_data, batch_meta in outputs:
            if outputs_key == "projections_2d_transformed":
                projection_2d_transformed = sliced_data[outputs_key]
                output_denormalizer = DeNormalizer()
                denormalized_projections = output_denormalizer(
                    projection_2d_transformed[..., :2],
                    sliced_data['targets']['projection_2d_scale'],
                    sliced_data['targets']['projection_2d_shift']
                ).cpu().numpy()
                projections_2d.append(denormalized_projections)
            else:
                projections_2d.append(sliced_data[outputs_key].cpu().numpy())

            for k in targets_keys:
                if k in sliced_data:
                    targets[k].append(sliced_data[k].cpu().numpy())
                else:
                    targets[k].append(sliced_data['targets'][k].cpu().numpy())

            for k in meta_keys:
                meta[k].append(batch_meta[k])

        projections_2d = np.concatenate(projections_2d, axis=0)
        for k, v in targets.items():
            targets[k] = np.concatenate(v, axis=0)
        for k, v in meta.items():
            if isinstance(v[0], list):
                meta[k] = list(itertools.chain(*v))
            else:
                meta[k] = np.concatenate(v, axis=0)

        # save predictions
        self._save_subset(self.predict_set_name, projections_2d,
                          targets, meta, save_dir=predictions_output_dir)

        return predictions_output_dir
