from typing import Dict, Iterable, Tuple
import numpy as np
import pandas

from pedestrians_video_2_carla.utils.argparse import boolean


class ClassificationDataModuleMixin:
    """
    Mixing to handle extracting & adding classification data to clip meta.
    Needs to be used in conjunction with the PandasDataModuleMixin
    or follow similar conventions.
    """

    def __init__(
        self,
        classification_targets_key: str = 'cross',
        num_classes: int = 2,
        label_frames: float = -1,
        label_mapping: Tuple = ('not-crossing', 'crossing', 'irrelevant'),
        balance_classes: bool = False,
        **kwargs
    ):
        self._classification_targets_key = classification_targets_key
        self._label_frames = label_frames
        self._label_mapping = label_mapping[:num_classes]
        self._num_classes = num_classes
        self._balance_classes = balance_classes

        super().__init__(**kwargs)

    @property
    def settings(self):
        return {
            **super().settings,
            'label_frames': self._label_frames,
            'num_classes': self._num_classes,
            'classification_targets_key': self._classification_targets_key,
            'balance_classes': self._balance_classes,
        }

    @classmethod
    def uses_classification_mixin(cls):
        return True

    @classmethod
    def add_cli_args(cls, parser):
        if not any([arg.dest == 'label_frames' for arg in parser._actions]):
            parser.add_argument(
                '--label_frames',
                type=float,
                default=-1,
                help='Fraction of last frames to search for "positive" labels. -1 means to check only the last frame.'
            )
        if not any([arg.dest == 'num_classes' for arg in parser._actions]):
            parser.add_argument(
                '--num_classes',
                default=2,
                type=int,
            )
        if not any([arg.dest == 'classification_targets_key' for arg in parser._actions]):
            parser.add_argument(
                '--classification_targets_key',
                type=str,
                default='cross',
            )
        if not any([arg.dest == 'balance_classes' for arg in parser._actions]):
            parser.add_argument(
                "--balance_classes",
                help="If True, will balance the classes in the train dataset.",
                default=False,
                type=boolean
            )
        return parser

    def _set_class_labels(self, df: pandas.DataFrame) -> None:
        """
        Sets classification labels for 'cross' column.

        :param df: DataFrame with labels
        :type df: pandas.DataFrame
        """
        self._class_labels = {
            # explicitly set crossing to be 1, so it potentially can be used in a binary classifier
            self._classification_targets_key: self._label_mapping,
        }

    def _set_class_counts(self, set_name: str, meta: Dict[str, Iterable]) -> None:
        """
        Helper function to set the class counts.
        Should set self._class_counts dict if required.
        By default it extracts one class per clip
        for each type of class in self.class_labels.keys().

        :param set_name: Name of the set
        :type set_name: str
        :param meta: Metadata
        :type meta: Dict[str, Iterable]
        """
        if self.class_labels is None:
            # not every dataset has class labels
            return

        for class_key, class_labels in self.class_labels.items():
            numeric_classes = np.array([class_labels.index(
                key) for key in meta[class_key]])
            counts = np.bincount(numeric_classes, minlength=self._num_classes)
            self._class_counts[set_name][class_key] = {
                label: counts[i] for i, label in enumerate(class_labels)
            }

    def _add_classification_to_meta(self, grouped, grouped_tail, meta):
        """
        Add cross data to meta. Needs to be called in the appropriate
        place, usually in `PandasDataModuleMixin._get_raw_data` method.
        """
        if self._classification_targets_key in grouped_tail.columns:
            if self._label_frames < 0:
                cross_values = grouped_tail.loc[:,
                                                self._classification_targets_key].to_numpy()
            else:
                cutoffs = np.ceil(grouped.size().to_numpy() *
                                  self._label_frames).astype(np.int) * -1
                cross_values = []
                for cutoff, (idx, rows) in zip(cutoffs, grouped):
                    cross_values.append(
                        np.any(
                            rows.loc[:, self._classification_targets_key].iloc[cutoff:].to_numpy())
                    )

            meta[self._classification_targets_key] = np.choose(
                cross_values,
                self.class_labels[self._classification_targets_key]
            ).tolist()

    def _save_subset(self, name, projection_2d, targets, meta, save_dir=None):
        # data is already shuffled, so we only need to limit the amount
        if name == 'train' and self._balance_classes:
            numeric_classes = np.array([self.class_labels[self._classification_targets_key].index(
                key) for key in meta[self._classification_targets_key]])
            class_counts = np.bincount(numeric_classes, minlength=self._num_classes)
            min_count = np.min(class_counts)
            samples_mask = np.zeros(len(projection_2d), dtype=bool)
            for ci in range(self._num_classes):
                class_indices: Tuple[np.ndarray] = (numeric_classes == ci).nonzero()
                limited_indices = tuple([ci[:min_count] for ci in class_indices])
                samples_mask[limited_indices] = True

            balanced_projection_2d = projection_2d[samples_mask]
            balanced_targets = {key: value[samples_mask]
                                for key, value in targets.items()}
            balanced_meta = {key: np.array(value)[samples_mask]
                             for key, value in meta.items()}

            set_size = super()._save_subset(name, balanced_projection_2d,
                                            balanced_targets, balanced_meta, save_dir)
            self._set_class_counts(name, balanced_meta)
        else:
            set_size = super()._save_subset(name, projection_2d, targets, meta, save_dir)
            self._set_class_counts(name, meta)

        return set_size
