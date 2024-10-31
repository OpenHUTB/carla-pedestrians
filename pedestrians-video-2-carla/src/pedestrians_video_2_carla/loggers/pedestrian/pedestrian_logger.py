from email.policy import default
import logging
from typing import List, Union

import numpy as np
from pedestrians_video_2_carla.modules.flow.output_types import \
    MovementsModelOutputType
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.transforms.pose.normalization.hips_neck_extractor import HipsNeckExtractor
from pytorch_lightning.loggers import LightningLoggerBase
from pytorch_lightning.loggers.base import rank_zero_experiment
from pytorch_lightning.utilities import rank_zero_only, rank_zero_warn

from pedestrians_video_2_carla.transforms.pose.normalization import Extractor

from .disabled_pedestrian_writer import DisabledPedestrianWriter
from .enums import MergingMethod, PedestrianRenderers
from .pedestrian_writer import PedestrianWriter


class PedestrianLogger(LightningLoggerBase):
    """
    Logger for video output.
    """

    def __init__(self,
                 save_dir: str,
                 name: str,
                 version: Union[int, str] = 0,
                 log_every_n_steps: int = 50,
                 video_saving_frequency_reduction: int = 10,
                 renderers: List[PedestrianRenderers] = None,
                 movements_output_type: MovementsModelOutputType = MovementsModelOutputType.pose_changes,
                 **kwargs):
        """
        Initialize PedestrianLogger.

        :param save_dir: Directory to save videos. Usually you get this from TensorBoardLogger.log_dir + 'videos'.
        :type save_dir: str
        :param name: Name of the experiment.
        :type name: str
        :param version: Version of the experiment.
        :type version: Union[int, str]
        :param log_every_n_steps: Log every n steps. This should match the Trainer setting,
            the final value will be determined by multiplying it with video_saving_frequency_reduction. Default: 50.
        :type log_every_n_steps: int
        :param video_saving_frequency_reduction: Reduce the video saving frequency by this factor. Default: 10.
        :type video_saving_frequency_reduction: int
        :param renderers: List of used renderers. Default: [].
        :type renderers: List[PedestrianRenderers]
        """
        super().__init__(
            agg_key_funcs=kwargs.get('agg_key_funcs', None),
            agg_default_func=kwargs.get('agg_default_func', np.mean),
        )

        self._save_dir = save_dir
        self._name = name
        self._version = version
        self._kwargs = kwargs
        self._experiment = None
        self._writer_cls = PedestrianWriter

        self._video_saving_frequency_reduction = video_saving_frequency_reduction
        self._log_every_n_steps = log_every_n_steps
        self._reduced_log_every_n_steps = self._log_every_n_steps * \
            self._video_saving_frequency_reduction

        if self._reduced_log_every_n_steps <= 1:
            rank_zero_warn("Video logging interval set to 0. Disabling video output.")
            self._writer_cls = DisabledPedestrianWriter

        # If renderers were not specified, use 'none'.
        self._renderers = renderers if (
            renderers is not None) and (len(renderers) > 0) else []

        # See if we can use CARLA renderer. It can still fail later if CARLA server is not available.
        # Here we only test if actual package or mock is used
        if PedestrianRenderers.carla in self._renderers or PedestrianRenderers.source_carla in self._renderers:
            try:
                import carla
            except ImportError:
                rank_zero_warn(
                    "CARLA renderers not available. Disabling CARLA renderers.")
                if PedestrianRenderers.carla in self._renderers:
                    self._renderers.remove(PedestrianRenderers.carla)
                if PedestrianRenderers.source_carla in self._renderers:
                    self._renderers.remove(PedestrianRenderers.source_carla)

        if PedestrianRenderers.none in self._renderers:
            self._renderers.remove(PedestrianRenderers.none)

        if kwargs.get('source_videos_dir', None) is None:
            try:
                self._renderers.remove(PedestrianRenderers.source_videos)
                rank_zero_warn(
                    "No source videos dir was specified. Disabling source videos renderer.")
            except ValueError:
                pass

        if kwargs.get('output_nodes', None) is None:
            try:
                self._renderers.remove(PedestrianRenderers.projection_points)
                rank_zero_warn(
                    "No output_nodes was specified. Disabling projection_points renderer.")
            except ValueError:
                pass

        if len(self._renderers) == 0:
            rank_zero_warn("No renderers specified. Disabling video output.")
            self._writer_cls = DisabledPedestrianWriter

        self._movements_output_type = movements_output_type

    @staticmethod
    def add_logger_specific_args(parent_parser):
        parser = parent_parser.add_argument_group("Pedestrian Logger")
        parser.add_argument(
            "--video_saving_frequency_reduction",
            dest="video_saving_frequency_reduction",
            help="Set video saving frequency reduction to save them every REDUCTION * log_every_n_steps. If 0 or less disables saving videos.",
            metavar="REDUCTION",
            default=10,
            type=lambda x: max(int(x), 0)
        )
        parser.add_argument(
            "--max_videos",
            dest="max_videos",
            help="Set maximum number of videos to save from each batch. Set to -1 to save all videos in batch. Default: 10",
            default=10,
            type=int
        )
        parser.add_argument(
            "--renderers",
            dest="renderers",
            help="""
                Set renderers to use for video output.
                To disable rendering, 'none' renderer must be explicitly passed as the only renderer.
                Choices: {}.
                Default: ['input_points', 'projection_points']
                """.format(
                set(PedestrianRenderers.__members__.keys())),
            metavar="RENDERER",
            default=[],
            choices=list(PedestrianRenderers),
            nargs="+",
            action="extend",
            type=PedestrianRenderers.__getitem__
        )
        parser.add_argument(
            "--merging_method",
            dest="merging_method",
            help="""
                How to merge multiple videos into one.
                Choices: {}.
                Default: 'square'
                """.format(
                set(MergingMethod.__members__.keys())),
            metavar="METHOD",
            default=MergingMethod.square,
            choices=list(MergingMethod),
            type=MergingMethod.__getitem__
        )
        parser.add_argument(
            "--source_videos_dir",
            dest="source_videos_dir",
            help="Directory to read source videos from. Required if 'source_videos' renderer is used. Default: None",
            default=None,
        )
        parser.add_argument(
            "--source_videos_overlay_skeletons",
            dest="source_videos_overlay_skeletons",
            help="Draw 2D input skeleton on top of source video. Default: False",
            default=False,
            action='store_true'
        )
        parser.add_argument(
            "--source_videos_overlay_bboxes",
            dest="source_videos_overlay_bboxes",
            help="Draw 2D input bboxes on top of source video. Default: False",
            default=False,
            action='store_true'
        )
        parser.add_argument(
            "--source_videos_overlay_classes",
            dest="source_videos_overlay_classes",
            help="Write class labels on top of source video. Default: False",
            default=False,
            action='store_true'
        )

        return parent_parser

    @property
    def name(self) -> str:
        return self._name

    @property
    def version(self) -> int:
        return self._version

    @property
    @rank_zero_experiment
    def experiment(self):
        if self._experiment is None:
            self._experiment = self._writer_cls(
                log_dir=self._save_dir,
                renderers=self._renderers,
                reduced_log_every_n_steps=self._reduced_log_every_n_steps,
                movements_output_type=self._movements_output_type,
                **self._kwargs
            )

        return self._experiment

    @rank_zero_only
    def log_hyperparams(self, params):
        pass

    @rank_zero_only
    def log_metrics(self, metrics, step):
        pass
