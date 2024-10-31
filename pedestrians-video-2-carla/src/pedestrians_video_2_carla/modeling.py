import argparse
import logging
import math
import os
import sys
from typing import Dict, List, Tuple, Type, Union
from pedestrians_video_2_carla.modules.flow.base_model import BaseModel

import pytorch_lightning as pl
from pytorch_lightning.callbacks import LearningRateMonitor, ModelCheckpoint, ModelSummary
from pytorch_lightning.loggers.tensorboard import TensorBoardLogger

from pedestrians_video_2_carla.modules.flow.autoencoder import \
    LitAutoencoderFlow
from pedestrians_video_2_carla.modules.flow.base import LitBaseFlow
from pedestrians_video_2_carla.modules.flow.classification import LitClassificationFlow
from pedestrians_video_2_carla.modules.flow.pose_lifting import \
    LitPoseLiftingFlow
from pedestrians_video_2_carla.modules.flow.pose_estimation import LitPoseEstimationFlow
from pedestrians_video_2_carla.utils.paths import get_run_id_from_checkpoint_path, resolve_ckpt_path
from pedestrians_video_2_carla.utils.term import TERM_COLORS, TERM_CONTROLS

try:
    import wandb
    from pytorch_lightning.loggers.wandb import WandbLogger
except ImportError:
    WandbLogger = None

import randomname
from pytorch_lightning.utilities.warnings import rank_zero_warn

from pedestrians_video_2_carla import __version__
from pedestrians_video_2_carla.data import discover as discover_datamodules
from pedestrians_video_2_carla.data.base.base_datamodule import BaseDataModule
from pedestrians_video_2_carla.loggers.pedestrian import PedestrianLogger

# global registry of available classes
data_modules: Dict[str, Type[BaseDataModule]] = {}
flow_modules: Dict[str, Type[LitBaseFlow]] = {}

# ---- CLI ----
# The functions defined in this section are wrappers around the main Python
# API allowing them to be called directly from the terminal as a CLI
# executable/script.


def add_program_args(parser: argparse.ArgumentParser):
    """
    Add program-level command line parameters. discover_available_classes needs to be called first.
    """
    global data_modules
    global flow_modules

    parser.add_argument(
        "--version",
        action="version",
        version="pedestrians-video-2-carla {ver}".format(ver=__version__),
    )
    parser.add_argument(
        "-v",
        "--verbose",
        dest="loglevel",
        help="set loglevel to INFO",
        action="store_const",
        const=logging.INFO,
    )
    parser.add_argument(
        "-vv",
        "--very_verbose",
        dest="loglevel",
        help="set loglevel to DEBUG",
        action="store_const",
        const=logging.DEBUG,
    )
    parser.add_argument(
        "-m",
        "--mode",
        dest="mode",
        help="set mode to train, test or predict",
        default="train",
        choices=["train", "test", "predict", "tune"],
    )
    parser.add_argument(
        "--flow",
        dest="flow",
        help="Flow to use",
        default="pose_lifting",
        choices=list(flow_modules.keys()),
        type=str,
    )
    parser.add_argument(
        "--data_module_name",
        dest="data_module_name",
        help="Data module class to use",
        default="CarlaRecorded",
        choices=list(data_modules.keys()),
        type=str,
    )
    parser.add_argument(
        "--root_dir",
        dest="root_dir",
        help="Root directory for the outputs/datasets/logs resolving.",
        default=os.environ.get("VIDEO2CARLA_ROOT_DIR", "/"),
        type=str,
    )
    parser.add_argument(
        "--logs_dir",
        dest="logs_dir",
        help="Directory for the logs. Must be abs path or relative to the root_dir.",
        default="runs",
        type=str,
    )
    parser.add_argument(
        "--prefer_tensorboard",
        dest="prefer_tensorboard",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--seed", dest="seed", type=int, default=22742,
        help="Seed for random number generators. 0 means random seed."
    )
    parser.add_argument(
        "--ckpt_path", dest="ckpt_path", type=str, default=None,
        help="Path of the checkpoint to load to resume training / testing / prediction"
    )
    return parser


def setup_logging(loglevel):
    """
    Setup basic logging

    :param loglevel: minimum loglevel for emitting messages
    :type loglevel: int
    """
    logformat = "[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
    logging.basicConfig(
        level=loglevel, stream=sys.stdout, format=logformat, datefmt="%Y-%m-%d %H:%M:%S"
    )

    matplotlib_logger = logging.getLogger("matplotlib")
    matplotlib_logger.setLevel(logging.INFO)


def main(
    args: Union[List[str], argparse.Namespace],
    version: str = None,
    return_trainer: bool = False,
    standalone: bool = True,
    tags: List[str] = None,
    project: str = None,
) -> Union[Tuple[str, str], Tuple[str, str, pl.Trainer]]:
    """
    :param args: command line parameters as list of strings
          (for example  ``["--verbose"]``).
    :type args: List[str]
    """
    # get random version name before seeding
    if version is None:
        version = randomname.get_name()

    discover_available_classes()

    if isinstance(args, argparse.Namespace):
        (data_module_cls, flow_module_cls, models_cls), _ = setup_classes(args)

        # seed everything as soon as we can if needed
        if args.seed:
            pl.seed_everything(args.seed, workers=True)
    else:
        parser = argparse.ArgumentParser(
            description="Map pedestrians movements from videos to CARLA"
        )

        args, (data_module_cls,
               flow_module_cls,
               models_cls) = setup_flow(args, parser)  # this also seeds everything

    dict_args = vars(args)

    # models
    models = {
        f"{model_type}_model": model_cls(**dict_args)
        for model_type, model_cls in models_cls.items()
    }

    # loggers - try to use WandbLogger or fallback to TensorBoardLogger
    # the primary logger log dir is used as default for all loggers & checkpoints
    if os.path.isabs(args.logs_dir):
        abs_logs_dir = args.logs_dir
    else:
        abs_logs_dir = os.path.join(args.root_dir, args.logs_dir)
    if (
        WandbLogger is not None
        and "PYTEST_CURRENT_TEST" not in os.environ
        and not args.prefer_tensorboard
    ):
        logger = WandbLogger(
            save_dir=abs_logs_dir,
            name=version,
            version=version,
            project=project if project is not None else args.flow,
            entity="carla-pedestrians",
            log_model=True,  # this will log models created by ModelCheckpoint,
            tags=tags,
        )
        log_dir = os.path.realpath(os.path.join(str(logger.experiment.dir), ".."))
    else:
        logger = TensorBoardLogger(
            save_dir=abs_logs_dir,
            name=os.path.join(
                args.flow,
                data_module_cls.__name__,
                *[
                    model.__class__.__name__
                    for model in models.values()
                ]
            ),
            version=version,
            default_hp_metric=False,
        )
        log_dir = logger.log_dir

    print(f"{TERM_CONTROLS.BOLD}Logging dir: {TERM_COLORS.MAGENTA}{log_dir}{TERM_CONTROLS.ENDC} ")

    # some models support this as a CLI option
    # so we only add it if it's not already set
    if "movements_model" in models:
        dict_args.setdefault("movements_output_type",
                             models["movements_model"].output_type)

    pedestrian_logger = PedestrianLogger(
        save_dir=os.path.join(log_dir, "videos"),
        name=logger.name,
        version=logger.version,
        **dict_args,
    )

    checkpoint_callback = ModelCheckpoint(
        dirpath=os.path.join(log_dir, "checkpoints"),
        monitor="val_loss/primary",
        mode="min",
        save_top_k=1,
    )
    lr_monitor = LearningRateMonitor(logging_interval="step")
    model_summary = ModelSummary(max_depth=2)

    # flow model
    if args.ckpt_path is not None:
        args.ckpt_path = resolve_ckpt_path(args.ckpt_path)

        if not os.path.isfile(args.ckpt_path):
            raise ValueError(f"Checkpoint {args.ckpt_path} does not exist")

        flow_module = flow_module_cls.load_from_checkpoint(
            checkpoint_path=args.ckpt_path,
            **models,
            **dict_args
        )
    else:
        flow_module = flow_module_cls(
            **models,
            **dict_args
        )

    # data
    dm = data_module_cls(**dict_args,
                         return_graph=flow_module.needs_graph,
                         needs_heatmaps=flow_module.needs_heatmaps,
                         needs_confidence=flow_module.needs_confidence,
                         )

    # training
    trainer = pl.Trainer.from_argparse_args(
        args,
        logger=[logger, pedestrian_logger],
        callbacks=[checkpoint_callback, lr_monitor, model_summary],
    )

    if args.mode == "train":
        trainer.fit(model=flow_module, datamodule=dm, ckpt_path=args.ckpt_path)
    elif args.mode == "tune":
        trainer.fit(model=flow_module, datamodule=dm)  # we ignore optimizer states etc.
    elif args.mode == "test":
        trainer.test(model=flow_module, datamodule=dm)
    elif args.mode == "predict":
        # we need to explicitly set the datamodule here
        dm.prepare_data()
        dm.setup(stage='predict')
        run_id = get_run_id_from_checkpoint_path(
            args.ckpt_path) if args.ckpt_path else version
        for set_name in args.predict_sets:
            dm.choose_predict_set(set_name)
            outputs = trainer.predict(
                model=flow_module,
                datamodule=dm,
                ckpt_path=args.ckpt_path,
                return_predictions=True
            )
            dm.save_predictions(
                run_id, outputs, flow_module.crucial_keys, flow_module.outputs_key)

    if standalone and isinstance(logger, WandbLogger):
        wandb.finish()

    if return_trainer:
        return log_dir, dm.subsets_dir, trainer

    return log_dir, dm.subsets_dir


def discover_available_classes() -> Tuple[Dict[str, Type[BaseDataModule]], Dict[str, Type[LitBaseFlow]]]:
    global data_modules
    global flow_modules

    data_modules = discover_datamodules()
    flow_modules = {
        'pose_lifting': LitPoseLiftingFlow,
        'autoencoder': LitAutoencoderFlow,
        'classification': LitClassificationFlow,
        'pose_estimation': LitPoseEstimationFlow,
    }

    return data_modules, flow_modules


def setup_flow(args, parser: argparse.ArgumentParser):
    """
    Sets up the flow params. Needs discover_available_classes to be called first.
    """

    parser = add_program_args(parser)
    tmp_args = args[:]
    try:
        tmp_args.remove("-h")
    except ValueError:
        pass
    try:
        tmp_args.remove("--help")
    except ValueError:
        pass
    program_args, _ = parser.parse_known_args(tmp_args)

    (data_module_cls,
     flow_module_cls,
     models_cls), parser = setup_classes(program_args, parser, tmp_args)

    # seed everything as soon as we can if needed
    if program_args.seed:
        pl.seed_everything(program_args.seed, workers=True)

    parser = pl.Trainer.add_argparse_args(parser)

    parser = flow_module_cls.add_model_specific_args(parser)
    parser = data_module_cls.add_data_specific_args(parser)

    for model_cls in models_cls.values():
        parser = model_cls.add_model_specific_args(parser)

    parser = PedestrianLogger.add_logger_specific_args(parser)

    args = parser.parse_args(args)
    setup_logging(args.loglevel)

    # prevent accidental infinite training
    if data_module_cls.uses_infinite_train_set():
        if args.limit_train_batches < 0:
            args.limit_train_batches = 1.0
        elif (
            isinstance(args.limit_train_batches, float)
            and args.limit_train_batches <= 1.0
        ):
            args.limit_train_batches = math.ceil(
                (4 * args.val_set_size) / args.batch_size
            )
            rank_zero_warn(
                f"""No limit on train batches was set or it was specified as a fraction (--limit_train_batches), this will result in infinite training (never-ending epoch 0).
    If you really want to do this, set --limit_train_batches=-1.0 and I will not bother you anymore.
    For now, I set it to `(4 * val_set_size) / batch_size = {args.limit_train_batches}` for you."""
            )

    return args, (data_module_cls, flow_module_cls, models_cls)


def setup_classes(program_args: argparse.Namespace, parser: argparse.ArgumentParser = None, args: List[str] = None) -> Tuple[BaseDataModule, LitBaseFlow, Dict[str, BaseModel]]:
    """
    Extracts the classes from the program args. discover_available_classes needs to be called first
    to get supported DataModule and FlowModule classes.
    """
    global flow_modules
    # TODO: should this be subcommand?
    flow_module_cls = flow_modules[program_args.flow]

    global data_modules
    data_module_cls = data_modules[program_args.data_module_name]

    available_models = flow_module_cls.get_available_models()
    default_models = flow_module_cls.get_default_models()
    selected_models: Dict[str, BaseModel] = {}

    # we either have 'hand-crafted' args or need to dynamically create & parse them
    if parser is not None:
        # dynamically create & parse args
        for model_type, model_choices in available_models.items():
            parser.add_argument(
                f"--{model_type}_model_name",
                dest=f"{model_type}_model_name",
                help=f"{model_type.capitalize()} model class to use",
                default=None,
                choices=list(model_choices.keys()),
                type=str,
            )

        program_args, _ = parser.parse_known_args(args)

    for model_type, model_choices in available_models.items():
        model_name = getattr(program_args, f'{model_type}_model_name', None)
        if model_name is not None and model_name in list(model_choices.keys()):
            selected_models[model_type] = model_choices[model_name]
        else:
            selected_models[model_type] = default_models[model_type]

    return (data_module_cls, flow_module_cls, selected_models), parser


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
