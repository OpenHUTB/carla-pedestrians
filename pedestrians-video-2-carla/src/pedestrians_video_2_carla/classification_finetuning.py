
import argparse
import copy
import logging
import os
import shutil
import sys
from typing import List
from pedestrians_video_2_carla.data.base.base_transforms import BaseTransforms

from pedestrians_video_2_carla.loggers.pedestrian.enums import PedestrianRenderers
from pedestrians_video_2_carla.modeling import discover_available_classes, main as modeling_main, setup_flow
from pedestrians_video_2_carla.modules.flow.output_types import MovementsModelOutputType
from pedestrians_video_2_carla.utils.paths import get_run_id_from_checkpoint_path, resolve_ckpt_path
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON

import randomname


def setup_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Classification flow with finetuning and denoising AE.",
    )
    parser.add_argument(
        '--keep_predictions',
        action='store_true',
        help='Keep the prediction datasets. By default they are deleted at the end of the script.',
    )
    parser.add_argument(
        '--force_train',
        action='store_true',
        help='Force training mode for classifier instead of tuning. Useful when resuming tuning from a checkpoint.',
    )
    parser.add_argument(
        '--ae_ckpt_path',
        type=str,
        help='Path to AE',
        default='wandb://carla-pedestrians/autoencoder/model-bright-node:v0',
    )

    return parser


def main(args: List[str]):
    # generate required version names before seeding
    experiment_tag = randomname.get_name()
    ae_predict_version = randomname.get_name()
    classifier_version = randomname.get_name()

    parser = setup_args()
    discover_available_classes()
    flow_args, _ = setup_flow(args, parser)

    # do a sanity check on the flow arguments
    if flow_args.ckpt_path is None:
        raise ValueError(
            "You must provide a classifier checkpoint path for fine-tuning!")

    # Gather JAAD predictions from the AE
    ae_ckpt_path = flow_args.ae_ckpt_path
    ae_pred_args = argparse.Namespace(**{
        'flow': 'autoencoder',
        'mode': 'predict',
        'predict_sets': ['train', 'val', 'test'],
        'renderers': [PedestrianRenderers.none],
        'overfit_batches': False,
        'skip_metadata': False,
        #
        'root_dir': flow_args.root_dir,
        'logs_dir': flow_args.logs_dir,
        'prefer_tensorboard': flow_args.prefer_tensorboard,
        'batch_size': flow_args.batch_size,
        'seed': flow_args.seed,
        'num_workers': flow_args.num_workers,
        #
        'data_module_name': flow_args.data_module_name,
        'clip_length': flow_args.clip_length,
        'clip_offset': flow_args.clip_offset,
        'label_frames': flow_args.label_frames,
        'strong_points': flow_args.strong_points,
        'train_proportions': getattr(flow_args, 'train_proportions', None),
        'val_proportions': getattr(flow_args, 'val_proportions', None),
        'test_proportions': getattr(flow_args, 'test_proportions', None),
        #
        'fast_dev_run': flow_args.fast_dev_run,

        # AE args
        # TODO: get this from cmd line instead of hardcoding - implement args prefixing
        'ckpt_path': ae_ckpt_path,
        'hidden_size': 191,
        'num_layers': 2,
        # fixed, since the important thing is what AE was trained with
        'transform': BaseTransforms.hips_neck_bbox,
        'movements_model_name': 'LSTM',
        'movements_output_type': MovementsModelOutputType.pose_2d,
        'input_nodes': CARLA_SKELETON,
        'output_nodes': CARLA_SKELETON,
    })

    (_, gt_data_subsets_dir, ae_trainer) = modeling_main(
        ae_pred_args,
        version=ae_predict_version,
        standalone=False,
        return_trainer=True,
        tags=[experiment_tag, 'ae_predict'],
    )

    ae_run_id = get_run_id_from_checkpoint_path(resolve_ckpt_path(ae_ckpt_path))

    try:
        import wandb
        wandb.finish()
    except ImportError:
        pass

    ae_output_nodes = ae_trainer.model.movements_model.output_nodes
    next_data_module_name = flow_args.data_module_name

    if not isinstance(gt_data_subsets_dir, str):
        gt_data_subsets_dir = gt_data_subsets_dir[0].replace(
            ae_trainer.datamodule._data_modules[0].__class__.__name__,
            ae_trainer.datamodule.__class__.__name__
        )
        next_data_module_name = ae_trainer.datamodule._data_modules[0].__class__.__name__.replace(
            'DataModule', '')

    del ae_trainer

    orig_ae_data_subsets_dir = os.path.join(gt_data_subsets_dir.replace(
        'DataModule', 'DataModulePredictions'), ae_run_id)
    ae_data_subsets_dir = os.path.join(gt_data_subsets_dir.replace(
        'DataModule', 'DataModulePredictions'), ae_predict_version)

    # rename the directory from ae checkpoint to ae predictions run id to avoid confusion/clash when multiple runs
    # are done with the same AE checkpoint
    os.rename(orig_ae_data_subsets_dir, ae_data_subsets_dir)

    logging.getLogger(__name__).info(
        f'Data after autoencoder saved in {ae_data_subsets_dir}')

    # Load the classifier from checkpoint and fine-tune it

    classifier_train_args = copy.deepcopy(flow_args)
    # override some key args
    classifier_train_args.flow = 'classification'
    classifier_train_args.mode = 'train' if flow_args.force_train else 'tune'
    classifier_train_args.data_module_name = next_data_module_name
    classifier_train_args.subsets_dir = ae_data_subsets_dir
    classifier_train_args.renderers = [PedestrianRenderers.none]

    # forcefully disable artificial noise - it was already applied to the data
    classifier_train_args.missing_joint_probabilities = [0]
    classifier_train_args.noise = 'zero'
    classifier_train_args.noise_param = 0

    # data_nodes format is determined by AE output
    classifier_train_args.data_nodes = ae_output_nodes
    classifier_train_args.input_nodes = ae_output_nodes

    # if recurrent GNNs are used, batch_size should be 1; TODO: make this smarter instead of listing all possible models here
    if flow_args.classification_model_name in ['GConvLSTM', 'DCRNN', 'TGCN', 'GConvGRU']:
        classifier_train_args.log_every_n_steps = classifier_train_args.batch_size * \
            classifier_train_args.log_every_n_steps
        classifier_train_args.batch_size = 1

    modeling_main(
        classifier_train_args,
        version=classifier_version,
        standalone=True,
        return_trainer=False,
        tags=[experiment_tag, 'fine_tune'],
    )

    if not flow_args.keep_predictions:
        shutil.rmtree(ae_data_subsets_dir, ignore_errors=True)


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
