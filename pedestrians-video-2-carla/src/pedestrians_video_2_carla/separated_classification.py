
import argparse
import copy
import glob
import logging
import os
import shutil
import sys
from typing import List
from pedestrians_video_2_carla.data.base.base_transforms import BaseTransforms

from pedestrians_video_2_carla.loggers.pedestrian.enums import PedestrianRenderers
from pedestrians_video_2_carla.modeling import discover_available_classes, main as modeling_main, setup_flow
from pedestrians_video_2_carla.modules.flow.output_types import MovementsModelOutputType
from pedestrians_video_2_carla.utils.paths import get_run_id_from_checkpoint_path, get_run_id_from_log_dir, resolve_ckpt_path
from pedestrians_video_2_carla.utils.printing import print_metrics
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.openpose.skeleton import BODY_25_SKELETON

import randomname


def setup_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Classification flow for data with & without denoising AE.",
    )
    parser.add_argument(
        '--keep_predictions',
        action='store_true',
        help='Keep the prediction datasets. By default they are deleted at the end of the script.',
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
    data_prep_version = randomname.get_name()
    ae_predict_version = randomname.get_name()
    classifier_version_a = randomname.get_name()
    classifier_version_b = randomname.get_name()
    classifier_version_c = randomname.get_name()

    parser = setup_args()
    discover_available_classes()
    flow_args, _ = setup_flow(args, parser)

    logging.getLogger(__name__).info(f"Starting data preparation flow")

    # store metrics to display at the end of the script
    metrics = {}

    common_predict_args = {
        'flow': 'autoencoder',
        'mode': 'predict',
        'predict_sets': ['train', 'val', 'test'],
        'renderers':[PedestrianRenderers.none],
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
        #
        'fast_dev_run': flow_args.fast_dev_run,
    }

    if flow_args.data_module_name == 'CarlaRecorded':
        common_predict_args['carla_rec_set_name'] = flow_args.carla_rec_set_name
    elif flow_args.data_module_name == 'JAADOpenPose':
        common_predict_args['strong_points'] = flow_args.strong_points

    noise_args = {
        k: v for k,v in vars(flow_args).items() if k.startswith('missing_joint_probabilities')
    }

    noise_args['noise'] = flow_args.noise
    noise_args['noise_param'] = flow_args.noise_param

    # Gather input data (add artificial noise)
    data_prep_args = argparse.Namespace(**{
        **common_predict_args,

        # fixed args
        'movements_output_type': MovementsModelOutputType.pose_2d,
        'movements_model_name': 'ZeroMovements',
        'ckpt_path': None,

        # experiment-dependent args
        **noise_args,
        'transform': BaseTransforms.none,  # do not deform data in any way,
    })

    if flow_args.data_module_name == 'CarlaRecorded':
        data_prep_args.input_nodes = CARLA_SKELETON
        data_prep_args.output_nodes = CARLA_SKELETON
    elif flow_args.data_module_name == 'JAADOpenPose':
        data_prep_args.input_nodes = BODY_25_SKELETON
        data_prep_args.output_nodes = BODY_25_SKELETON

    prep_log_dir, gt_data_subsets_dir = modeling_main(
        data_prep_args,
        version=data_prep_version,
        standalone=True,
        tags=[experiment_tag, 'data_prep'],
    )

    prep_run_id = get_run_id_from_log_dir(prep_log_dir)
    prep_data_subsets_dir = os.path.join(gt_data_subsets_dir.replace(
        'DataModule', 'DataModulePredictions'), prep_run_id)
    logging.getLogger(__name__).info(f'Prepared data saved in {prep_data_subsets_dir}')

    # Gather predictions from the AE
    ae_ckpt_path = flow_args.ae_ckpt_path
    ae_pred_args = argparse.Namespace(**{
        **common_predict_args,

        'subsets_dir': prep_data_subsets_dir,

        # AE args
        # TODO: get this from cmd line instead of hardcoding - implement args prefixing
        'ckpt_path': ae_ckpt_path,
        'hidden_size': 191,
        'num_layers': 2,
        'transform': BaseTransforms.hips_neck_bbox,  # fixed, since the important thing is what AE was trained with
        'movements_model_name': 'LSTM',
        'movements_output_type': MovementsModelOutputType.pose_2d,
        'input_nodes': CARLA_SKELETON,
        'output_nodes': CARLA_SKELETON,
    })

    (_, _, ae_trainer) = modeling_main(
        ae_pred_args,
        version=ae_predict_version,
        standalone=False,
        return_trainer=True,
        tags=[experiment_tag, 'ae_predict'],
    )

    ae_run_id = get_run_id_from_checkpoint_path(resolve_ckpt_path(ae_ckpt_path))
    ae_output_nodes = ae_trainer.model.movements_model.output_nodes
    del ae_trainer

    try:
        import wandb
        wandb.finish()
    except ImportError:
        pass

    orig_ae_data_subsets_dir = os.path.join(gt_data_subsets_dir.replace(
        'DataModule', 'DataModulePredictions'), ae_run_id)
    ae_data_subsets_dir = os.path.join(gt_data_subsets_dir.replace(
        'DataModule', 'DataModulePredictions'), ae_predict_version)

    # rename the directory from ae checkpoint to ae predictions run id to avoid confusion/clash when multiple runs
    # are done with the same AE checkpoint
    os.rename(orig_ae_data_subsets_dir, ae_data_subsets_dir)

    logging.getLogger(__name__).info(f'Data after autoencoder saved in {ae_data_subsets_dir}')

    # Train the same classifier three times: on clean data (for baseline), once on noisy data and once with denoising AE
    for version, subsets_dir, tag, after_run_params in [
        (classifier_version_a, gt_data_subsets_dir, 'clean', { 'ae': False }),
        (classifier_version_b, prep_data_subsets_dir, 'noisy', { **noise_args, 'ae': False }),
        (classifier_version_c, ae_data_subsets_dir, 'noisy_ae', { **noise_args, 'ae': True }),
    ]:
        logging.getLogger(__name__).info(f"Training classifier on {subsets_dir}.")

        # train from scratch; resuming classifier training is not supported
        classifier_train_args = copy.deepcopy(flow_args)
        # override some key args
        classifier_train_args.flow = 'classification'
        classifier_train_args.mode = 'train'
        classifier_train_args.subsets_dir = subsets_dir
        classifier_train_args.renderers = [PedestrianRenderers.none]

        # forcefully disable artificial noise - it was already applied to the data
        classifier_train_args.missing_joint_probabilities = [0]
        classifier_train_args.noise = 'zero'
        classifier_train_args.noise_param = 0
        for k in vars(flow_args):
            if k.startswith('missing_joint_probabilities'):
                delattr(classifier_train_args, k)

        # if tag is 'noisy_ae', then data_nodes format is determined by AE output
        if tag == 'noisy_ae':
            classifier_train_args.data_nodes = ae_output_nodes
            classifier_train_args.input_nodes = ae_output_nodes
        else:
            classifier_train_args.data_nodes = data_prep_args.input_nodes
            classifier_train_args.input_nodes = data_prep_args.input_nodes

        # if recurrent GNNs are used, batch_size should be 1; TODO: make this smarter instead of listing all possible models here
        if flow_args.classification_model_name in ['GConvLSTM', 'DCRNN', 'TGCN', 'GConvGRU', 'GCNBestPaper']:
            classifier_train_args.log_every_n_steps = classifier_train_args.batch_size * classifier_train_args.log_every_n_steps
            classifier_train_args.batch_size = 1

        (_, _, trainer) = modeling_main(
            classifier_train_args,
            version=version,
            standalone=False,
            return_trainer=True,
            tags=[experiment_tag, tag],
        )

        if len(after_run_params) > 0 and hasattr(trainer.logger[0].experiment.config, 'update'):
            # store the info about the artificial noise applied to the data
            trainer.logger[0].experiment.config.update(after_run_params, allow_val_change=True)

        try:
            import wandb
            wandb.finish()
        except ImportError:
            pass

        metrics[version] = {k: v for k,
                            v in trainer.logged_metrics.items() if k.startswith('hp')}

        del trainer

    for name, results in metrics.items():
        print_metrics(results, f'{name} metrics:')

    if not flow_args.keep_predictions:
        shutil.rmtree(ae_data_subsets_dir, ignore_errors=True)
        shutil.rmtree(prep_data_subsets_dir, ignore_errors=True)


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
