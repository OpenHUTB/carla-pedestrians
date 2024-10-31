
import argparse
import copy
import sys
from typing import List

import numpy as np
import randomname
import torch
import torch.nn.functional as F
import wandb
from tqdm.auto import tqdm

from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.loggers.pedestrian.enums import \
    PedestrianRenderers
from pedestrians_video_2_carla.modeling import discover_available_classes, setup_flow, main as modeling_main
from pedestrians_video_2_carla.utils.printing import print_metrics


def setup_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measuring sensitivity of the classification model to missing joints.",
    )

    return parser


def main(args: List[str]):
    num_joints = len(CARLA_SKELETON)

    # generate required version names before seeding
    versions = [
        randomname.get_name()
        for _ in range(num_joints+1)
    ]
    experiment_tag = versions[0]
    summary_version = randomname.get_name()

    parser = setup_args()
    discover_available_classes()
    flow_args, _ = setup_flow(args, parser)

    # store metrics to display at the end of the script
    metrics = {}

    # Train the same classifier 27 times: on clean data (for baseline), and once for every missing joint
    for idx, version in tqdm(enumerate(versions), desc="Training classifier"):
        # train from scratch; resuming classifier training is not supported
        classifier_train_args = copy.deepcopy(flow_args)
        # override some key args - just in case
        classifier_train_args.flow = 'classification'
        classifier_train_args.mode = 'train'
        classifier_train_args.renderers = [PedestrianRenderers.none]

        # forcefully set the missing joint params
        classifier_train_args.missing_joint_probabilities = [0.0] * num_joints
        if idx > 0:
            classifier_train_args.missing_joint_probabilities[idx - 1] = 1.0
        classifier_train_args.noise = 'zero'
        classifier_train_args.noise_param = 0

        # if recurrent GNNs are used, batch_size should be 1; TODO: make this smarter instead of listing all possible models here
        if flow_args.classification_model_name in ['GConvLSTM', 'DCRNN', 'TGCN', 'GConvGRU']:
            classifier_train_args.log_every_n_steps = classifier_train_args.batch_size * \
                classifier_train_args.log_every_n_steps
            classifier_train_args.batch_size = 1

        tag = CARLA_SKELETON(idx - 1).name if idx > 0 else 'baseline'
        (_, _, trainer) = modeling_main(
            classifier_train_args,
            version=version,
            standalone=True,
            return_trainer=True,
            tags=[
                experiment_tag,
                tag,
            ],
            project='sensitivity'
        )

        metrics[tag] = {k: v for k,
                        v in trainer.logged_metrics.items() if k.startswith('hp')}

        del trainer

    for name, results in metrics.items():
        print_metrics(results, f'{name} metrics:')

    summary_run = wandb.init(
        name=summary_version,
        entity="carla-pedestrians",
        project='sensitivity',
        tags=[
            experiment_tag,
            'summary'
        ],
    )

    metrics_data = []
    columns = ['metric'] + list(metrics.keys())

    for metric in metrics[list(metrics.keys())[0]].keys():
        row = [metric]
        for name, results in metrics.items():
            row.append(results[metric].item())
        metrics_data.append(row)

    metrics_table = wandb.Table(data=metrics_data, columns=columns)

    weights_data = []
    for row in metrics_data:
        name = row[0]
        baseline = np.array(row[1])
        joints = np.array(row[2:(num_joints+2)])
        weights = baseline / joints
        normalized_weights = F.softmax(torch.tensor(weights)).numpy()
        weights_data.append([name, *normalized_weights, np.var(normalized_weights)])

    weights_table = wandb.Table(
        data=weights_data, columns=columns[:1] + columns[2:] + ['var'])

    summary_run.log({
        'metrics_table': metrics_table,
        'weights_table': weights_table,
    })

    summary_run.finish()


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
