import argparse
import hashlib
import itertools
from multiprocessing.pool import ThreadPool
import sys
from typing import List
from tqdm.auto import tqdm
import yaml
import os
import subprocess

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


def work(variant_config, logs_dir, pbar):
    arg_tuples = [
        (f'--{k}', *v) if not isinstance(v, str) and getattr(v, '__iter__',
                                                                False) else ((f'--{k}',) if v is None else (f'--{k}={v}',))
        for k, v in variant_config.items()
    ]
    arg_list = [arg for arg_tuple in arg_tuples for arg in arg_tuple]
    arg_hash = hashlib.md5(' '.join(arg_list).encode()).hexdigest()

    with open(os.path.join(logs_dir, 'stdout', f'{arg_hash}.out'), 'w') as f:
        subprocess.run(
            ['python', '-m', 'pedestrians_video_2_carla'] + arg_list,
            stdout=f,
            stderr=subprocess.STDOUT
        )

    pbar.update(1)


def main(args: List[str]):
    """
    :param args: command line parameters as list of strings
          (for example  ``["--verbose"]``).
    :type args: List[str]
    """

    parser = argparse.ArgumentParser(description="""
        Helper script to run multiple tests in parallel.
        This is NOT intended to be used for hyperparameter tuning,
        but to compare the performance of different, predefined models/variants,
        how the models/variants behave with various data scenarios (noise, missing points)
        or simply queue some experiments.
        """)
    parser.add_argument(
        '-c',
        '--config',
        type=str,
        default=None,
        help='path to the configuration file'
    )
    parser.add_argument(
        "-r",
        "--root_dir",
        dest="root_dir",
        help="Root directory for the outputs/datasets/logs resolving.",
        default=os.environ.get("VIDEO2CARLA_ROOT_DIR", "/"),
        type=str,
    )
    parser.add_argument(
        "-n",
        "--num_workers",
        dest="num_workers",
        type=int,
        default=4,
        help="Number of workers to use for tests parallelization."
    )
    args = parser.parse_args(args)

    with open(args.config, 'r') as f:
        config = yaml.load(f, Loader=Loader)

    # ensure log dir exists
    logs_dir = config['common_params'].get('logs_dir', 'compare_logs')
    if not os.path.isabs(logs_dir):
        logs_dir = os.path.join(args.root_dir, logs_dir)
    if logs_dir is not None:
        os.makedirs(logs_dir, exist_ok=True)
        os.makedirs(os.path.join(logs_dir, 'stdout'), exist_ok=True)

    # what models to use?
    if 'movements_model_name' in config['compare_params']:
        models = config['compare_params']['movements_model_name']
        del config['compare_params']['movements_model_name']
    else:
        models = [config['common_params']['movements_model_name']]
        del config['common_params']['movements_model_name']

    if 'compare_model' not in config:
        config['compare_model'] = {}
    if 'common_model' not in config:
        config['common_model'] = {}

    # how many 'common' tests will be performed?
    compare_params_count = sum([len(config['compare_params'][param])
                               for param in config['compare_params'] if param != 'movements_model_name'])

    # how many model variants will be tested?
    model_variants_count = sum([len(config['compare_model'].get(model, {'dummy': [None]})[k])
                                for model in models
                                for k in config['compare_model'].get(model, {'dummy': [None]})])

    # how many tests will be performed?
    tests_count = compare_params_count * model_variants_count

    pbar = tqdm(total=tests_count, desc='Tests')
    tp = ThreadPool(processes=args.num_workers)

    for model in models:
        model_variants = config['compare_model'].get(model, {})
        model_common = config['common_model'].get(model, {})
        param_variants = config['compare_params']
        param_common = config['common_params']

        common_config = {
            **param_common,
            **model_common,
        }

        keys = list(model_variants.keys()) + list(param_variants.keys())
        for combination in itertools.product(*model_variants.values(), *param_variants.values()):
            variant_config = {
                'movements_model_name': model,
                **common_config,
                **dict(zip(keys, combination)),
                'root_dir': args.root_dir,
            }

            tp.apply_async(work, (variant_config, logs_dir, pbar))

    tp.close()
    tp.join()


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
