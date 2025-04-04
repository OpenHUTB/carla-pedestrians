# 该主文件用于显示如何使用这里的视觉算法。
import os
import argparse

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description=__doc__)

    argparser.add_argument(
        '--process-type',
        default=None,
        type=str,
        required=True
    )

    argparser.add_argument(
        '--gpus',
        nargs='+',
        dest='gpus',
        type=str,
        required=True
    )

    argparser.add_argument(
        '-f',
        '--folder',
        default='NoDate',
        dest='folder',
        type=str,
        help='The folder of the configuration files'
    )
    argparser.add_argument(
        '-e',
        '--exp',
        default=None,
        dest='exp',
        type=str,
        help='The experiment name of the configuration file'
    )

    argparser.add_argument(
        '--training_results_root',
        default=None,
        type=str,
        required=True
    )

    argparser.add_argument(
        '--dataset_path',
        default=None,
        type=str,
        required=True
    )

    args = argparser.parse_args()

    if args.training_results_root is not None:
        os.environ.setdefault('TRAINING_RESULTS_ROOT', args.training_results_root)
    else:
        raise Exception("TRAINING_RESULTS_ROOT is required")

    if args.dataset_path is not None:
        os.environ.setdefault('DATASET_PATH', args.dataset_path)
    else:
        raise Exception("DATASET_PATH is required")

    if args.gpus:
        # 检查传递的 GPU 向量是否有效。
        for gpu in args.gpus:
            try:
                int(gpu)
            except ValueError:  # 重新抛出有意义的错误。
                raise ValueError("GPU is not a valid int number")
        os.environ["CUDA_VISIBLE_DEVICES"] = ','.join(args.gpus)  # 这必须先于整个执行
    else:
        raise ValueError('You need to define the ids of GPU you want to use by adding: --gpus')

    from console import execute_train_val, execute_val
    if args.process_type is not None:
        if args.process_type == 'train_val':
            if args.exp is None:
                raise ValueError("You should set the exp alias")
            execute_train_val(gpus_list=args.gpus, exp_batch=args.folder, exp_alias=args.exp)

        elif args.process_type == 'val_only':
            if args.exp is None:
                raise ValueError("You should set the exp alias")
            execute_val(gpus_list=args.gpus, exp_batch=args.folder, exp_alias=args.exp)

        else:
            raise Exception("Invalid name for --process-type, chose from (train_val, train_only, val_only)")

    else:
        raise Exception(
            "You need to define the process type with argument '--process-type': train_val, train_only, val_only")

