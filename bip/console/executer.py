
import os
from configs._global import create_exp_path
from . import train_val, val


def execute_train_val(gpus_list, exp_batch, exp_alias):
    """

    Args:
        gpu: 用于此次执行的 gpu。
        module_name: 模块名称，如果是训练、驾驶驱动或评估
        exp_alias: 要执行的实验别名、文件名。
        path: 数据集的路径

    Returns:

    """
    create_exp_path(os.environ['TRAINING_RESULTS_ROOT'], exp_batch, exp_alias)
    train_val.execute(gpus_list, exp_batch, exp_alias)


def execute_val(gpus_list, exp_batch, exp_alias):
    """

    Args:
        gpu: 用于此次执行的 gpu。
        module_name: 模块名称，如果是训练、驾驶驱动或评估
        exp_alias: 要执行的实验别名、文件名。
        path: 数据集的路径

    Returns:

    """
    create_exp_path(os.environ['TRAINING_RESULTS_ROOT'], exp_batch, exp_alias)
    val.execute(gpus_list, exp_batch, exp_alias)


