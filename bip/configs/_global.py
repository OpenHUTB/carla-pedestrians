from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from ast import literal_eval
from configs.attribute_dict import AttributeDict
import copy
import numpy as np
import os
import yaml

from logger._logger import create_log

_g_conf = AttributeDict()

_g_conf.immutable(False)

"""#### 常规配置参数 ####"""
"""#### 训练相关参数"""
_g_conf.MAGICAL_SEED = 0
_g_conf.NUM_WORKER = 14
_g_conf.DATA_PARALLEL = False
_g_conf.TRAINING_RESUME = True
_g_conf.BATCH_SIZE = 120
_g_conf.NUMBER_EPOCH = 100     # 训练迭代总次数
_g_conf.TRAIN_DATASET_NAME = []
_g_conf.VALID_DATASET_NAME = []      # 可以评估多个数据集，因此需要列出一个列表
_g_conf.DATA_USED = ['rgb_left', 'rgb_central', 'rgb_right']
_g_conf.IMAGE_SHAPE = [3, 88, 200]
# 您要使用的帧步长。
# 例如，如果您希望有 5 个连续输入图像，每个图像以 20 帧为步长，则应设置 INPUT_FRAMES_NUM =5 和 INPUT_FRAME_INTERVAL=20
_g_conf.ENCODER_INPUT_FRAMES_NUM = 1
_g_conf.ENCODER_STEP_INTERVAL = 1
_g_conf.ENCODER_OUTPUT_STEP_DELAY = 0  # 我们是想预测未来的数据点，还是只预测当前的数据点
_g_conf.DECODER_OUTPUT_FRAMES_NUM= 1
_g_conf.AUGMENTATION = False
_g_conf.DATA_FPS = 10
_g_conf.DATA_COMMAND_CLASS_NUM = 4
_g_conf.DATA_COMMAND_ONE_HOT = True
_g_conf.DATA_NORMALIZATION = {}
_g_conf.IMG_NORMALIZATION = {'mean': [0.485, 0.456, 0.406], 'std': [0.229, 0.224, 0.225]}   # 默认使用 ImageNet
_g_conf.EXP_SAVE_PATH = '_results'
_g_conf.TARGETS = ['steer', 'throttle', 'brake']  # 从浮点数据中，网络应该估计
_g_conf.ACCELERATION_AS_ACTION = False
_g_conf.OTHER_INPUTS= ['speed'] # 从浮点数据中，输入到神经网络的

"""#### 优化器相关参数 ####"""
_g_conf.LOSS = ''    # 它可以是损失的名称，例如 L1、CrossEntropy，也可以是架构名称，例如“faster R cnn，deeplabv3”，这意味着我们使用与该架构相同的损失
_g_conf.LOSS_WEIGHT = {}
_g_conf.LEARNING_RATE = 0.0002       # 原始学习率设置
_g_conf.LEARNING_RATE_DECAY = True
_g_conf.LEARNING_RATE_MINIMUM = 0.00001
_g_conf.LEARNING_RATE_DECAY_EPOCHES = []    # 我们每 1000 次迭代调整一次学习率
_g_conf.LEARNING_RATE_POLICY = {'name': 'normal', 'level': 0.5, 'momentum': 0, 'weight_decay': 0}   # 对于每个 LEARNING_RATE_STEP，lr 乘以 0.5

"""#### 网络相关参数 ####"""
_g_conf.MODEL_TYPE = ''
_g_conf.MODEL_CONFIGURATION = {}
_g_conf.IMAGENET_PRE_TRAINED = False
_g_conf.LOAD_CHECKPOINT = ''

"""#### 验证相关参数"""
_g_conf.EVAL_SAVE_LAST_Conv_ACTIVATIONS = True     # 要保存的注意力图的骨干的最后一层卷积层
_g_conf.EVAL_BATCH_SIZE = 1          # 评估的批次大小
_g_conf.EVAL_SAVE_EPOCHES = [1]      # 我们指定要进行离线评估的时期
_g_conf.EVAL_IMAGE_WRITING_NUMBER = 10
_g_conf.EARLY_STOPPING = False          # 默认情况下，我们不应用提前停止
_g_conf.EARLY_STOPPING_PATIENCE = 3
_g_conf.EVAL_DRAW_OFFLINE_RESULTS_GRAPHS = ['MAE']

"""#### 日志相关参数"""
_g_conf.TRAIN_LOG_SCALAR_WRITING_FREQUENCY = 2
_g_conf.TRAIN_IMAGE_WRITING_NUMBER = 2
_g_conf.TRAIN_IMAGE_LOG_FREQUENCY = 1000
_g_conf.TRAIN_PRINT_LOG_FREQUENCY = 100


def merge_with_yaml(yaml_filename, process_type='train_val'):
    """加载 yaml 配置文件并将其合并到全局配置对象中"""
    global _g_conf
    with open(yaml_filename, 'r') as f:

        yaml_file = yaml.load(f)

        yaml_cfg = AttributeDict(yaml_file)

    path_parts = os.path.split(yaml_filename)
    if process_type == 'train_val':
        _g_conf.EXPERIMENT_BATCH_NAME = os.path.split(path_parts[-2])[-1]
        _g_conf.EXPERIMENT_NAME = path_parts[-1].split('.')[-2]
    if process_type == 'val_only':
        _g_conf.EXPERIMENT_BATCH_NAME = os.path.split(path_parts[-2])[-1]
        _g_conf.EXPERIMENT_NAME = path_parts[-1].split('.')[-2]
    elif process_type == 'drive':
        _g_conf.EXPERIMENT_BATCH_NAME = os.path.split(os.path.split(path_parts[-2])[-2])[-1]
        _g_conf.EXPERIMENT_NAME = os.path.split(path_parts[-2])[-1]
    _merge_a_into_b(yaml_cfg, _g_conf)


def get_names(folder):
    alias_in_folder = os.listdir(os.path.join('configs', folder))

    experiments_in_folder = {}
    for experiment_alias in alias_in_folder:

        g_conf.immutable(False)
        merge_with_yaml(os.path.join('configs', folder, experiment_alias))

        experiments_in_folder.update({experiment_alias: g_conf.EXPERIMENT_GENERATED_NAME})

    return experiments_in_folder


# 创建实验路径
def create_exp_path(root, experiment_batch, experiment_name):
    # 这是硬编码的，日志始终保留在 _logs 文件夹中
    root_path = os.path.join(root, '_results')
    if not os.path.exists(os.path.join(root_path, experiment_batch, experiment_name)):
        os.makedirs(os.path.join(root_path, experiment_batch, experiment_name))


def set_type_of_process(process_type, root):
    """
    此函数用于设置当前过程的类型，测试、训练或验证，以及每个过程的详细信息，因为单个实验可能有许多验证和测试。

    注意：调用此功能后，配置将关闭

    Args:
        type:

    Returns:

    """

    if process_type == 'train_val' or process_type == 'train_only' or process_type == 'val_only' or process_type == 'drive':
        _g_conf.PROCESS_NAME = process_type
        if not os.path.exists(os.path.join(root,'_results', _g_conf.EXPERIMENT_BATCH_NAME,
                                           _g_conf.EXPERIMENT_NAME,
                                           'checkpoints')):
            os.mkdir(os.path.join(root,'_results', _g_conf.EXPERIMENT_BATCH_NAME,
                                  _g_conf.EXPERIMENT_NAME,
                                  'checkpoints'))

        _g_conf.EXP_SAVE_PATH = os.path.join(root,'_results', _g_conf.EXPERIMENT_BATCH_NAME, _g_conf.EXPERIMENT_NAME)

    else:
        raise ValueError("Not found type of process")

    if process_type == 'train_val' or process_type == 'train_only' or process_type == 'val_only':
        create_log(_g_conf.EXP_SAVE_PATH,
                _g_conf.TRAIN_LOG_SCALAR_WRITING_FREQUENCY,
                _g_conf.TRAIN_IMAGE_LOG_FREQUENCY)

    _g_conf.immutable(True)


def _merge_a_into_b(a, b, stack=None):
    """将配置字典 a 合并到配置字典 b 中，当 b 中的选项也在 a 中指定时，会破坏这些选项。
    """

    assert isinstance(a, AttributeDict) or isinstance(a, dict), 'Argument `a` must be an AttrDict'
    assert isinstance(b, AttributeDict) or isinstance(a, dict), 'Argument `b` must be an AttrDict'

    for k, v_ in a.items():
        full_key = '.'.join(stack) + '.' + k if stack is not None else k
        # a 必须指定 b 中的键
        if k not in b:
            # 是否超过第二个堆栈stack
            if stack is not None:
                b[k] = v_
            else:
                raise KeyError('Non-existent config key: {}'.format(full_key))

        v = copy.deepcopy(v_)
        v = _decode_cfg_value(v)

        v = _check_and_coerce_cfg_value_type(v, b[k], k, full_key)

        # 递归合并字典

        b[k] = v


def _decode_cfg_value(v):
    """将原始配置值（例如，来自 yaml 配置文件或命令行参数）解码为 Python 对象。
    """
    # 从原始 yaml 解析的配置将包含需要转换为 AttrDict 对象的字典键

    # 所有剩余的处理仅适用于字符串
    if not isinstance(v, str):
        return v
    # Try to interpret `v` as a:
    #   string, number, tuple, list, dict, boolean, or None
    try:
        v = literal_eval(v)
    # 以下两个例外情况当 v 表示字符串时允许其通过。
    #

    except ValueError:
        pass
    except SyntaxError:
        pass
    return v


def _check_and_coerce_cfg_value_type(value_a, value_b, key, full_key):
    """检查用于替换“value_b”的“value_a”是否具有正确的类型。如果类型完全匹配，或者属于可以轻松强制转换类型的少数情况之一，则类型正确。
    """
    # 类型必须匹配（有一些例外）
    type_b = type(value_b)
    type_a = type(value_a)
    if type_a is type_b:
        return value_a

    # Exceptions: numpy arrays, strings, tuple<->list
    if isinstance(value_b, type(None)):
        value_a = value_a
    elif isinstance(value_b, np.ndarray):
        value_a = np.array(value_a, dtype=value_b.dtype)
    elif isinstance(value_b, str):
        value_a = str(value_a)
    elif isinstance(value_a, tuple) and isinstance(value_b, list):
        value_a = list(value_a)
    elif isinstance(value_a, list) and isinstance(value_b, tuple):
        value_a = tuple(value_a)
    elif isinstance(value_b, range) and not isinstance(value_a, list):
        value_a = eval(value_a)
    elif isinstance(value_b, range) and isinstance(value_a, list):
        value_a = list(value_a)
    elif isinstance(value_b, dict):
        value_a = eval(value_a)
    else:
        raise ValueError(
            'Type mismatch ({} vs. {}) with values ({} vs. {}) for config '
            'key: {}'.format(type_b, type_a, value_b, value_a, full_key)
        )
    return value_a


g_conf = _g_conf

