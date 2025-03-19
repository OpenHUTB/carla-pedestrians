import os
import torch
import time
import shutil
from configs import g_conf, set_type_of_process, merge_with_yaml
from network.models_console import Models
from _utils.training_utils import seed_everything, DataParallelWrapper, check_saved_checkpoints, update_learning_rate
from _utils.utils import extract_targets, extract_other_inputs, extract_commands, print_train_info, test_stop
from _utils.evaluation import evaluation_saving
from logger import _logger


def train_upstream_task(model, optimizer):
    """
    用于模型训练的上游任务

    """
    early_stopping_flags = []
    acc_time = 0.0
    time_start = time.time()

    while True:
        # 我们得到了模型的数据加载器 dataloader
        dataloader = model._train_loader
        for data in dataloader:
            early_stopping_flags = evaluation_saving(model, optimizer, early_stopping_flags, save_all_checkpoints=True)
            if early_stopping_flags and all(early_stopping_flags[-int(g_conf.EARLY_STOPPING_PATIENCE):]):
                print(' Apply early stopping, training stopped !')
                break

            if g_conf.LEARNING_RATE_DECAY:
                if model._done_epoch in g_conf.LEARNING_RATE_DECAY_EPOCHES and \
                        ((model._current_iteration-1)*g_conf.BATCH_SIZE <= len(model) * model._done_epoch):
                    update_learning_rate(optimizer, minimumlr=g_conf.LEARNING_RATE_MINIMUM)

            src_images = [[data['current'][i][camera_type].cuda() for camera_type in g_conf.DATA_USED] for i in range(len(data['current']))]
            src_directions = [extract_commands(data['current'][i]['can_bus']['direction']).cuda() for i in
                              range(len(data['current']))]
            src_s = [extract_other_inputs(data['current'][i]['can_bus'], g_conf.OTHER_INPUTS,
                                     ignore=['direction']).cuda() for i in range(len(data['current']))]
            if g_conf.ENCODER_OUTPUT_STEP_DELAY > 0 or g_conf.DECODER_OUTPUT_FRAMES_NUM != g_conf.ENCODER_INPUT_FRAMES_NUM:
                tgt_a = [extract_targets(data['future'][i]['can_bus_future'], g_conf.TARGETS).cuda() for i in range(len(data['future']))]
            else:
                tgt_a = [extract_targets(data['current'][i]['can_bus'], g_conf.TARGETS).cuda() for i in range(len(data['current']))]

            action_outputs = model.forward(src_images, src_directions, src_s)
            loss_params = {
                'action_output': action_outputs,
                'targets_action': tgt_a,
                'variable_weights': g_conf.LOSS_WEIGHT
            }

            if g_conf.ACCELERATION_AS_ACTION:
                loss, steer_loss, acceleration_loss = model.loss(loss_params)
                acc_time = print_train_info(g_conf.TRAIN_PRINT_LOG_FREQUENCY, g_conf.BATCH_SIZE, model, time_start,
                                            acc_time, loss.item(), steer_loss.item(), acceleration_loss.item())
            else:
                loss, steer_loss, throttle_loss, brake_loss = model.loss(loss_params)
                acc_time = print_train_info(g_conf.TRAIN_PRINT_LOG_FREQUENCY, g_conf.BATCH_SIZE, model, time_start,
                                            acc_time, loss.item(), steer_loss.item(), throttle_loss.item(), brake_loss.item)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            time_start = time.time()
            """
            ################################################
                添加 tensorboard 日志
            #################################################
            """
            _logger.add_scalar('Loss', loss.item(), model._current_iteration)

            # 向 tensorboard 添加损失
            _logger.add_scalar('Loss_steer', steer_loss.item(), model._current_iteration)
            if g_conf.ACCELERATION_AS_ACTION:
                _logger.add_scalar('Loss_acceleration', acceleration_loss.item(), model._current_iteration)
            else:
                _logger.add_scalar('Loss_throttle', throttle_loss.item(), model._current_iteration)
                _logger.add_scalar('Loss_brake', brake_loss.item(), model._current_iteration)

            if test_stop(g_conf.NUMBER_EPOCH*len(model), model._current_iteration * g_conf.BATCH_SIZE):
                print('')
                print('Training finished !!')
                break
            model._current_iteration += 1
            model._done_epoch = (model._current_iteration*g_conf.BATCH_SIZE // len(model))

            del src_images
            del src_directions
            del tgt_a
            del src_s
            del action_outputs

        else:
            continue
        break


# 也许我们可以用默认名称来调用主函数
def execute(gpus_list, exp_batch, exp_name):
    """
        解码器的主要训练功能。
    Args:
        gpus_list: 所有可使用的 GPU 列表
        exp_batch: 包含实验的文件夹
        exp_name: 别名，实验名称

    Returns:
        None

    """
    import resource
    rlimit = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (4096, rlimit[1]))
    print(torch.cuda.device_count(), 'GPUs to be used: ', gpus_list)
    merge_with_yaml(os.path.join('configs', exp_batch, exp_name + '.yaml'))
    shutil.copyfile(os.path.join('configs', exp_batch, exp_name + '.yaml'),
                    os.path.join(os.environ["TRAINING_RESULTS_ROOT"], '_results',
                                 g_conf.EXPERIMENT_BATCH_NAME, g_conf.EXPERIMENT_NAME, exp_name + '.yaml'))
    set_type_of_process('train_val', root=os.environ["TRAINING_RESULTS_ROOT"])
    seed_everything(seed=g_conf.MAGICAL_SEED)

    model = Models(g_conf.MODEL_TYPE, g_conf.MODEL_CONFIGURATION)
    # print("===================== 模型配置 =====================")
    # print("")
    # print(model)

    num_params = 0
    for param in model.parameters():
        num_params += param.numel()
    print('model params: ', num_params)

    # 在Adam的基础上增加了权重衰减（也称为L2正则化），这是对模型参数的系数惩罚，有助于防止过拟合
    optimizer = torch.optim.AdamW(model.parameters(), lr=g_conf.LEARNING_RATE)
    if len(gpus_list) > 1 and g_conf.DATA_PARALLEL:
        print("Using multiple GPUs parallel! ")
        model = DataParallelWrapper(model)

    # 加载特定检查点
    if g_conf.LOAD_CHECKPOINT:
        latest_checkpoint = os.path.join(os.environ["TRAINING_RESULTS_ROOT"], '_results', g_conf.EXPERIMENT_BATCH_NAME,
                                                                g_conf.EXPERIMENT_NAME, 'checkpoints', g_conf.LOAD_CHECKPOINT)

    # 从头开始训练模型，或恢复之前的训练
    elif g_conf.TRAINING_RESUME:
        latest_checkpoint = check_saved_checkpoints(os.path.join(os.environ["TRAINING_RESULTS_ROOT"], '_results', g_conf.EXPERIMENT_BATCH_NAME,
                                                                g_conf.EXPERIMENT_NAME, 'checkpoints'))
    else:
        latest_checkpoint = None

    if latest_checkpoint is not None:
        checkpoint = torch.load(latest_checkpoint)
        pretrained_dict = checkpoint['model']

        if isinstance(model, torch.nn.DataParallel):
            model.module.load_state_dict(pretrained_dict)
        else:
            model.load_state_dict(pretrained_dict)
        optimizer.load_state_dict(checkpoint['optimizer'])
        # 从检查点加载优化器状态后，我们手动将其移动到 GPU 内存
        for state in optimizer.state.values():
            for k, v in state.items():
                if torch.is_tensor(v):
                    state[k] = v.cuda()
        for param_group in optimizer.param_groups:
            print('')
            print('    Resum training from epoch -> ', checkpoint['epoch'])
            print('    Resum the latest learning rate -> ', param_group['lr'])
            if g_conf.LEARNING_RATE_DECAY:
                print('      - learning rate decay at epoch', g_conf.LEARNING_RATE_DECAY_EPOCHES, ', minimum lr:', g_conf.LEARNING_RATE_MINIMUM)
            print('')
            print('=======================================================================================')
            print('')

        model._current_iteration = checkpoint['iteration'] + 1
        model._done_epoch = checkpoint['epoch']
    else:
        print('')
        print('    Training from epoch 0')
        print('    Initial learning rate -> ', g_conf.LEARNING_RATE)
        if g_conf.LEARNING_RATE_DECAY:
            print('      - learning rate decay at epoch', g_conf.LEARNING_RATE_DECAY_EPOCHES, ', minimum lr:', g_conf.LEARNING_RATE_MINIMUM)
        print('')
        print('=======================================================================================')
        print('')

    model.cuda()
    model.train()
    train_upstream_task(model, optimizer)

