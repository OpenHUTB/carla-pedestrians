import os
import torch
from configs import g_conf, set_type_of_process, merge_with_yaml
from network.models_console import Models
from _utils.training_utils import seed_everything, DataParallelWrapper, check_saved_checkpoints_in_total
from _utils.utils import write_model_results, draw_offline_evaluation_results, eval_done


def val_task(model):
    """
    用于评估模型的上游任务
    """

    model.eval()
    results_dict = model._eval(model._current_iteration, model._done_epoch)
    if results_dict is not None:
        write_model_results(g_conf.EXP_SAVE_PATH, model.name,
                            results_dict, acc_as_action=g_conf.ACCELERATION_AS_ACTION)
        draw_offline_evaluation_results(g_conf.EXP_SAVE_PATH, metrics_list=g_conf.EVAL_DRAW_OFFLINE_RESULTS_GRAPHS,
                                                    x_range=g_conf.EVAL_SAVE_EPOCHES)

    else:
        raise ValueError('No evaluation results !')


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
    rlimit = resource.getrlimit(resource.RLIMIT_NOFILE)  # 使用当前资源的软限制和硬限制返回一个元组（软，硬）。
    # 设定资源消耗的新限制。该限制参数必须是一个元组(soft, hard)两个整数描述的新限制的。
    # 软限制可以被一个进程改变，但是不能大于硬限制。硬限制只能被降低，但是必须大于软限制。想要提高硬限制，必须是超级用户。
    # resource.RLIMIT_NOFILE： 当前进程的最大打开文件描述符数。
    resource.setrlimit(resource.RLIMIT_NOFILE, (4096, rlimit[1]))
    print(torch.cuda.device_count(), 'GPUs to be used: ', gpus_list)
    merge_with_yaml(os.path.join('configs', exp_batch, exp_name + '.yaml'))
    set_type_of_process('val_only', root=os.environ["TRAINING_RESULTS_ROOT"])
    seed_everything(seed=g_conf.MAGICAL_SEED)

    model = Models(g_conf.MODEL_TYPE, g_conf.MODEL_CONFIGURATION)
    if len(gpus_list) > 1 and g_conf.DATA_PARALLEL:
        print("Using multiple GPUs parallel! ")
        model = DataParallelWrapper(model)
    # 检查我们是否从零开始训练模型，或者是否继续在之前的模型上进行训练
    all_checkpoints = check_saved_checkpoints_in_total(os.path.join(os.environ["TRAINING_RESULTS_ROOT"], '_results', g_conf.EXPERIMENT_BATCH_NAME,
                                                             g_conf.EXPERIMENT_NAME, 'checkpoints'))
    if all_checkpoints is not None:
        for eval_checkpoint in all_checkpoints:
            if int(eval_checkpoint.split('_')[-1].split('.')[0]) in g_conf.EVAL_SAVE_EPOCHES:
                if not eval_done(os.path.join(
                        os.environ["TRAINING_RESULTS_ROOT"], '_results', g_conf.EXPERIMENT_BATCH_NAME, g_conf.EXPERIMENT_NAME),
                                 g_conf.VALID_DATASET_NAME,
                        eval_checkpoint.split('_')[-1].split('.')[0]):
                    checkpoint = torch.load(eval_checkpoint)
                    if isinstance(model, torch.nn.DataParallel):
                        model.module.load_state_dict(checkpoint['model'])
                    else:
                        model.load_state_dict(checkpoint['model'])

                    model._current_iteration = checkpoint['iteration'] + 1
                    model._done_epoch = checkpoint['epoch']
                    print('')
                    print('---------------------------------------------------------------------------------------')
                    print('')
                    print('Evaluating epoch:', str(checkpoint['epoch']))

                    model.cuda()
                    val_task(model)
        print('')
        print('---------------------------------------------------------------------------------------')
        print('Evaluation finished !!')
    else:
        print('')
        print('---------------------------------------------------------------------------------------')
        print('No checkpoints to be evaluated. Done')



