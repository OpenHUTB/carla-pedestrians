import datetime
import logging
import time
import torch
import os
from contextlib import contextmanager

from _utils.utils import extract_targets, extract_commands, \
    write_model_results, is_result_better, extract_other_inputs, draw_offline_evaluation_results, eval_done
from configs import g_conf
from logger import _logger
from dataloaders.transforms import inverse_normalize


@contextmanager
def inference_context(model):
    """
    模型暂时更改为评估模式，然后恢复到以前模式的环境。

    Args:
        model: 一个 torch 模块
    """
    training_mode = model.training
    model.eval()
    yield
    model.train(training_mode)


def evaluation_on_model(model, data_loaders, model_name, evaluator, eval_iteration, eval_epoch):
    results_dict = {}
    for data_loader in data_loaders:
        total = len(data_loader.dataset)  # 推理数据加载器必须具有固定长度
        dataset_name = data_loader.dataset.dataset_name
        info = "Start evaluation for model {} on {} with {} images at iteration {} / epoch {} ".format(model_name, dataset_name,
                                                                                                       total, eval_iteration,
                                                                                                       eval_epoch)
        print(info)
        evaluator.reset()
        logging_interval = 500
        start_time = time.time()

        with inference_context(model), torch.no_grad():
            for idx, x in enumerate(data_loader):
                src_images = [[x['current'][i][camera_type].cuda() for camera_type in g_conf.DATA_USED] for i in range(len(x['current']))]
                src_directions = [extract_commands(x['current'][i]['can_bus']['direction']).cuda() for i in
                                      range(len(x['current']))]
                src_speeds = [extract_other_inputs(x['current'][i]['can_bus'], g_conf.OTHER_INPUTS,
                                              ignore=['direction']).cuda() for i in range(len(x['current']))]

                # 数据集中的 CAN总线文件 can_bus000000.json 中包含了：
                # 加速度：acceleration
                # 刹车：brake
                # 方向：direction
                # 自我车辆位置：ego_position (x,y,z)
                # 速度：speed
                # 方向盘：steer
                # 油门：throttle
                if g_conf.ENCODER_OUTPUT_STEP_DELAY > 0 or g_conf.DECODER_OUTPUT_FRAMES_NUM != g_conf.ENCODER_INPUT_FRAMES_NUM:
                    tgt_a = [extract_targets(x['future'][i]['can_bus_future'], g_conf.TARGETS).cuda() for i in range(len(x['future']))]
                else:
                    tgt_a = [extract_targets(x['current'][i]['can_bus'], g_conf.TARGETS).cuda() for i in range(len(x['current']))]

                action_outputs, src_layers, tx_en_attn_weights = model.forward_eval(src_images, src_directions, src_speeds)
                torch.cuda.synchronize()
                evaluator.process(action_outputs, tgt_a)  # 模型的动作输出 action_output - 真实的动作 tgt_a

                """
                ################################################
                    向 TensorBoard 添加可视化
                #################################################
                """

                if idx in list(range(0, min(g_conf.EVAL_IMAGE_WRITING_NUMBER, len(data_loader)))):
                    # 每批仅保存一个以节省时间
                    eval_images = [[x['current'][i][camera_type][:1].cuda() for camera_type in g_conf.DATA_USED] for i
                                   in range(len(x['current']))]
                    eval_directions = [extract_commands(x['current'][i]['can_bus']['direction'])[:1].cuda() for i in
                                       range(len(x['current']))]
                    eval_speeds = [extract_other_inputs(x['current'][i]['can_bus'], g_conf.OTHER_INPUTS,
                                                        ignore=['direction'])[:1].cuda() for i in
                                   range(len(x['current']))]
                    input_frames = []
                    for frame in eval_images:
                        cams = []
                        for i in range(len(g_conf.DATA_USED)):
                            cams.append(inverse_normalize(frame[i], g_conf.IMG_NORMALIZATION['mean'], g_conf.IMG_NORMALIZATION['std']).detach().cpu().numpy().squeeze())
                        input_frames.append(cams)

                    # 我们只保存第一批
                    if g_conf.EVAL_SAVE_LAST_Conv_ACTIVATIONS:
                        _logger.add_gradCAM_attentions_to_disk('Valid', model, [eval_images, eval_directions, eval_speeds],
                                                               input_rgb_frames= input_frames,
                                                               epoch=eval_epoch,
                                                               save_path=os.path.join(g_conf.EXP_SAVE_PATH, 'Eval', 'Valid_gradCAM' + '_' + dataset_name),
                                                               batch_id=idx)


                if (idx + 1) % logging_interval == 0:
                    duration = time.time() - start_time
                    seconds_per_img = duration / ((idx + 1)*g_conf.EVAL_BATCH_SIZE)
                    eta = datetime.timedelta(seconds=int(seconds_per_img * total - duration))
                    info = "Evaluation done {}/{}. {:.4f} s / img. ETA={}".format((idx + 1)*g_conf.EVAL_BATCH_SIZE, total, seconds_per_img, str(eta))
                    print(info)

                del src_images
                del src_directions
                del tgt_a
                del action_outputs
                del src_layers
                del tx_en_attn_weights

        results = evaluator.evaluate(eval_epoch, dataset_name)
        if results is None:
            results = {}
        results_dict[dataset_name]=results
    return results_dict


#@timeit
def save_model_if_better(results_dict, model, optimizer, save_all=False):
    # 如果该模型比前一个更好，我们会保存该模型
    if g_conf.PROCESS_NAME == 'train_val':
        dataset_name = list(results_dict.keys())[0]
        results = results_dict[dataset_name]
        is_better_flag = is_result_better(g_conf.EXP_SAVE_PATH, model.name, dataset_name)
        # 强制保存所有检查点
        if save_all:
            print('Checkpoint at iteration {} / epoch {} is saved'.format(str(model._current_iteration-1), str(model._done_epoch)))
            if is_better_flag:
                best_pred = results[model.name]['MAE']
                save_model_if_better.best_pred = best_pred
            else:
                if hasattr(save_model_if_better, "best_pred"):
                    best_pred = save_model_if_better.best_pred
                else:
                    best_pred = 0
            if isinstance(model, torch.nn.DataParallel):
                model_state_dict = model.module.state_dict()
            else:
                model_state_dict = model.state_dict()
            saving_dict = {
                'model': model_state_dict,
                'optimizer': optimizer.state_dict(),
                'iteration': model._current_iteration-1,
                'epoch': model._done_epoch,
                'best_pred': best_pred
            }
            torch.save(saving_dict, os.path.join(g_conf.EXP_SAVE_PATH, 'checkpoints', str(model.name) + '_' +
                                                 str(model._done_epoch)+ '.pth'))

        else:
            if is_better_flag:
                print('Checkpoint at iteration {} / epoch {} is saved'.format(str(model._current_iteration-1), str(model._done_epoch)))
                best_pred = results[model.name]['MAE']

                save_model_if_better.best_pred = best_pred
                if isinstance(model, torch.nn.DataParallel):
                    model_state_dict = model.module.state_dict()
                else:
                    model_state_dict = model.state_dict()
                saving_dict = {
                    'model': model_state_dict,
                    'optimizer': optimizer.state_dict(),
                    'iteration': model._current_iteration-1,
                    'epoch': model._done_epoch,
                    'best_pred': best_pred
                }
                torch.save(saving_dict, os.path.join(g_conf.EXP_SAVE_PATH, 'checkpoints', str(model.name) + '_' +
                                                 str(model._done_epoch)+ '.pth'))
            else:
                if hasattr(save_model_if_better, "best_pred"):
                    best_pred = save_model_if_better.best_pred
                else:
                    best_pred = 0

    else:
        raise NotImplementedError('Not found process name !')

    return is_better_flag, best_pred


#@timeit
def evaluation_saving(model, optimizers, early_stopping_flags, save_all_checkpoints = False):
    """
    如果模型更好，则进行评估并保存
    """
    if g_conf.PROCESS_NAME == 'train_val':
        if (model._done_epoch != 0) and (model._done_epoch in g_conf.EVAL_SAVE_EPOCHES) \
                and ((model._current_iteration-1)*g_conf.BATCH_SIZE <= len(model) * model._done_epoch):

            # 检查 检查点是否已被评估
            if not eval_done(os.path.join(os.environ["TRAINING_RESULTS_ROOT"], '_results', g_conf.EXPERIMENT_BATCH_NAME,g_conf.EXPERIMENT_NAME),
                                 g_conf.VALID_DATASET_NAME, model._done_epoch):
                print('')
                print('---------------------------------------------------------------------------------------')
                print('')
                print('Evaluating epoch:', str(model._done_epoch))
                # 切换到评估模式
                model.eval()
                results_dict = model._eval(model._current_iteration, model._done_epoch)
                if results_dict is not None:
                    write_model_results(g_conf.EXP_SAVE_PATH, model.name,
                                        results_dict, acc_as_action=g_conf.ACCELERATION_AS_ACTION)
                    draw_offline_evaluation_results(g_conf.EXP_SAVE_PATH, metrics_list=g_conf.EVAL_DRAW_OFFLINE_RESULTS_GRAPHS,
                                                    x_range=g_conf.EVAL_SAVE_EPOCHES)
                    is_better_flag, best_pred = save_model_if_better(results_dict, model, optimizers, save_all=save_all_checkpoints)
                    if g_conf.EARLY_STOPPING:
                        early_stopping_flags.append(not is_better_flag)
                    else:
                        early_stopping_flags.append(False)
                else:
                    raise ValueError('No evaluation results !')
                # 切换回训练模式并继续训练
                model.train()
                print('')
                print('---------------------------------------------------------------------------------------')
                print('')

    else:
        raise NotImplementedError('No this type of process name defined')

    return early_stopping_flags