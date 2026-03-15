from __future__ import unicode_literals
import os
import torch
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from .tensorboard_logger import Logger
from _utils.grad_cam.grad_cam import GradCAM

# 我们将文件名保存在 glogger 中，以避免包含全局 global
TRAIN_IMAGE_LOG_FREQUENCY = 1
TRAIN_LOG_FREQUENCY = 1
tl = ''


def create_log(save_full_path, train_log_frequency=1, train_image_log_frequency=15):
               #eval_log_frequency=1, image_log_frequency=15):

    """

    Arguments
        save_full_path: 保存 tensorboard 日志的完整路径
        log_frequency: 对数值进行记录的频率
        image_log_frequency: 记录图像的频率
    """
    global tl
    global TRAIN_LOG_FREQUENCY
    global TRAIN_IMAGE_LOG_FREQUENCY

    TRAIN_LOG_FREQUENCY = train_log_frequency
    TRAIN_IMAGE_LOG_FREQUENCY = train_image_log_frequency
    tl = Logger(os.path.join(save_full_path, 'tensorboard_logs'))


def add_scalar(tag, value, iteration=None):

    """
        用于在 tensorboard 上记录原始输出。
    """

    if iteration is not None:
        if iteration % TRAIN_LOG_FREQUENCY == 0:
            tl.scalar_summary(tag, value, iteration)
    else:
        raise ValueError('iteration is not supposed to be None')


def add_gradCAM_attentions_to_disk(process_type, model, source_input, input_rgb_frames,
                                            epoch, save_path=None, batch_id=None):

    global TRAIN_IMAGE_LOG_FREQUENCY
    cmap = plt.get_cmap('jet')

    # 用于保存主干的训练注意力图
    if process_type == 'Train':
        pass

    # 用于保存主干的验证注意力图
    elif process_type == 'Valid':
        S = len(source_input[0])
        cam_num = len(source_input[0][0])
        _, C, H, W = source_input[0][0][0].shape

        target_layers = [model._model.encoder_embedding_perception.layer4[-1]]
        cam = GradCAM(model=model._model, target_layers=target_layers)

        with torch.enable_grad():
            grayscale_cam = cam(input_tensor_list=source_input)   # [S*cam, H, W]

        grayscale_cam = grayscale_cam.reshape((S, cam_num, H, W))

        Seq = []
        for s in range(S):
            cams = []
            for cam_id in range(cam_num):
                att = grayscale_cam[s, cam_id, :]
                cmap_att = np.delete(cmap(att), 3, 2)
                cmap_att = Image.fromarray((cmap_att * 255).astype(np.uint8))
                # cams.append(cmap_att)
                cams.append(Image.blend(Image.fromarray(((input_rgb_frames[s][cam_id]).transpose(1, 2, 0) * 255).astype(np.uint8)), cmap_att, 0.5))
            Seq.append(np.concatenate(cams, 1))
        current_att = np.concatenate(Seq, 0)

        if save_path:
            if not os.path.exists(os.path.join(save_path, str(epoch), '-1')):
                os.makedirs(os.path.join(save_path, str(epoch), '-1'))

            # 我们将所需的骨干层保存到磁盘
            current_att = Image.fromarray(current_att)
            current_att.save(os.path.join(save_path, str(epoch), '-1',
                             str(batch_id) +'.jpg'))

        else:
            raise RuntimeError('You need to set the save_path')