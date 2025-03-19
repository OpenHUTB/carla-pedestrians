
import torch
from configs import g_conf
from collections import OrderedDict

import matplotlib.pyplot as plt
import os


class CIL_multiview_Evaluator(object):
    """
    评估
    """
    def __init__(self, name):
        self.reset()
        self.name = name

    def reset(self):
        self._action_batch_errors_mat = 0
        self._total_num = 0
        self._metrics = {}
        self.steers =[]
        self.accelerations=[]
        self.gt_steers =[]
        self.gt_accelerations=[]

    def process(self, action_outputs, targets_action):
        """
        计算验证数据集中神经网络的输出和目标的误差总和
        """
        B = action_outputs.shape[0]
        self._total_num += B
        action_outputs = action_outputs[:, -1, -len(g_conf.TARGETS):]
        self.steers += list(action_outputs[:,0].detach().cpu().numpy())
        self.accelerations += list(action_outputs[:, 1].detach().cpu().numpy())
        self.gt_steers += list(targets_action[-1][:, 0].detach().cpu().numpy())
        self.gt_accelerations += list(targets_action[-1][:, 1].detach().cpu().numpy())
        # 模型预测的输出动作 action_outputs - 真实的目标动作targets_action
        actions_loss_mat_normalized = torch.clip(action_outputs, -1, 1) - targets_action[-1]  # (B, len(g_conf.TARGETS))

        # 对输出和目标进行非标准化处理以计算实际误差
        if g_conf.ACCELERATION_AS_ACTION:
            self._action_batch_errors_mat += torch.abs(actions_loss_mat_normalized)  # [-1, 1]

        else:
            pass

    def evaluate(self, current_epoch, dataset_name):
        self.metrics_compute(self._action_batch_errors_mat)  # 性能衡量的计算
        results = OrderedDict({self.name: self._metrics})

        plt.figure()
        W, H = plt.gcf().get_size_inches()
        plt.gcf().set_size_inches([4.0*W, H])
        # 绿色表示真实的加速度值（gt_accelerations）、蓝色表示预测的加速度值(accelerations)
        # 理想的情况下随着训练的进行，预测的加速度会向真实加速度靠拢
        plt.plot(range(len(self.gt_accelerations)), self.gt_accelerations, color = 'green')
        plt.plot(range(len(self.accelerations)), self.accelerations, color = 'blue')
        plt.ylim([-1.2, 1.2])  # 实际的值在[-1,1]之间
        plt.xlabel('frame id')
        plt.ylabel('')
        # 保存 加速度 测试的精度图
        plt.savefig(os.path.join(g_conf.EXP_SAVE_PATH, 'acc_'+dataset_name+'_epoch'+str(current_epoch)+'.jpg'))
        plt.close()

        plt.figure()
        W, H = plt.gcf().get_size_inches()
        plt.gcf().set_size_inches([4.0*W, H])
        plt.plot(range(len(self.gt_steers)), self.gt_steers, color='green')
        plt.plot(range(len(self.steers)), self.steers, color='blue')
        plt.ylim([-1.2, 1.2])
        plt.xlabel('frame id')
        plt.ylabel('')
        # 保存 方向盘的 平均绝对误差
        plt.savefig(os.path.join(g_conf.EXP_SAVE_PATH, 'steer_'+dataset_name+'_epoch'+str(current_epoch)+'.jpg'))
        plt.close()
        return results

    # 性能衡量的计算
    def metrics_compute(self, action_errors_mat):
        # 方向盘的平均绝对误差(MAE, Mean Absolute Error)
        self._metrics.update({'MAE_steer': torch.sum(action_errors_mat, 0)[0] / self._total_num})
        if g_conf.ACCELERATION_AS_ACTION:
            self._metrics.update({'MAE_acceleration': torch.sum(action_errors_mat, 0)[1] / self._total_num})
        else:
            pass
        self._metrics.update({'MAE': torch.sum(action_errors_mat) / self._total_num})


