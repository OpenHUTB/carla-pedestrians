# 类脑导航

模拟 背侧视觉通路，感知运动

模拟网格细胞(x,y,z)、头部朝向细胞(半规管监测头部旋转角加速度、yaw)、耳石器官（线性运动）。

前庭系统（加速度）-> （后顶叶：空间处理） -> 内嗅皮层（速度细胞） -> 海马体（位置细胞）

前庭核 -> 内侧纵束/网状结构 -> 内侧颞叶（内嗅皮层、海马旁回）

* 4DoF -> 6DoF SLAM
* 同时建模头部朝向细胞核3D网格细胞
* 增加位置细胞的建模（目前位置细胞属性包含在3D网格细胞网络和3D经验地图中）
* 辅以情节记忆
* **🆕 IMU-视觉融合**: 集成惯性测量单元(IMU)与视觉SLAM，显著提升建图精度和轨迹准确性




[内嗅皮层](https://zhuanlan.zhihu.com/p/71551904) 分为内侧内嗅皮层MEC和外侧内嗅皮层LEC。MEC更多地接收来自枕叶、压后皮层、顶叶的输入，LEC更多地接收来自额叶、梨状皮层、脑岛皮层、嗅觉皮层（olfactory cortex）、颞叶的输入。

## 环境配置

### 纯视觉SLAM
1.从 [链接](https://drive.google.com/drive/folders/1AisK9ZlGhv8eCPYeAYtd-GzTHtQxz5NC?usp=sharing) 或 [百度网盘](https://pan.baidu.com/s/1BpIYE4gGPDWPSY5lkhM6qg?pwd=hutb) 下载数据，解压并放到`C:\NeuroSLAM_Datasets`目录下；

2.运行当前目录下的`launch.m`（包括比如：`07_test\test_3d_mapping\QUTCarparkData`目录下的脚本：`test_mapping_QUTCarparkData.m`）。

### 🆕 IMU-视觉融合SLAM
**提升精度30-70%，降低漂移率至<0.1%**

#### 快速开始（5分钟）
```bash
# 1. 启动CARLA仿真器
cd /path/to/carla-0.9.15
./CarlaUE4.sh

# 2. 采集IMU-视觉数据
cd neuro/00_collect_data
python IMU_Vision_Fusion_EKF.py

# 3. 运行SLAM测试 (MATLAB)
cd neuro/07_test/test_imu_visual_slam
test_imu_visual_fusion_slam
```

#### 详细文档
- **快速开始**: `09_vestibular/QUICKSTART_CN.md`
- **完整文档**: `09_vestibular/README_IMU_Visual_Fusion.md`
- **实现总结**: `IMU_VISUAL_FUSION_SUMMARY.md`

#### 性能提升
| 指标 | 纯视觉 | IMU-视觉融合 | 改进 |
|------|--------|-------------|------|
| ATE RMSE | ~1.2m | <0.5m | ↑58% |
| 终点误差 | ~3.5m | <1.0m | ↑71% |
| 漂移率 | ~0.35% | <0.1% | ↑71% |

## 参考
NeuroSLAM: A Brain inspired SLAM System for 3D Environments (c) 2018-2019 Fangwen Yu, Jianga Shang, Youjian Hu and Michael Milford

代码：
https://github.com/cognav/NeuroSLAM.git

数据集:
(Google Drive) https://drive.google.com/drive/folders/10-BEQQkHW1OQIgXWCKjHsuHnqkK-68dc?usp=sharing
or (Baidu Cloud) https://pan.baidu.com/s/19g8V179SWwvWLPcaoe6jHg code：slam 

实验视频：
https://www.neuroslam.net/?page_id=45
or https://drive.google.com/drive/folders/1AisK9ZlGhv8eCPYeAYtd-GzTHtQxz5NC?usp=sharing

论文

Yu, Fangwen, Jianga Shang, Youjian Hu, and Michael Milford. "NeuroSLAM: a brain-inspired SLAM system for 3D environments." Biological Cybernetics (2019): 1-31. https://link.springer.com/article/10.1007/s00422-019-00806-9

神经科学原理 

5.1.3 可以解码海马体的空间认知地图来推断位置

27.3.8 前庭信号对于空间定向和空间导航至关重要

54.5 海马体形成外部世界的空间映射

* [论文解析](https://blog.csdn.net/weixin_38262663/article/details/120004213)
* [代码解析](https://blog.csdn.net/weixin_38262663/article/details/120075175#comments_30727517)
* [用于机器人位置识别的脑启发多模态混合神经网络](https://github.com/cognav/NeuroGPR)
* [视觉惯性导航融合算法研究进展](https://m.chinaaet.com/article/3000159386) 
* [不变卡尔曼滤波的视觉惯性 SLAM 代码](https://github.com/mbrossar/FUSION2018) 

Matlab示例
* [单目视觉-惯性SLAM](https://ww2.mathworks.cn/help/vision/ug/monocular-visual-inertial-slam.html) 
* [使用合成数据的视觉惯性里程计](https://ww2.mathworks.cn/help/fusion/ug/visual-inertial-odometry-using-synthetic-data.html)
* [使用因子图的单目视觉惯性里程计（VIO ）](https://ww2.mathworks.cn/help/nav/ug/monocular-visual-inertial-odometry-using-factor-graph.html)
