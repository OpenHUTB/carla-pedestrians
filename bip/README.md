# CIL++ with Multi-View Attention Learning
-------------------------------------------------------------

 <img src="Driving.gif" height="250">

### Publications
This is the official code release of the paper:

Yi Xiao, Felipe Codevilla, Diego Porres and Antonio M. Lopez. [Scaling Vision-based End-to-End Driving with Multi-View Attention Learning](https://arxiv.org/abs/2302.03198).

Please cite our paper if you find this work useful (will be soon updated with the IROS citation):

         @misc{xiao2023scaling,
         title={Scaling Vision-based End-to-End Driving with Multi-View Attention Learning},
         author={Yi Xiao and Felipe Codevilla and Diego Porres and Antonio M. Lopez},
         year={2023},
         eprint={2302.03198},
         archivePrefix={arXiv},
         primaryClass={cs.CV}
         }

### 视频
Please watch our online [video](https://youtu.be/fY56Gliz_Rw?si=VfXa-_b6TgdVgLZD) for more interesting scenario demonstrations

-------------------------------------------------------------
### 概要

在此存储库中，您可以找到以下材料：

 * 对我们论文中提出的经过训练的 CIL++ 模型进行基准测试
 * 使用 Roach RL 专家驱动程序从 [该论文](https://arxiv.org/abs/2108.08265) 收集数据集
 * 训练/评估（离线）您自己训练过的 CIL++ 模型
 * 在 CARLA 0.9.13 上测试你的模型

-------------------------------------------------------------
### 环境设置

Python 版本: 3.8

Cuda 版本： 11.6

所需要的包: [requirements.txt](https://github.com/yixiao1/CILv2_multiview/blob/main/requirements.txt)

* 为实验设置 conda 环境：

        conda create --name CILv2Env python=3.8
        conda activate CILv2Env

* 下载 [CARLA 0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13/) 到你的根目录并构建 CARLA docker：

        export ROOTDIR=<Path to your root directory>
        cd $ROOTDIR
        export CARLAPATH=$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/:$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg

* 为了使用 CARLA 容器，可使用 1) 拉取或者 2) 构建容器：

    1）拉取官方镜像，运行：

        docker pull carlasim/carla:0.9.13

    2）构建镜像，运行（可选）:

        docker image build -f $ROOTDIR/CARLA_0.9.13/Dockerfile -t carla_0.9.13 $ROOTDIR/CARLA_0.9.13/
    
    启动docker：

        sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh

* 将 CIL++ 存储库下载到您的根目录中：

        cd $ROOTDIR
        git clone https://github.com/yixiao1/CILv2_multiview.git

* 定义环境变量：

        export TRAINING_ROOT=$ROOTDIR/CILv2_multiview
        export DRIVING_TEST_ROOT=$TRAINING_ROOT/run_CARLA_driving/
        export SCENARIO_RUNNER_ROOT=$TRAINING_ROOT/scenario_runner/
        export PYTHONPATH=$CARLAPATH:$TRAINING_ROOT:$DRIVING_TEST_ROOT:$SCENARIO_RUNNER_ROOT
        export TRAINING_RESULTS_ROOT=<Path to the directory where the results to be saved>
        export DATASET_PATH=<Path to the directory where the datasets are stored>
        export SENSOR_SAVE_PATH=<Path to the directory where the driving test frames are stored>

* 安装所需要的包：

        pip install -r requirements.txt
        pip install opencv-python
        pip install tensorflow-gpu==2.11.0

-------------------------------------------------------------
### 对训练好的 CIL++ 进行基准测试

* 下载训练好的 CIL++ 模型 [_results.tar.gz](https://drive.google.com/file/d/1GLo5mVrmyNsb5pLqksYnjR8fN1-ZptHE/view?usp=sharing)
到目录 `TRAINING_RESULTS_ROOT/_results` 下。保存的目录模式应该为 $TRAINING_RESULTS_ROOT/_results/Ours/TownXX/...：

        mkdir -p $TRAINING_RESULTS_ROOT/_results
        tar -zxvf _results.tar.gz -C $TRAINING_RESULTS_ROOT/_results/

* 对训练好的 CIL++ 在未见过的Town02新天气上进行基准测试（连接不上服务器导致失败后，再次运行可继续），包括Town03和未见过的Town05基准测试（只有图片结果，没有量化的性能指标）: 

        cd $DRIVING_TEST_ROOT
        ./scripts/run_evaluation/CILv2/nocrash_newweathertown_Town02.sh

* 参考 [链接](https://openhutb.github.io/carla_doc/scenario_runner/agent_evaluation) 对代理进行评估（调用`run_CARLA_driving\driving\evaluator.py`），结果保存到`run_CARLA_driving/results/leaderboard/leaderboard_Town05/Ours_Town12346_5_40_Seed0_20FPS.json`：
      
        ./scripts/run_evaluation/CILv2/leaderboard_Town05.sh

注意：如果已经验证完成，再次运行会报错：`_timer.cancel() AttributeError: 'NoneType' object has no attribute 'cancel'`，不影响。

-------------------------------------------------------------
### 使用 Roach RL expert 进行数据收集

为了训练模型，你可以

* 下载我们收集好的数据集：

    为了方便下载，文件夹被分成几个部分，并全部压缩在 zip 文件中。可以直接从 [百度网盘链接](https://pan.baidu.com/s/1j2mnbCZIxdOrzNDtAr4NBg?pwd=hutb) 下载下面说明的数据。
    对于模型训练，请将它们解压到您的 `DATASET_PATH` 中。数据加载器 dataloader 将访问 $DATASET_PATH/<dataset_folder_name> 的完整路径，其中 `<dataset_folder_name>` 将由 [exp yaml 文件](https://github.com/yixiao1/CILv2_multiview/blob/main/configs/CILv2/CILv2_3cam_smalltest.yaml) 中的 `TRAIN_DATASET_NAME`/`VALID_DATASET_NAME` 定义

    * 小测试数据集：

        训练： [part 1](http://datasets.cvc.uab.es/CILv2/smalltrain1.tar.gz), [part 2](http://datasets.cvc.uab.es/CILv2/smalltrain2.tar.gz)

        离线评估： [part 3](http://datasets.cvc.uab.es/CILv2/smallval1.tar.gz)


    * 单车道城镇：

        训练： [part 4](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_LBC_3cam.tar.gz),
        [part 5](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_NoCrash_3cam.tar.gz)

        离线评估： [part 6](http://datasets.cvc.uab.es/CILv2/Roach_LBCRoutes_3cam_valid.tar.gz)

    * 多车道城镇：

        训练：
        [part 7](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T1_3cam.tar.gz),
        [part 8](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T1_dense_3cam.tar.gz),
        [part 9](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T2_3cam.tar.gz),
        [part 10](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T3_3cam.tar.gz),
        [part 11](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T4_3cam.tar.gz),
        [part 12](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T4_dense_3cam.tar.gz),
        [part 13](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T6_3cam.tar.gz),
        [part 14](http://datasets.cvc.uab.es/CILv2/Roach_carla0913_fps10_dense_normalcamera_T6_dense_3cam.tar.gz)

        离线评估： 和 part 6 一样

* 收集新的数据集。我们用于数据收集的 RL 专家驱动程序来自[这项工作](https://github.com/zhejz/carla-roach)

-------------------------------------------------------------
### 在新训练的 CIL++ 模型上 训练并执行离线评估

* 你需要定义一个用于训练的配置文件。请参考 `configs` 文件夹中的 [此文件](https://github.com/yixiao1/CILv2_multiview/blob/main/configs/CILv2/CILv2_3cam_smalltest.yaml) 作为示例

* 运行 main.py 文件：

        python main.py --process-type train_val --gpus 0 --folder CILv2 --exp CILv2_3cam_smalltest

        python main.py --process-type train_val --gpus 0 --folder CILv2 --exp CILv2_3cam_single_lane

        python main.py --process-type train_val --gpus 0 --folder CILv2 --exp CILv2_3cam_multi_lane

    这里 `--process-type` 定义处理类型（可以是 train_val 或者 val_only）, `--gpus` 定义使用的 gpus，
    `--folder` 是 [配置文件夹名](https://github.com/yixiao1/CILv2_multiview/tree/main/configs/CILv2),
    并且 `--exp` 是 [配置 yaml 文件名](https://github.com/yixiao1/CILv2_multiview/blob/main/configs/CILv2/CILv2_3cam_smalltest.yaml) 。
    你的结果将保存在 `$TRAINING_RESULTS_ROOT/_results/<folder_name>/<exp_name>/` 。
    热力图位于 `$TRAINING_RESULTS_ROOT/_results/CILv2/CILv2_3cam_smalltest/Eval/Valid_gradCAM_smallval1/30/-1`

-------------------------------------------------------------
### 在 CARLA 模拟器上测试你自己训练的模型

* 请确保您的模型以与下载的 CIL++ 模型相同的正确模式保存： 

        cd $TRAINING_RESULTS_ROOT/_results/<folder_name>/<exp_name>/

    其中 `folder_name` 是实验文件夹名称，`exp_name` 是配置文件名。
    您的模型都保存在 `./checkpoints/`

* 定义基准测试的配置文件：

        cd $TRAINING_RESULTS_ROOT/_results/<folder_name>/<exp_name>
        > config45.json

    在json文件中，需要定义要测试的模型/检查点： 

            {
                "agent_name": "CILv2",
                "checkpoint": 45,
                "yaml": "CILv2.yaml"
            }
    其中 `checkpoint` 表示需要测试的检查点，`yaml` 是训练配置文件，在训练过程中自动生成，请参考下载的 [_results.tar.gz](https://drive.google.com/file/d/1GLo5mVrmyNsb5pLqksYnjR8fN1-ZptHE/view?usp=sharing) 中的 json 文件

* 对你的模型进行基准测试：

    请注意，为了对您自己训练的模型进行基准测试，您需要通过更改 `--agent-config` 来修改 [脚本](https://github.com/yixiao1/CILv2_multiview/blob/main/run_CARLA_driving/scripts/run_evaluation/CILv2/nocrash_newweathertown_Town02_lbc.sh) 

        cd $DRIVING_TEST_ROOT
        run ./scripts/run_evaluation/CILv2/nocrash_newweathertown_Town02.sh

-------------------------------------------------------------

### 问题
- 执行`docker pull`报找不到服务器的错：Error response from daemon: Get "https://registry-1.docker.io/v2/": net/http:

解决办法：
```shell
touch /etc/docker/daemon.json
chmod 777 -R /etc/docker/daemon.json
vi /etc/docker/daemon.json
```
添加内容：
```text
{
  "registry-mirrors": ["https://docker-proxy.741001.xyz","https://registry.docker-cn.com"]
}
```
```shell
systemctl restart docker
systemctl daemon-reload
systemctl restart docker
systemctl daemon-reload
```

- 安装nvidia docker
```shell
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker.service
```

- 测试 pytorch 的 GPU 是否可用：
```shell
import torch
print(torch.__version__)
print(torch.cuda.is_available())
```

- 根据进程名杀死所有 Carla 服务进程：
```shell
killall CarlaUE4-Linux-Shipping
```

- ImportError: libtiff.so.5: cannot open shared object file: No such file or directory
```shell
ll /usr/lib/x86_64-linux-gnu/libtiff.so
# /usr/lib/x86_64-linux-gnu/libtiff.so -> libtiff.so.6.0.1
sudo apt install libtiff5-dev
```

- 单车道城镇可以，多车道城镇训练报错：
```text
  File "/home/d/workspace/CILv2_multiview/dataloaders/transforms.py", line 113, in encode_directions_4
    raise ValueError("Unexpcted direction identified %s" % str(directions))
ValueError: Unexpcted direction identified 5.0
```
解决：将`configs/CILv2/CILv2_3cam_multi_lane.yaml`中的`DATA_COMMAND_CLASS_NUM`配置改为 6。

-------------------------------------------------------------
### License
The code is released under a CC-BY-NC 4.0 license, which only allows personal and research use.
For a commercial license, please contact the authors. Portions of source code taken from external sources
are annotated with links to original files and their corresponding licenses.

-------------------------------------------------------------
### Acknowledgements
 <img src="logo.png" height="100">

 This research is supported as a part of the project TED2021-132802B-I00 funded by MCIN/AEI/10.13039/501100011033 and the European Union NextGenerationEU/PRTR.

 Yi Xiao acknowledges the support to her PhD study provided by the Chinese Scholarship Council (CSC), Grant No.201808390010. Diego Porres acknowledges the support to his PhD study provided by Grant PRE2018-083417 funded by MCIN/AEI /10.13039/501100011033 and FSE invierte en tu futuro. Antonio M. López acknowledges the financial support to his general research activities given by ICREA under the ICREA Academia Program. Antonio thanks the synergies, in terms of research ideas, arising from the project PID2020-115734RB-C21 funded by MCIN/AEI/10.13039/501100011033.

 The authors acknowledge the support of the Generalitat de Catalunya CERCA Program and its ACCIO agency to CVC’s general activities.





