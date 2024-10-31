# CARLA 行人
为 CARLA 带来更加逼真的行人运动。

该项目是 [自动驾驶汽车对抗案例项目](https://project-arcane.eu/) 的一部分。

## 克隆
当没有其他选项获取代码（不是可通过 pip 安装的代码）时，此项目包含子模块。因此，为了确保所有模型都能正确运行，请使用以下命令进行克隆：

```sh
git clone --recurse-submodules https://github.com/OpenHUTB/carla-pedestrians.git
```

## 运行步骤

### 第 0 步
将`openpose`,`pedestrians-common`,`pedestrians-video-2-carla`,`pedestrians-scenarios`文件夹中的每个`.env.template`复制成一个新的文件`.env`，并调整变量，尤其是数据集的路径（例如，对于数据集根目录`VIDEO2CARLA_DATASETS_PATH=/datasets`，预期结构为`/datasets/JAAD`,`/datasets/PIE`等）。

注意：用默认的目录会报错，需要把`.env`中的路径都改成`./datasets`，否则会报错：`ERROR: Named volume "datasets:/datasets:ro" is used in service "openpose" but no declaration was found in the volumes section.`。

### 第 1 步
使用 `openpose/docker-compose.yml` 中指定的容器通过 [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) 从视频片段中提取行人骨架：

```sh
cd openpose
docker-compose -f "docker-compose.yml" --env-file .env up -d --build
docker exec -it carla-pedestrians_openpose_1 /bin/bash
```

下载 [行人意图估计PIE数据集](https://data.nvision2.eecs.yorku.ca/PIE_dataset/) 。

容器内部（查看/修改 `extract_poses_from_dataset.sh` 之后）：
```sh
cd /app
./extract_poses_from_dataset.sh
```

生成的文件将保存在 `carla-pedestrians_outputs` Docker 卷中。默认情况下，`extract_poses.sh` 脚本会尝试使用 `JAAD` 数据集。


### 第 2 步
使用我们的代码运行 CARLA 服务器和容器。为方便起见，提供了一个 `compose-up.sh` 脚本，它将来自子模块的多个 `docker-compose.yml` 文件汇集在一起​​并设置通用环境变量。

当使用 NVIDIA GPU 和某些类 UNIX 系统时，您只需运行：
```sh
./compose-up.sh
```

使用 CPU 时，需要指定 `PLATFORM=cpu` 或修改脚本。此外，在 MacOS 上使用 Docker Desktop 时，默认的 GROUP_ID 和 SHM_SIZE 将不起作用，因此需要手动设置。在 MacOS 上运行的结果示例命令是：
```sh
PLATFORM=cpu GROUP_ID=1000 SHM_SIZE=2147483648 ./compose-up.sh
```

有关运行每个单独容器的详细信息，请参阅相关的 `README.md` 文件：
- [pedestrians-video-2-carla](https://github.com/wielgosz-info/pedestrians-video-2-carla/blob/main/README.md)
- [pedestrians-scenarios](https://github.com/wielgosz-info/pedestrians-scenarios/blob/main/README.md)

要快速关闭 `carla-pedestrians` 项目中的所有容器，请使用：

```sh
docker-compose down --remove-orphans
```

## 参考骨架
`pedestrians-video-2-carla/src/pedestrians_video_2_carla/data/carla/files` 中的参考骨架数​​据是从 [CARLA 项目 Walkers *.uasset 文件](https://bitbucket.org/carla-simulator/carla-content) 中提取的。


## Cite
If you use this repo please cite:

```
@misc{wielgosz2023carlabsp,
      title={{CARLA-BSP}: a simulated dataset with pedestrians}, 
      author={Maciej Wielgosz and Antonio M. López and Muhammad Naveed Riaz},
      month={May},
      year={2023},
      eprint={2305.00204},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```

## License
Our code is released under [MIT License](https://github.com/wielgosz-info/carla-pedestrians/blob/main/LICENSE).

The most up-to-date third-party info can be found in the submodules repositories, but here is a non-exhaustive list:

This project uses (and is developed to work with) [CARLA Simulator](https://carla.org/), which is released under [MIT License](https://github.com/carla-simulator/carla/blob/master/LICENSE).

This project uses videos and annotations from [JAAD dataset](https://data.nvision2.eecs.yorku.ca/JAAD_dataset/), created by Amir Rasouli, Iuliia Kotseruba, and John K. Tsotsos, to extract pedestrians movements and attributes. The videos and annotations are released under [MIT License](https://github.com/ykotseruba/JAAD/blob/JAAD_2.0/LICENSE).

This project uses [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose), created by Ginés Hidalgo, Zhe Cao, Tomas Simon, Shih-En Wei, Yaadhav Raaj, Hanbyul Joo, and Yaser Sheikh, to extract pedestrians skeletons from videos. OpenPose has its [own licensing](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/LICENSE) (basically, academic or non-profit organization noncommercial research use only).

This project uses software, models and datasets from [Max-Planck Institute for Intelligent Systems](https://is.mpg.de/en), namely [VPoser: Variational Human Pose Prior for Body Inverse Kinematics](https://github.com/nghorbani/human_body_prior), [Body Visualizer](https://github.com/nghorbani/body_visualizer), [Configer](https://github.com/MPI-IS/configer) and [Perceiving Systems Mesh Package](https://github.com/MPI-IS/mesh), which have their own licenses (non-commercial scientific research purposes, see each repo for details). The models can be downloaded from ["Expressive Body Capture: 3D Hands, Face, and Body from a Single Image" website](https://smpl-x.is.tue.mpg.de). Required are the "SMPL-X with removed head bun" or other SMPL-based model that can be fed into [BodyModel](https://github.com/nghorbani/human_body_prior/blob/master/src/human_body_prior/body_model/body_model.py) - right now our code utilizes only [first 22 common SMPL basic joints](https://meshcapade.wiki/SMPL#related-models-the-smpl-family#skeleton-layout). For VPoser, the "VPoser v2.0" model is used. Both downloaded models need to be put in `pedestrians-video-2-carla/models` directory. If using other SMPL models, the defaults in `pedestrians-video-2-carla/src/pedestrians_video_2_carla/data/smpl/constants.py` may need to be modified. SMPL-compatible datasets can be obtained from [AMASS: Archive of Motion Capture As Surface Shapes](https://amass.is.tue.mpg.de/). Each available dataset has its own license / citing requirements. During the development of this project, we mainly used [CMU](http://mocap.cs.cmu.edu/) and [Human Eva](http://humaneva.is.tue.mpg.de/) SMPL-X Gender Specific datasets.


## Funding

|                                                                                                                              |                                                                                                                      |                                                                                                                                                                                                                                                                                                                                                                                      |
| ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img src="docs/_static/images/logos/Logo Tecniospring INDUSTRY_white.JPG" alt="Tecniospring INDUSTRY" style="height: 24px;"> | <img src="docs/_static/images/logos/ACCIO_horizontal.PNG" alt="ACCIÓ Government of Catalonia" style="height: 35px;"> | <img src="docs/_static/images/logos/EU_emblem_and_funding_declaration_EN.PNG" alt="This project has received funding from the European Union's Horizon 2020 research and innovation programme under Marie Skłodowska-Curie grant agreement No. 801342 (Tecniospring INDUSTRY) and the Government of Catalonia's Agency for Business Competitiveness (ACCIÓ)." style="height: 70px;"> |

