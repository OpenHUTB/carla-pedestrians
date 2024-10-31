# Pedestrians Video2CARLA

This is a part of the bigger project to bring the more realistic pedestrian movements to CARLA.
It isn't intended for fully standalone use. Please see the [main project README.md](https://github.com/wielgosz-info/carla-pedestrians/blob/main/README.md) or [Adversarial Cases for Autonomous Vehicles (ARCANE) project website](https://project-arcane.eu/) for details.

## Setup

Copy the `.env.template` file to `.env` and edit the values as required.

Run the CARLA server (optional) & the containers with our code as described in [Step 2 of main project README.md](https://github.com/wielgosz-info/carla-pedestrians/blob/main/README.md#Step 2). This should result in the `carla-pedestrians_video2carla_1` container running.

If you don't have the Nvidia GPU, there is CPU-only version available `docker-compose.cpu.yml`.
Please note that currently running CARLA server requires GPU, so without it the `source_carla`
`carla` renderers shouldn't be used, since it would result in errors.

### Using JAAD dataset

If needed (you want to use `JAADOpenPoseDataModule`), inside the container run the
```sh
python src/pedestrians_video_2_carla/data/utils/jaad_annotations_xml_2_csv.py
```
script to convert the JAAD annotations from XML to CSV. The output will be in `/outputs/JAADOpenPoseDataModule` and this is where `JAADOpenPoseDataModule` will look for it by default. Please note that the data module expects to find the keypoints files in `/outputs/JAADOpenPoseDataModule/openpose`, you need to process the dataset with OpenPose to get them.

(For now) the `annotations.csv` file needs to have `video`, `frame`, `x1`,`y1`, `x2`, `y2`, `id`, `action`, `gender`, `age`, `group_size` and `speed` columns, where `x1`,`y1`, `x2`, `y2` define pedestrian bounding box, `id` is the pedestrian id, `action` is what the pedestrian is doing (since right now only the `walking` ones will be used) and `speed` is the car speed category (for now only `stopped` cars will be used). For now we are also only using fragments where `group_size=1`.

### Conda

The preferred way of running is via Docker. If conda is used, in addition to creating the env from the provided `environment.yml` file, following steps need to be done:

1. Create and activate conda environment with the following command:
   
    ```sh
    conda create -f environment.yml
    conda activate pedestrians
    ```
    
2. Install the `pedestrians_video_2_carla` package with:

    ```sh
    COMMIT=$(git rev-parse --short HEAD) SETUPTOOLS_SCM_PRETEND_VERSION="0.0.post0.dev38+${COMMIT}.dirty" pip install -e .
    ```

3. Run `pytest tests` to see if everything is working.

**Please note that conda env is not actively maintained.**

## Running

Full list of options is available by running inside the container:

```sh
python -m pedestrians_video_2_carla --help
```

Please note that data module and model specific options may change if you switch the DataModule or Model.

### Example 'start training' command

```sh
python -m pedestrians_video_2_carla \
  --flow=pose_lifting \
  --mode=train \
  --data_module_name=CarlaRecorded \
  --movements_model_name=LinearAE \
  --batch_size=256 \
  --num_workers=32 \
  --clip_length=16 \
  --data_nodes=CARLA_SKELETON \
  --input_nodes=CARLA_SKELETON \
  --output_nodes=CARLA_SKELETON \
  --max_epochs=500 \
  --loss_modes=loc_2d_3d \
  --renderers none \
  --gpus=0,1 \
  --accelerator=ddp
```

### Example run with rendering

```sh
python -m pedestrians_video_2_carla \
  --flow=autoencoder \
  --mode=train \
  --data_module_name=CarlaRecorded \
  --batch_size=256 \
  --num_workers=32 \
  --data_nodes=CARLA_SKELETON \
  --input_nodes=CARLA_SKELETON \
  --output_nodes=CARLA_SKELETON \
  --loss_modes=loc_2d_3d \
  --gpus=0,1 \
  --accelerator=ddp \
  --log_every_n_steps=16 \
  --renderers source_videos input_points projection_points carla \
  --source_videos_dir=/datasets/CARLA/BinarySinglePedestrian \
  --source_videos_overlay_skeletons \
  \
  --movements_model_name=Seq2SeqEmbeddings \
  --movements_output_type=pose_2d \
  --max_epochs=200 \
  --clip_length=16
```

## Reference skeletons
Reference skeleton data in `src/pedestrians_video_2_carla/data/carla/files` are extracted form [CARLA project Walkers *.uasset files](https://bitbucket.org/carla-simulator/carla-content).

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
Our code is released under [MIT License](https://github.com/wielgosz-info/pedestrians-video-2-carla/blob/main/LICENSE).

This project uses (and is developed to work with) [CARLA Simulator](https://carla.org/), which is released under [MIT License](https://github.com/carla-simulator/carla/blob/master/LICENSE).

This project uses videos and annotations from [JAAD dataset](https://data.nvision2.eecs.yorku.ca/JAAD_dataset/), created by Amir Rasouli, Iuliia Kotseruba, and John K. Tsotsos, to extract pedestrians movements and attributes. The videos and annotations are released under [MIT License](https://github.com/ykotseruba/JAAD/blob/JAAD_2.0/LICENSE).

This project uses [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose), created by Ginés Hidalgo, Zhe Cao, Tomas Simon, Shih-En Wei, Yaadhav Raaj, Hanbyul Joo, and Yaser Sheikh, to extract pedestrians skeletons from videos. OpenPose has its [own licensing](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/LICENSE) (basically, academic or non-profit organization noncommercial research use only).

This project uses software, models and datasets from [Max-Planck Institute for Intelligent Systems](https://is.mpg.de/en), namely [VPoser: Variational Human Pose Prior for Body Inverse Kinematics](https://github.com/nghorbani/human_body_prior), [Body Visualizer](https://github.com/nghorbani/body_visualizer), [Configer](https://github.com/MPI-IS/configer) and [Perceiving Systems Mesh Package](https://github.com/MPI-IS/mesh), which have their own licenses (non-commercial scientific research purposes, see each repo for details). The models can be downloaded from ["Expressive Body Capture: 3D Hands, Face, and Body from a Single Image" website](https://smpl-x.is.tue.mpg.de). Required are the "SMPL-X with removed head bun" or other SMPL-based model that can be fed into [BodyModel](https://github.com/nghorbani/human_body_prior/blob/master/src/human_body_prior/body_model/body_model.py) - right now our code utilizes only [first 22 common SMPL basic joints](https://meshcapade.wiki/SMPL#related-models-the-smpl-family#skeleton-layout). For VPoser, the "VPoser v2.0" model is used. Both downloaded models need to be put in `models` directory. If using other SMPL models, the defaults in `src/pedestrians_video_2_carla/data/smpl/constants.py` may need to be modified. SMPL-compatible datasets can be obtained from [AMASS: Archive of Motion Capture As Surface Shapes](https://amass.is.tue.mpg.de/). Each available dataset has its own license / citing requirements. During the development of this project, we mainly used [CMU](http://mocap.cs.cmu.edu/) and [Human Eva](http://humaneva.is.tue.mpg.de/) SMPL-X Gender Specific datasets.

## Funding

|                                                                                                                              |                                                                                                                      |                                                                                                                                                                                                                                                                                                                                                                                      |
| ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img src="docs/_static/images/logos/Logo Tecniospring INDUSTRY_white.JPG" alt="Tecniospring INDUSTRY" style="height: 24px;"> | <img src="docs/_static/images/logos/ACCIO_horizontal.PNG" alt="ACCIÓ Government of Catalonia" style="height: 35px;"> | <img src="docs/_static/images/logos/EU_emblem_and_funding_declaration_EN.PNG" alt="This project has received funding from the European Union's Horizon 2020 research and innovation programme under Marie Skłodowska-Curie grant agreement No. 801342 (Tecniospring INDUSTRY) and the Government of Catalonia's Agency for Business Competitiveness (ACCIÓ)." style="height: 70px;"> |

<!-- pyscaffold-notes -->

## Note

This project has been set up using PyScaffold 4.0.2. For details and usage
information on PyScaffold see https://pyscaffold.org/.
