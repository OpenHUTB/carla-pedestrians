# Pedestrians Video2CARLA (vendored)

This directory is a vendored copy of
[pedestrians-video-2-carla](https://github.com/wielgosz-info/pedestrians-video-2-carla)
(Maciej Wielgosz, Antonio M. Lopez, Muhammad Naveed Riaz), used here to bring more
realistic pedestrian motion into CARLA. It is not intended for fully standalone use;
see the [main project README](https://github.com/wielgosz-info/carla-pedestrians) or the
[ARCANE project site](https://project-arcane.eu/) for the wider context.

## Setup

Rename `.env.template` to `.env` and edit the values as required. Run the CARLA
server (optional) and the containers as described in the main project README; this
results in the `carla-pedestrians_video2carla_1` container running. Without an
NVIDIA GPU use the CPU-only `docker-compose.cpu.yml`; note that the CARLA server
itself needs a GPU, so without one the `source_carla` and `carla` renderers must
not be used.

### Using the JAAD dataset

If `JAADOpenPoseDataModule` is needed, convert the annotations inside the container:

```sh
python src/pedestrians_video_2_carla/data/utils/jaad_annotations_xml_2_csv.py
```

The output lands in `/outputs/JAADOpenPoseDataModule`, where the data module looks
for it by default; keypoint files are expected under
`/outputs/JAADOpenPoseDataModule/openpose` and must be produced with OpenPose.
`annotations.csv` requires the columns `video`, `frame`, `x1`, `y1`, `x2`, `y2`,
`id`, `action`, `gender`, `age`, `group_size` and `speed`; only `walking`
pedestrians and `stopped` cars are used, and only fragments with `group_size=1`.

### Conda

Docker is the preferred route. With conda, create the environment and install the
package:

```sh
conda create -f environment.yml
conda activate pedestrians
COMMIT=$(git rev-parse --short HEAD) SETUPTOOLS_SCM_PRETEND_VERSION="0.0.post0.dev38+${COMMIT}.dirty" pip install -e .
pytest tests
```

The conda environment is not actively maintained.

## Running

The full option list is printed by

```sh
python -m pedestrians_video_2_carla --help
```

Data-module and model specific options change when the DataModule or model changes.
An example training run:

```sh
python -m pedestrians_video_2_carla --flow=pose_lifting --mode=train \
  --data_module_name=CarlaRecorded --movements_model_name=LinearAE \
  --batch_size=256 --num_workers=32 --clip_length=16 \
  --data_nodes=CARLA_SKELETON --input_nodes=CARLA_SKELETON --output_nodes=CARLA_SKELETON \
  --max_epochs=500 --loss_modes=loc_2d_3d --renderers none --gpus=0,1 --accelerator=ddp
```

## Reference skeletons

Reference skeleton data in `src/pedestrians_video_2_carla/data/carla/files` are
extracted from the CARLA project *Walkers* `.uasset` files.

## Citation

```
@misc{wielgosz2023carlabsp,
      title={{CARLA-BSP}: a simulated dataset with pedestrians},
      author={Maciej Wielgosz and Antonio M. Lopez and Muhammad Naveed Riaz},
      month={May},
      year={2023},
      eprint={2305.00204},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```

## Licence

Vendored code is MIT. It builds on CARLA (MIT), the JAAD dataset (MIT), OpenPose
(academic / non-commercial use only) and MPI-IS components (non-commercial
scientific research), each under its own licence; see the upstream repository.
