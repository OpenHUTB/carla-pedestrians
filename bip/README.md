# CIL++ with Multi-View Attention Learning (vendored)

This directory is a vendored copy of [CILv2_multiview](https://github.com/yixiao1/CILv2_multiview)
by Yi Xiao, Felipe Codevilla, Diego Porres and Antonio M. Lopez, kept here because the
CARLA-based baselines in this repository are built on top of it. Upstream code and
licence are unchanged; the notes below summarise upstream usage and add the
repository-specific paths used here.

## Publications

Yi Xiao, Felipe Codevilla, Diego Porres and Antonio M. Lopez.
*Scaling Vision-based End-to-End Driving with Multi-View Attention Learning*,
[arXiv:2302.03198](https://arxiv.org/abs/2302.03198).

```
@misc{xiao2023scaling,
      title={Scaling Vision-based End-to-End Driving with Multi-View Attention Learning},
      author={Yi Xiao and Felipe Codevilla and Diego Porres and Antonio M. Lopez},
      year={2023},
      eprint={2302.03198},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```

## Video

Online demonstrations: <https://youtu.be/fY56Gliz_Rw>

## What is in this directory

* benchmark the trained CIL++ model proposed upstream;
* collect datasets with the Roach RL expert driver from
  [arXiv:2108.08265](https://arxiv.org/abs/2108.08265);
* train / evaluate (offline) your own CIL++ models;
* test models on CARLA 0.9.13.

## Environment setup

Python 3.8, CUDA 11.6, packages from
[requirements.txt](https://github.com/yixiao1/CILv2_multiview/blob/main/requirements.txt).

```sh
conda create --name CILv2Env python=3.8
conda activate CILv2Env
```

Download CARLA 0.9.13 into your root directory and export the paths:

```sh
export ROOTDIR=<path to your root directory>
cd $ROOTDIR
export CARLAPATH=$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/:$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
```

The CARLA container can be pulled or built:

```sh
docker pull carlasim/carla:0.9.13
# or
docker image build -f $ROOTDIR/CARLA_0.9.13/Dockerfile -t CARLA0913 $ROOTDIR/CARLA_0.9.13/
```

Then clone the upstream repository and define the environment variables:

```sh
cd $ROOTDIR
git clone https://github.com/yixiao1/CILv2_multiview.git
export TRAINING_ROOT=$ROOTDIR/CILv2_multiview
export DRIVING_TEST_ROOT=$TRAINING_ROOT/run_CARLA_driving/
export SCENARIO_RUNNER_ROOT=$TRAINING_ROOT/scenario_runner/
export PYTHONPATH=$CARLAPATH:$TRAINING_ROOT:$DRIVING_TEST_ROOT:$SCENARIO_RUNNER_ROOT
export TRAINING_RESULTS_ROOT=<path where results are written>
export DATASET_PATH=<path where datasets are stored>
export SENSOR_SAVE_PATH=<path where driving-test frames are stored>
conda install --file requirements.txt
```

## Benchmarking the trained CIL++ model

Download the upstream trained models (`_results.tar.gz`) into
`$TRAINING_RESULTS_ROOT/_results` so that the layout is
`$TRAINING_RESULTS_ROOT/_results/Ours/TownXX/...`, then

```sh
cd $DRIVING_TEST_ROOT
./scripts/run_evaluation/CILv2/nocrash_newweathertown_Town02.sh
```

Benchmarking on the unseen Town02 weather, Town03 and the unseen Town05 runs the
same way; a failed run can simply be restarted (the evaluation resumes). Note that
re-running an already validated evaluation raises
`_timer.cancel() AttributeError: 'NoneType' object has no attribute 'cancel'`,
which is harmless. Agent evaluation is described in the
[HUTB CARLA docs](https://openhutb.github.io/carla_doc/scenario_runner/agent_evaluation).

## Dataset collection with the Roach RL expert

Collected datasets are published upstream in several parts (`smalltrain1/2`,
`smallval1`, `Roach_carla0913_fps10_dense_normalcamera_*`), and additionally
mirrored on Baidu Netdisk: <https://pan.baidu.com/s/1j2mnbCZIxdOrzNDtAr4NBg?pwd=hutb>.
Unpack them into `$DATASET_PATH`; the dataloader reads
`$DATASET_PATH/<dataset_folder_name>`, where `<dataset_folder_name>` is set by
`TRAIN_DATASET_NAME` / `VALID_DATASET_NAME` in the experiment yaml file.

## Training and offline evaluation

Define a configuration file (see `configs/CILv2/CILv2_3cam_smalltest.yaml`) and run

```sh
python main.py --process-type train_val --gpus 0 --folder CILv2 --exp CILv2_3cam_smalltest
```

`--process-type` is `train_val` or `val_only`, `--folder` is the configuration
folder, `--exp` the yaml file name. Results go to
`$TRAINING_RESULTS_ROOT/_results/<folder_name>/<exp_name>/`.

## Testing your own trained models on CARLA

Models must be saved in the same layout as the downloaded ones, under
`./checkpoints/` of the experiment directory. Then define a per-checkpoint json:

```json
{
    "agent_name": "CILv2",
    "checkpoint": 45,
    "yaml": "CILv2.yaml"
}
```

and point the evaluation script at it with `--agent-config`.

## Troubleshooting

* `ModuleNotFoundError: No module named 'resource'` on Windows — `resource` is a
  Unix-only module.
* `TypeError: load() missing 1 required positional argument: 'Loader'` — pin
  `pyyaml` back to 5.4.1.
* `docker pull` cannot reach the registry — configure a mirror, or build the
  image locally as shown above.
* Multi-lane towns fail to train while single-lane towns work — raise
  `DATA_COMMAND_CLASS_NUM` in `configs/CILv2/CILv2_3cam_multi_lane.yaml` to 6.
* To stop all CARLA service processes, kill them by process name.

## Licence

Upstream code is released under CC-BY-NC 4.0 (personal and research use only);
see the upstream repository for commercial licensing and for the licences of
third-party code, `scenario_runner` and `pytorch_grad_cam` included here.
