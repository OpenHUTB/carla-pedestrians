export ROOTDIR=/home/d/workspace/CILv2_multiview/
cd $ROOTDIR
# export CARLAPATH=$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/:$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
# export CARLAPATH=$ROOTDIR/CARLA_0.9.15/PythonAPI/carla/:$ROOTDIR/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
export CARLAPATH=$ROOTDIR/CARLA_0.9.13/PythonAPI/carla/


# define environment variables
export TRAINING_ROOT=$ROOTDIR
export DRIVING_TEST_ROOT=$TRAINING_ROOT/run_CARLA_driving/
export SCENARIO_RUNNER_ROOT=$TRAINING_ROOT/scenario_runner/
# export PYTHONPATH=$CARLAPATH:$TRAINING_ROOT:$DRIVING_TEST_ROOT:$SCENARIO_RUNNER_ROOT
export PYTHONPATH=$CARLAPATH:$TRAINING_ROOT:$DRIVING_TEST_ROOT:$SCENARIO_RUNNER_ROOT
export TRAINING_RESULTS_ROOT=$ROOTDIR/_results
export DATASET_PATH=$ROOTDIR/_results/datasets
export SENSOR_SAVE_PATH=$ROOTDIR/_results/sensor


# install the required package
# pip --file requirements.txt


# mkdir -p $TRAINING_RESULTS_ROOT/_results
# tar -zxvf _results.tar.gz -C $TRAINING_RESULTS_ROOT/_results/


# 对我们训练好的 CIL++ 模型进行基准测试:
# cd $DRIVING_TEST_ROOT
# ./scripts/run_evaluation/CILv2/nocrash_newweathertown_Town02.sh

# 进行测试
# python main.py --process-type val_only --gpus 0 --folder CILv2 --exp CILv2_3cam_smalltest

# 启动docker
# docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
# sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh
