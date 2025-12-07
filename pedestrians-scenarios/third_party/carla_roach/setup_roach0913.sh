#!/bin/sh
export CUR_DIR=$(pwd)
# For getting data
# TDA_Latest
export COIL_DATASET_PATH=${CUR_DIR}/datasets/CARLA/
export COIL_SYNTHETIC_DATASET_PATH=${CUR_DIR}/datasets/CARLA/
export COIL_REAL_DATASET_PATH=${CUR_DIR}/datasets/CARLA/
#export CARLA_ROOT=/home/dporres/Documents/TDA_Latest
export CARLA_ROOT=${CUR_DIR}/carla
#export CARLA_ROOT=/home/dporres/Documents/TDA_v13_1/
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/:${PYTHONPATH}

export PYTHONPATH=${CUR_DIR}:${PYTHONPATH}
export PYTHONPATH=${CUR_DIR}/agents/:${PYTHONPATH}
