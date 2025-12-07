#!/bin/bash

# 对经过训练的 CIL++ 代理运行 Town05 排行榜驾驶测试

ROOTDIR=/home/d/workspace/CILv2_multiview/
CARLAPATH=/home/d/workspace/CILv2_multiview/CARLA_0.9.13/PythonAPI/carla/

# define environment variables
TRAINING_ROOT=/home/d/workspace/CILv2_multiview/
DRIVING_TEST_ROOT=/home/d/workspace/CILv2_multiview/run_CARLA_driving/
SCENARIO_RUNNER_ROOT=/home/d/workspace/CILv2_multiview/scenario_runner/
# PYTHONPATH=/home/d/workspace/CILv2_multiview/CARLA_0.9.13/PythonAPI/carla/
TRAINING_RESULTS_ROOT=/home/d/workspace/CILv2_multiview/_results
DATASET_PATH=/home/d/workspace/CILv2_multiview/_results/datasets
SENSOR_SAVE_PATH=/home/d/workspace/CILv2_multiview/_results/sensor

PYTHONPATH=/home/d/workspace/CILv2_multiview/scenario_runner/


leaderboard_Town05_cilv2 () {
    python ${DRIVING_TEST_ROOT}/driving/evaluator.py \
    --debug=0 \
    --scenarios=${DRIVING_TEST_ROOT}/data/leaderboard/leaderboard_Town05.json  \
    --routes=${DRIVING_TEST_ROOT}/data/leaderboard \
    --repetitions=1 \
    --resume=True \
    --track=SENSORS \
    --agent=${DRIVING_TEST_ROOT}/driving/autoagents/CILv2_agent.py \
    --checkpoint=${DRIVING_TEST_ROOT}/results/leaderboard  \
    --agent-config=${TRAINING_RESULTS_ROOT}/_results/Ours/Town12346_5/config40.json \
    --docker=carlasim/carla:0.9.13 \
    --gpus=0 \
    --fps=20 \
    --PedestriansSeed=0 \
    --trafficManagerSeed=0 \
    --save-driving-vision
}


function_array=("leaderboard_Town05_cilv2")

# 如果 Carla 崩溃，则恢复基准测试，直到基准测试完成
RED=$'\e[0;31m'
NC=$'\e[0m'
for run in "${function_array[@]}"; do
    PYTHON_RETURN=1
    until [ $PYTHON_RETURN == 0 ]; do
      ${run}
      PYTHON_RETURN=$?
      echo "${RED} PYTHON_RETURN=${PYTHON_RETURN}!!! Start Over!!!${NC}" >&2
      sleep 2
    done
    sleep 2
done

echo "Bash script done."