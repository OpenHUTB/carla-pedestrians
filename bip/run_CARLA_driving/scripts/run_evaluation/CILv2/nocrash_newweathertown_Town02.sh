#!/bin/bash

# 对训练好的 CIL++ 代理运行 Town02 nocrash benchmark

# 运行carla 0.9.13 docker来作为服务端
nocrash_newweathertown_empty_cilv2 () {
    python ${DRIVING_TEST_ROOT}/driving/evaluator.py \
    --debug=0 \
    --scenarios=${DRIVING_TEST_ROOT}/data/nocrash/nocrash_newweathertown_empty_Town02_lbc.json  \
    --routes=${DRIVING_TEST_ROOT}/data/nocrash \
    --repetitions=1 \
    --resume=True \
    --track=SENSORS \
    --agent=${DRIVING_TEST_ROOT}/driving/autoagents/CILv2_agent.py \
    --checkpoint=${DRIVING_TEST_ROOT}/results/nocrash  \
    --agent-config=${TRAINING_RESULTS_ROOT}/_results/Ours/Town1_2/config45.json \
    --docker=carlasim/carla:0.9.13 \
    --gpus=0 \
    --fps=20 \
    --PedestriansSeed=0 \
    --trafficManagerSeed=0 \
    --save-driving-vision
}


nocrash_newweathertown_regular_cilv2 () {
    python ${DRIVING_TEST_ROOT}/driving/evaluator.py \
    --debug=0 \
    --scenarios=${DRIVING_TEST_ROOT}/data/nocrash/nocrash_newweathertown_regular_Town02_lbc.json  \
    --routes=${DRIVING_TEST_ROOT}/data/nocrash \
    --repetitions=1 \
    --resume=True \
    --track=SENSORS \
    --agent=${DRIVING_TEST_ROOT}/driving/autoagents/CILv2_agent.py \
    --checkpoint=${DRIVING_TEST_ROOT}/results/nocrash  \
    --agent-config=${TRAINING_RESULTS_ROOT}/_results/Ours/Town1_2/config45.json \
    --docker=carlasim/carla:0.9.13 \
    --gpus=0 \
    --fps=20 \
    --PedestriansSeed=0 \
    --trafficManagerSeed=0 \
    --save-driving-vision
}

nocrash_newweathertown_busy_cilv2 () {
    python ${DRIVING_TEST_ROOT}/driving/evaluator.py \
    --debug=0 \
    --scenarios=${DRIVING_TEST_ROOT}/data/nocrash/nocrash_newweathertown_busy_Town02_lbc.json  \
    --routes=${DRIVING_TEST_ROOT}/data/nocrash \
    --repetitions=1 \
    --resume=True \
    --track=SENSORS \
    --agent=${DRIVING_TEST_ROOT}/driving/autoagents/CILv2_agent.py \
    --checkpoint=${DRIVING_TEST_ROOT}/results/nocrash  \
    --agent-config=${TRAINING_RESULTS_ROOT}/_results/Ours/Town1_2/config45.json \
    --docker=carlasim/carla:0.9.13 \
    --gpus=0 \
    --fps=20 \
    --PedestriansSeed=0 \
    --trafficManagerSeed=0 \
    --save-driving-vision
}

# 包含3个CILv2场景：空的场景、常规场景、繁忙场景
# 不同场景是由于传入的场景参数 --scenarios 不一样
# 在运行到 _results/sensor/nocrash_newweathertown_regular_Town02_lbc/Ours_Town1_2_45_Seed0_20FPS/WetSunset_route00006
# 连接不到服务器了
function_array=(
# "nocrash_newweathertown_empty_cilv2"
# "nocrash_newweathertown_regular_cilv2"
"nocrash_newweathertown_busy_cilv2")


# 当 Carla 崩溃的时候恢复基准测试，直到完成基准测试。
RED=$'\e[0;31m'
NC=$'\e[0m'
# 遍历执行数组中的所有基准测试用例
for run in "${function_array[@]}"; do
    PYTHON_RETURN=1
    # python 执行返回1表示不成功，一直需要执行到成功（即返回0）
    until [ $PYTHON_RETURN == 0 ]; do
      # 拿到数组的一个函数名，并执行，运行的结果保存到：_results/sensor/nocrash_newweathertown_{empty,regular,busy}_Town02_lbc/*
      ${run}
      PYTHON_RETURN=$?
      echo "${RED} PYTHON_RETURN=${PYTHON_RETURN}!!! Start Over!!!${NC}" >&2
      sleep 2
    done
    sleep 2
done

echo "Bash script done."