#!/bin/bash
# nlm_sweep_newfoot.sh — 驱动 NLM_SWEEP_NEWFOOT_ONE 全网格 (每集一条串行链, 两集并行)
# 用法: nohup bash nlm_sweep_newfoot.sh <Town01Data_IMU_Fusion|Town10HDData_IMU_Fusion> > /tmp/xxx.log 2>&1 &
cd /home/yangrb/openhutb/neuro/07_test/test_imu_visual_slam/quickstart
MATLAB="/home/yangrb/下载/MATLAB/matlab_2021b_linux_clean/matlab_2021b/bin/matlab"
DS="$1"
if [ -z "$DS" ]; then echo "用法: $0 <dataset>"; exit 1; fi

n=0
for vt in 0.04 0.06 0.08 0.10 0.12; do
  for ce in 5 10 20 50; do
    n=$((n+1))
    echo ""
    echo "############ [${n}/20] ${DS} VT=${vt} CE=${ce}  $(date '+%m-%d %H:%M:%S') ############"
    SWEEP_DS="$DS" SWEEP_VT="$vt" SWEEP_CE="$ce" "$MATLAB" -batch NLM_SWEEP_NEWFOOT_ONE 2>&1 \
      | grep -E "SAVED|FAILED|RMSE|闭环数|DIAG\]|VT阈值|错误|error" | tail -12
  done
done
echo "=== ${DS} SWEEP ALL DONE $(date '+%m-%d %H:%M:%S') ==="
