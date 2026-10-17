#!/bin/bash
# run_scale_nlm_ab.sh — 尺度线 NLM 层 A/B (4 CARLA 集)
# 口径: 深度尺度头 replay 变体 (fp_head030, db=0.30) 作为 NLM 的 EKF-odo 基座,
#       P2S_*.m 驱动 (RESULT_SUBDIR=slam_results_scale_nlm, 不碰规范 p2ft 目录),
#       每集跑完立即恢复活体 fusion_pose (基线文件必须原样归还)。
# 前置: replay_depth_ab.py --depth-file depth_scale_head.txt --deadband 0.30 --tag head030
#       已把变体存到 _p11_backup_20261014/<set>/fp_head030.txt
set -e
cd ../../neuro
DATA=data
BK=data/_p11_backup_20261014
M=$HOME/下载/MATLAB/matlab_2021b_linux_clean/matlab_2021b/bin/matlab
QS=07_test/test_imu_visual_slam/quickstart
for s in Town03 Town05 Town07 Town10HD; do
  d=${DATA}/${s}Data_IMU_Fusion
  echo "########## ${s} ##########"
  cp ${d}/fusion_pose.txt ${BK}/${s}/fp_live_nlm.txt
  cp ${BK}/${s}/fp_head030.txt ${d}/fusion_pose.txt
  (cd ${QS} && ${M} -batch "P2S_${s}" 2>&1 | grep -E "A-fix|完成" | tail -2)
  cp ${BK}/${s}/fp_live_nlm.txt ${d}/fusion_pose.txt
done
echo "SCALE_NLM_AB_DONE"
