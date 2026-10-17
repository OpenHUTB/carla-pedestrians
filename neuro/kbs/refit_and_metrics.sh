#!/bin/bash
# refit_and_metrics.sh — 回装规范 P2 输出到基座 slam_results/ 并重算 final_metrics
# 口径: slam_results_p2ft/ (P2 定解 A/B 输出) → slam_results/ (论文指标读取处),
#       仅回装 trajectories.mat + experiences.mat (闭环计数来源)。
set -e
cd ../../neuro
for s in Town05Data_IMU_Fusion Town03Data_IMU_Fusion Town07Data_IMU_Fusion \
         Town10HDData_IMU_Fusion; do
  d=data/${s}
  cp ${d}/slam_results_p2ft/trajectories.mat ${d}/slam_results/trajectories.mat
  cp ${d}/slam_results_p2ft/experiences.mat ${d}/slam_results/experiences.mat
  echo "回装 ${s}"
done
d=data/01_NeuroSLAM_Datasets/KITTI07Data_IMU_Fusion
cp ${d}/slam_results_p2ft/trajectories.mat ${d}/slam_results/trajectories.mat
cp ${d}/slam_results_p2ft/experiences.mat ${d}/slam_results/experiences.mat
echo "回装 KITTI07Data_IMU_Fusion"
$HOME/anaconda3/envs/tdmpc/bin/python kbs/compute_final_metrics.py
