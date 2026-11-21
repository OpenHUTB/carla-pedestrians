#!/bin/bash
# ============================================================================
# grid_search_gates.sh — HART 门控常数坐标式网格搜索 (导师批准 2026-11-16)
#
# 前置: hart_transformer_extractor.m 已改为 getenv 读门控
#       (HART_INPUT_GATE / HART_FORGET_GATE / HART_OUTPUT_GATE, 缺省 0.5/0.5/0.9)
# 用法: bash grid_search_gates.sh TOWN03 [PARALLEL]
#       PARALLEL=1 串行(默认); PARALLEL=2 双 MATLAB 并行(与 matlab_batch 纪律一致)
#
# 逻辑: 坐标式扫描——每次只动一个轴(11 档 0:0.1:1), 其余轴保持当前最优,
#       共 33 组合; 每组合跑一次 RERUN_<场景>, 从日志提取末段 RMSE,
#       汇总到 grid_search_results.csv, 最后按轴打印最优值。
# ============================================================================
SCENE="${1:-TOWN03}"
PARALLEL="${2:-1}"
AXES=(INPUT FORGET OUTPUT)
VALS=(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)
DEFAULTS=(0.5 0.5 0.9)   # INPUT FORGET OUTPUT 当前基线
CSV="$(cd "$(dirname "$0")" && pwd)/grid_search_results.csv"
echo "axis,value,rmse,log" > "$CSV"

get_axis_idx() { case "$1" in INPUT) echo 0;; FORGET) echo 1;; OUTPUT) echo 2;; esac; }

set_gates() {  # $1=axis  $2=value
  case "$1" in
    INPUT)  export HART_INPUT_GATE="$2";;
    FORGET) export HART_FORGET_GATE="$2";;
    OUTPUT) export HART_OUTPUT_GATE="$2";;
  esac
}

run_one() {  # $1=axis $2=value
  local axis="$1" val="$2"
  if grep -q "^$axis,$val,[0-9]" "$CSV" 2>/dev/null; then
    echo "[GS] 跳过已完成 $axis=$val"
    return
  fi
  local tag="${axis}_${val}"
  local log="/tmp/gs_${SCENE}_${tag//./_}.log"
  echo "[GS] 跑 $axis=$val ..."
  ~/matlab_batch.sh "RERUN_${SCENE}" > "$log" 2>&1
  local rc=$?
  # 解析 NLM(经验地图)行 RMSE: 从 performance_report.txt 的 "Experience Map:" 块提取
  local rmse
  local report="$(cd "$(dirname "$0")/../.." && pwd)/data/01_NeuroSLAM_Datasets/${SCENE}Data_IMU_Fusion/slam_results/performance_report.txt"
  cp "$report" "/tmp/gs_report_${SCENE}_${tag//./_}.txt" 2>/dev/null  # 组合报告快照(事后取证)
  rmse=$(grep -A6 "Experience Map:" "$report" 2>/dev/null | grep -m1 "RMSE" | grep -oE "[0-9]+\.[0-9]+")
  if [ -z "$rmse" ]; then
    # 回退: 日志末段 RMSE (9b 设计初衷; 报告缺块/时序异常时兜底)
    rmse=$(grep "RMSE:" "$log" 2>/dev/null | tail -1 | grep -oE "[0-9]+\.[0-9]+")
    echo "[GS]   (报告无 Experience 块, 用日志末段 RMSE=$rmse)"
  fi
  echo "$axis,$val,$rmse,$log" >> "$CSV"
  echo "[GS] $axis=$val → RMSE=${rmse:-解析失败} (exit=$rc)"
}

pids=()
for axis in "${AXES[@]}"; do
  for v in "${VALS[@]}"; do
    set_gates "$axis" "$v"
    if [ "$PARALLEL" = "2" ]; then
      run_one "$axis" "$v" & pids+=($!)
      while [ "$(jobs -rp | wc -l)" -ge 2 ]; do wait -n; done
    else
      run_one "$axis" "$v"
    fi
  done
done
wait

# 按轴汇总最优
echo "==== 各轴最优 ===="
for axis in "${AXES[@]}"; do
  awk -F, -v a="$axis" '$1==a && $3!="" && $3!="解析失败" {print $3, $2}' "$CSV" \
    | sort -n | head -1 | awk -v a="$axis" '{print a" 最优: "$2" (RMSE="$1")"}'
done
echo "完整表见 $CSV"
