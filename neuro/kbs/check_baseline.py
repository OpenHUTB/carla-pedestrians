#!/usr/bin/env python3
"""kbs/check_baseline.py — NLM 基线守护 (2026-10-17 用户约束: 原本性能不得弱化)

约束: 任何算法改动后, 五场景 × 双口径 (Sim(3)/SE(3)) 全部指标不得劣于
冻结基线 (容差 0.011, 与 se3_metrics.py 交叉校验口径一致)。
任一指标劣化 → exit 1 (不合入)。

用法:
  python3 check_baseline.py                 # 重算并与冻结基线比对
  python3 check_baseline.py --freeze        # 显式冻结当前重算值为新基线 (需慎用)
  python3 check_baseline.py --offline       # 不重算, 仅校验仓内 JSON 与冻结基线一致

数据源: ../data/01_NeuroSLAM_Datasets/<set>/slam_results/ (论文规范目录)
冻结基线: baseline_frozen.json (随仓库提交, 手工修改无效——重算即覆盖)
"""
import json, os, sys, subprocess

# 锚定到脚本所在目录 (相对路径以 kbs/ 为基准)
os.chdir(os.path.dirname(os.path.abspath(__file__)))

TOL = 0.011  # 与 se3_metrics.py 交叉校验同口径
FROZEN = 'baseline_frozen.json'

def recompute():
    """重算双口径全部指标 (调用两个指标入口, 输出即 current JSON)"""
    r1 = subprocess.run([sys.executable, 'compute_final_metrics.py'],
                        capture_output=True, text=True)
    if r1.returncode != 0:
        print(r1.stdout[-2000:]); print(r1.stderr[-2000:])
        sys.exit('重算 final_metrics 失败')
    r2 = subprocess.run([sys.executable, 'se3_metrics.py'],
                        capture_output=True, text=True)
    if r2.returncode != 0:
        print(r2.stdout[-2000:]); print(r2.stderr[-2000:])
        sys.exit('重算 se3_metrics 失败')

def collect():
    fm = json.load(open('final_metrics.json'))['table1']
    sm = json.load(open('se3_metrics.json'))
    cur = {}
    for sd, v in fm.items():
        for cfg, key in [('NLM', 'nlm'), ('EKF', 'ekf'), ('VO', 'vo')]:
            cur[f'{sd}/{cfg}/sim3_rmse'] = v[key]['rmse']
            cur[f'{sd}/{cfg}/sim3_drift'] = v[key]['drift']
    for sd, row in sm.items():
        for cfg in ['NLM-全系统', 'NLM-闭环线', 'EKF', 'VO', 'EKF+尺度', 'ORB-only']:
            if cfg in row and 'se3_rmse' in row[cfg]:
                cur[f'{sd}/{cfg}/se3_rmse'] = row[cfg]['se3_rmse']
                cur[f'{sd}/{cfg}/se3_drift'] = row[cfg]['se3_drift']
                cur[f'{sd}/{cfg}/sim3_rmse'] = row[cfg]['sim3_rmse']
    return cur

def main():
    offline = '--offline' in sys.argv
    freeze = '--freeze' in sys.argv

    if freeze:
        recompute()
        cur = collect()
        json.dump(cur, open(FROZEN, 'w'), indent=1, ensure_ascii=False, sort_keys=True)
        print(f'已冻结 {len(cur)} 项指标为基线 → {FROZEN}')
        return

    if not offline:
        recompute()
    cur = collect()

    if not os.path.exists(FROZEN):
        sys.exit(f'缺少冻结基线 {FROZEN} — 首次请运行: python3 check_baseline.py --freeze')
    ref = json.load(open(FROZEN))

    # 覆盖检查: cur 必须是 ref 的超集 (指标键不得丢失)
    missing = [k for k in ref if k not in cur]
    if missing:
        sys.exit(f'守护失败: {len(missing)} 项基线指标在重算结果中消失 (如 {missing[:3]})')

    bad = []
    for k, v0 in sorted(ref.items()):
        v = cur[k]
        if v < v0 - TOL:
            bad.append((k, v0, v))
        elif abs(v - v0) > TOL:
            print(f'  漂移(可接受, 不劣化): {k}: {v0:.3f} -> {v:.3f}')

    if bad:
        print('\n════ 基线守护失败 — 以下指标劣化, 不允许合入 ════')
        for k, v0, v in bad:
            print(f'  {k}: {v0:.3f} -> {v:.3f}  (劣化 {v0-v:.3f})')
        sys.exit(1)
    print(f'\n════ PASS ════  {len(ref)} 项指标全部不劣于冻结基线 (容差 {TOL}) — 原有性能零弱化')

if __name__ == '__main__':
    main()
