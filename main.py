#!/usr/bin/env python3
"""
NeuroSLAM — 一键运行入口 (根目录)
直接调用 neuro/main.py 完成数据采集 → 消融实验评估 → 可视化全流程

用法:
    python main.py                      # 一键运行完整流程
    python main.py --setup              # 安装所有依赖
    python main.py --collect-only       # 仅数据采集（需CARLA）
    python main.py --skip-collect       # 跳过采集，只评估（使用已有数据）
    python main.py --host 192.168.1.1   # 指定CARLA服务器地址
"""

import os
import sys
import subprocess

if __name__ == '__main__':
    ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
    NEURO_MAIN = os.path.join(ROOT_DIR, 'neuro', 'main.py')

    cmd = [sys.executable, NEURO_MAIN] + sys.argv[1:]
    sys.exit(subprocess.run(cmd).returncode)
