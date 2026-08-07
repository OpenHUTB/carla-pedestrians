#!/usr/bin/env python3
"""
NeuroSLAM — 一键运行入口
串联 数据采集 → 消融实验评估 → 可视化 全流程

用法:
    python main.py                      # 一键运行：自动采集 + 评估（自动覆盖旧数据）
    python main.py --setup              # 安装所有依赖
    python main.py --collect-only       # 仅数据采集（需CARLA）
    python main.py --skip-collect       # 跳过采集，只评估（使用已有数据）
    python main.py --keep-data          # 保留已有数据，仅采集新数据（不删除旧数据）
    python main.py --host 192.168.1.1   # 指定CARLA服务器地址
"""

import os
import sys
import time
import json
import glob
import shutil
import shlex
import argparse
import subprocess
import socket

# 修复 Windows 终端 GBK 编码问题
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# ─── 路径常量（全部相对路径，无绝对路径依赖） ─────────────────
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
COLLECT_SCRIPT = os.path.join(ROOT_DIR, '00_collect_data', 'IMU_Vision_Fusion_EKF.py')
ABLATE_SCRIPT = os.path.join(ROOT_DIR, '07_test', 'run_ablation.py')
REQUIREMENTS_FILE = os.path.join(ROOT_DIR, 'requirements.txt')
DATA_DIR = os.path.join(ROOT_DIR, 'data')
DEFAULT_HOST = 'localhost'
DEFAULT_PORT = 2000


# ═══════════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════════

def print_banner():
    print()
    print("=" * 62)
    print("    NeuroSLAM — Bio-Inspired VIO Pipeline")
    print("    python main.py --help  查看所有选项")
    print("=" * 62)
    print()


def check_carla_server(host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=3.0):
    """检测 CARLA 服务器是否在运行"""
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.close()
        return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False


def _carla_exe_name():
    """返回当前平台 CARLA 可执行文件名称"""
    return 'CarlaUE4.exe' if sys.platform == 'win32' else 'CarlaUE4.sh'


def find_carla_exe():
    """
    自动搜索 CARLA 服务器可执行文件（跨平台）。
    返回 (exe_path, carla_root) 或 (None, None)

    搜索策略：
    1. 环境变量 CARLA_ROOT
    2. 从脚本目录向上逐层搜索（最多 6 层）
    3. 搜索用户主目录下常见 CARLA 目录名
    4. 深度遍历（限制深度 5，跳过无关目录）
    """
    _exe_name = _carla_exe_name()

    # 1) 优先使用 CARLA_ROOT 环境变量
    _carla_root_env = os.environ.get('CARLA_ROOT', '')
    if _carla_root_env:
        _exe = os.path.join(_carla_root_env, _exe_name)
        if os.path.isfile(_exe):
            return _exe, _carla_root_env

    # 2) 从脚本目录向上搜索（最多 6 层）
    _search_dir = ROOT_DIR
    for _ in range(6):
        _exe = os.path.join(_search_dir, _exe_name)
        if os.path.isfile(_exe):
            return _exe, _search_dir
        _parent = os.path.dirname(_search_dir)
        if _parent == _search_dir:
            break
        _search_dir = _parent

    # 3) 搜索用户主目录下常见 CARLA 目录名
    _home = os.path.expanduser('~')
    _carla_dir_names = ['CARLA', 'carla', 'CARLA_0.9.16', 'CARLA_0.9.15',
                        'CARLA_0.9.14', 'carla-0.9.16']
    for _name in _carla_dir_names:
        _check = os.path.join(_home, _name, _exe_name)
        if os.path.isfile(_check):
            return _check, os.path.dirname(_check)
        # 也搜索一级子目录（如 ~/CARLA/CARLA_0.9.16/）
        _parent_dir = os.path.join(_home, _name)
        if os.path.isdir(_parent_dir):
            for _sub in os.listdir(_parent_dir):
                _check = os.path.join(_parent_dir, _sub, _exe_name)
                if os.path.isfile(_check):
                    return _check, os.path.dirname(_check)

    # 4) 从脚本目录向上深度搜索（限制深度 5，跳过无关目录）
    _skip_dirs = {
        '$RECYCLE.BIN', 'System Volume Information', 'Windows', '$WinREAgent',
        'Config.Msi', 'MSOCache', 'PerfLogs', 'Recovery',
        'Temp', 'Python', 'AppData', 'Desktop', 'Documents',
        'Downloads', 'Music', 'Pictures', 'Videos', 'OneDrive',
        '__pycache__', '.git', 'node_modules', '.venv', 'venv',
        'alphapose', 'openpose', 'bip', 'walker', 'pedestrians-scenarios',
        'pedestrians-video-2-carla', 'carla-common', 'resources', 'docs',
    }

    _search_roots = []
    _d = ROOT_DIR
    for _ in range(5):
        _search_roots.append(_d)
        _parent = os.path.dirname(_d)
        if _parent == _d:
            break
        _d = _parent

    for _root in _search_roots:
        for _dirpath, _dirnames, _filenames in os.walk(_root):
            # 跳过无关目录
            _dirnames[:] = [d for d in _dirnames
                            if d not in _skip_dirs
                            and not d.startswith('$')
                            and not d.startswith('.')]
            _depth = _dirpath.replace(_root, '').count(os.sep)
            if _depth > 5:
                _dirnames[:] = []
                continue
            if _exe_name in _filenames:
                _exe_path = os.path.join(_dirpath, _exe_name)
                _carla_root = _dirpath
                if os.path.basename(_carla_root).lower() == 'win64':
                    _carla_root = os.path.dirname(os.path.dirname(os.path.dirname(_carla_root)))
                return _exe_path, _carla_root

    return None, None


def find_carla_python():
    """
    找到能导入 carla 模块的 Python 解释器（跨平台）。
    返回 (python_path_list, description) 或 (None, error_msg)

    搜索策略：
    1. 当前 Python 解释器
    2. (Windows) py 启动器 / LOCALAPPDATA / ProgramFiles 下的 Python
    3. (Linux/macOS) PATH 中的 python3 系列
    """
    candidates = []

    # 1) 当前 Python（优先）
    candidates.append(("当前Python", [sys.executable]))

    if sys.platform == 'win32':
        # Windows Python Launcher
        for ver in ["3.12", "3.11", "3.10", "3.9", "3.8", "3"]:
            candidates.append((f"py -{ver}", ["py", f"-{ver}"]))
            candidates.append((f"python{ver}", [f"python{ver}"]))
        # LOCALAPPDATA 下的 Python 安装
        local_app_data = os.environ.get('LOCALAPPDATA', '')
        if local_app_data:
            for ver in ['312', '311', '310', '39', '38']:
                base = os.path.join(local_app_data, 'Programs', 'Python',
                                    f'Python{ver}', 'python.exe')
                if os.path.exists(base):
                    candidates.append((base, [base]))
        for prog_env in ['ProgramFiles', 'ProgramFiles(x86)']:
            prog_base = os.environ.get(prog_env, '')
            if prog_base:
                for ver in ['312', '311', '310', '39', '38']:
                    base = os.path.join(prog_base, 'Python', f'Python{ver}', 'python.exe')
                    if os.path.exists(base):
                        candidates.append((base, [base]))
    else:
        # Linux/macOS: 搜索 PATH 中的 python3 解释器
        _seen = set()
        for _cmd in ['python3', 'python3.12', 'python3.11', 'python3.10',
                     'python3.9', 'python3.8', 'python']:
            _found = shutil.which(_cmd)
            if _found and _found not in _seen:
                _seen.add(_found)
                candidates.append((_found, [_found]))

    # 去重
    seen = set()
    unique = []
    for label, cmd in candidates:
        key = tuple(cmd)
        if key not in seen:
            seen.add(key)
            unique.append((label, cmd))

    for label, cmd in unique:
        try:
            result = subprocess.run(
                [*cmd, '-c', 'import carla; print("ok")'],
                capture_output=True, timeout=15,
                text=True,
            )
            if result.returncode == 0 and 'ok' in result.stdout:
                return cmd, label
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue

    if sys.platform == 'win32':
        return None, (
            "找不到可导入 carla 的 Python 解释器。\n"
            "CARLA 0.9.8 需要 Python 3.7，CARLA 0.9.16 需要 Python 3.12。\n"
            "安装 carla 包:\n"
            "  py -3.7 -m pip install <CARLA_DIR>\\PythonAPI\\carla\\dist\\carla-*.egg\n"
            "  py -3.12 -m pip install <CARLA_DIR>\\PythonAPI\\carla\\dist\\carla-*.whl"
        )
    else:
        return None, (
            "找不到可导入 carla 的 Python 解释器。\n"
            "请确保已在 Python 环境中安装 carla wheel:\n"
            "  pip install <CARLA_DIR>/PythonAPI/carla/dist/carla-*.whl"
        )


def _launch_carla(carla_exe):
    """启动 CARLA 服务器（跨平台），返回 Popen 对象"""
    _cwd = os.path.dirname(carla_exe)
    if sys.platform == 'win32':
        return subprocess.Popen(
            [carla_exe, '-RenderOffScreen', '-quality-level=Low'],
            cwd=_cwd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
        )
    else:
        return subprocess.Popen(
            ['bash', carla_exe, '-RenderOffScreen', '-quality-level=Low'],
            cwd=_cwd,
            start_new_session=True,
        )


def _carla_launch_cmd_str(carla_exe):
    """返回人类可读的 CARLA 启动命令字符串"""
    if sys.platform == 'win32':
        return f"{carla_exe} -RenderOffScreen -quality-level=Low"
    else:
        return f"bash {carla_exe} -RenderOffScreen -quality-level=Low"


def run_python_script(script_path, desc, python_exe=None, extra_args=None, env=None):
    """
    运行一个 Python 脚本。
    python_exe: 字符串路径或字符串列表，为 None 则用 sys.executable
    env: 额外的环境变量 dict
    返回 bool 表示是否成功
    """
    if python_exe is None:
        python_exe = [sys.executable]
    elif isinstance(python_exe, str):
        python_exe = [python_exe]

    print(f"\n{'-' * 60}")
    print(f"  >> {desc}")
    print(f"  >> 脚本: {os.path.basename(script_path)}")
    print(f"{'-' * 60}\n")

    cmd = python_exe + [script_path]
    if extra_args:
        cmd.extend(extra_args)

    run_env = os.environ.copy()
    if env:
        run_env.update(env)

    print(f"[INFO] 命令: {shlex.join(cmd)}")

    result = subprocess.run(cmd, cwd=os.path.dirname(script_path) or ROOT_DIR, env=run_env)
    if result.returncode != 0:
        # CARLA 0.9.8 在退出时已知会崩溃 (PyEval_SaveThread: NULL tstate)
        # 退出码 3221226505 = STATUS_STACK_BUFFER_OVERRUN，数据已成功保存
        if result.returncode == 3221226505:
            print(f"\n[WARN] CARLA 0.9.8 退出时发生已知崩溃 (exit code {result.returncode})")
            print("[INFO] 数据采集已完成，忽略此错误")
            return True
        print(f"\n[ERROR] {desc} 失败 (exit code {result.returncode})")
        return False
    print(f"\n[OK] {desc} 完成")
    return True


def discover_datasets(data_root):
    """发现 data/ 下所有有效数据集（含 ground_truth.txt 的目录）"""
    datasets = []
    if not os.path.isdir(data_root):
        return datasets
    for entry in sorted(os.listdir(data_root)):
        full = os.path.join(data_root, entry)
        if not os.path.isdir(full):
            continue
        gt = os.path.join(full, 'ground_truth.txt')
        if os.path.exists(gt):
            datasets.append(full)
    return datasets


def install_dependencies():
    """安装 Python 依赖"""
    if not os.path.exists(REQUIREMENTS_FILE):
        print(f"[WARN] 未找到 {REQUIREMENTS_FILE}，跳过依赖安装")
        return True

    print(f"\n[INFO] 安装依赖: pip install -r {REQUIREMENTS_FILE}")
    result = subprocess.run(
        [sys.executable, '-m', 'pip', 'install', '-r', REQUIREMENTS_FILE],
        cwd=ROOT_DIR,
    )
    if result.returncode != 0:
        print("[ERROR] 依赖安装失败，请手动执行:")
        print(f"  {sys.executable} -m pip install -r {REQUIREMENTS_FILE}")
        return False
    print("[OK] 依赖安装完成")
    return True


# ═══════════════════════════════════════════════════════════
#  Pipeline 步骤
# ═══════════════════════════════════════════════════════════

def step_collect(carla_host, carla_port, carla_py, keep_data=False, map_name=None):
    """Step 1: CARLA 数据采集（非交互式）"""
    carla_root = None

    # 检查 CARLA 服务器，若未运行则自动启动
    if not check_carla_server(carla_host, carla_port):
        carla_exe, carla_root = find_carla_exe()
        if carla_exe:
            print("\n" + "=" * 60)
            print(f"  [自动启动] CARLA: {carla_exe}")
            print("=" * 60)
            try:
                _launch_carla(carla_exe)
                print("  [等待] CARLA 启动中...")
                for i in range(90):
                    if check_carla_server(carla_host, carla_port):
                        print(f"[OK] CARLA 服务器已连接 (耗时约 {i} 秒)\n")
                        break
                    time.sleep(1)
                else:
                    print("[WARN] CARLA 启动超时，但继续尝试...\n")
            except Exception as e:
                print(f"[WARN] 自动启动 CARLA 失败: {e}")
                print("  请手动启动 CARLA:")
                print(f"    {_carla_launch_cmd_str(carla_exe)}")
                print()
                while not check_carla_server(carla_host, carla_port):
                    time.sleep(3)
                print("[OK] CARLA 服务器已连接\n")
        else:
            _exe_name = _carla_exe_name()
            print("\n" + "=" * 60)
            print(f"  [等待] CARLA 服务器未连接，且未找到 {_exe_name}")
            print("  请手动启动 CARLA，例如:")
            print(f"    {_exe_name} -RenderOffScreen -quality-level=Low")
            print("=" * 60)
            while not check_carla_server(carla_host, carla_port):
                time.sleep(3)
            print("[OK] CARLA 服务器已连接\n")

    if not carla_root:
        carla_exe, carla_root = find_carla_exe()

    # 处理已有数据：一键模式默认自动删除旧数据，--keep-data 保留
    if os.path.isdir(DATA_DIR):
        existing = discover_datasets(DATA_DIR)
        if existing:
            if keep_data:
                print(f"[INFO] 保留已有 {len(existing)} 个数据集，将在其基础上继续采集")
                for d in existing:
                    pngs = len([f for f in os.listdir(d) if f.endswith('.png')])
                    print(f"  - {os.path.basename(d)} ({pngs} 张图像)")
            else:
                print(f"[INFO] 自动清理旧数据: {len(existing)} 个数据集")
                for d in existing:
                    shutil.rmtree(d)
                    print(f"  已删除: {os.path.basename(d)}")

    # 传递 CARLA_ROOT 环境变量给子进程
    env = {}
    if carla_root:
        env['CARLA_ROOT'] = carla_root
        print(f"[INFO] CARLA_ROOT = {carla_root}")

    extra_args = ['--headless']
    if map_name:
        extra_args.extend(['--map', map_name])
        print(f"[INFO] 目标地图: {map_name}")

    return run_python_script(COLLECT_SCRIPT,
                             "Step 1/2: CARLA 数据采集 (IMU + Vision EKF 融合)",
                             python_exe=carla_py,
                             extra_args=extra_args,
                             env=env)


def step_ablate():
    """Step 2: 消融实验评估"""
    datasets = discover_datasets(DATA_DIR)
    if not datasets:
        print("\n[ERROR] 没有找到有效数据集")
        print(f"请确保 {DATA_DIR}/ 下有包含 ground_truth.txt 的数据目录")
        print("运行数据采集: python main.py --collect-only")
        return False

    print(f"\n[INFO] 发现 {len(datasets)} 个数据集:")
    for d in datasets:
        pngs = len([f for f in os.listdir(d) if f.endswith('.png')])
        print(f"  - {os.path.basename(d)} ({pngs} 张图像)")

    return run_python_script(ABLATE_SCRIPT, "Step 2/2: 消融实验评估")


def print_results():
    """打印已有评估结果"""
    result_files = [
        os.path.join(DATA_DIR, 'ablation_results.json'),
        os.path.join(DATA_DIR, 'ablation_summary.json'),
    ]
    for rf in result_files:
        if os.path.exists(rf):
            break
    else:
        return

    try:
        with open(rf, 'r', encoding='utf-8') as f:
            results = json.load(f)
    except Exception:
        return

    print("\n" + "=" * 62)
    print("  Evaluation Results")
    print("=" * 62)
    print(f"{'Method':<22} {'ATE(m)':<10} {'RPE(m/f)':<10} {'Drift%':<10}")
    print("-" * 62)
    for r in results:
        ate = r.get('ATE_m', 'N/A')
        rpe = r.get('RPE_m', 'N/A')
        drift = r.get('Drift_pct', 'N/A')
        print(f"{r.get('method', '?'):<22} {str(ate):<10} {str(rpe):<10} {str(drift):<10}")

    for sub in discover_datasets(DATA_DIR):
        for fname in ['ablation_comparison.png', 'ablation_trajectory.png']:
            path = os.path.join(sub, fname)
            if os.path.exists(path):
                print(f"  图表: {path}")


# ═══════════════════════════════════════════════════════════
#  主入口
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='NeuroSLAM — 一键数据采集 + 消融评估',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python main.py                      一键采集 + 评估（自动清理旧数据）
  python main.py --map Town10HD       在 Town10HD 地图采集
  python main.py --keep-data          保留旧数据，追加采集新数据
  python main.py --setup              安装所有 Python 依赖
  python main.py --collect-only       仅采集 CARLA 数据
  python main.py --skip-collect       跳过采集，只评估已有数据
  python main.py --host 192.168.1.1   连接远程 CARLA 服务器
        """,
    )
    parser.add_argument('--setup', action='store_true',
                        help='安装所有 Python 依赖后退出')
    parser.add_argument('--collect-only', action='store_true',
                        help='仅运行 CARLA 数据采集')
    parser.add_argument('--skip-collect', action='store_true',
                        help='跳过数据采集，仅运行评估')
    parser.add_argument('--keep-data', action='store_true',
                        help='保留已有数据，不自动删除（默认会清理旧数据）')
    parser.add_argument('--host', default=DEFAULT_HOST,
                        help=f'CARLA 服务器地址 (默认: {DEFAULT_HOST})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help=f'CARLA 服务器端口 (默认: {DEFAULT_PORT})')
    parser.add_argument('--map', default=None,
                        help='CARLA 地图名称 (默认: 不干预，由采集脚本 DEFAULT_TARGET_MAP 决定), '
                             '例如: Town01, Town02, Town03, Town05, Town10HD')

    args = parser.parse_args()

    print_banner()

    # ── --setup: 安装依赖 ──
    if args.setup:
        if install_dependencies():
            print("\n[OK] 环境准备完成，现在可以运行: python main.py")
        else:
            sys.exit(1)
        return

    # ── 确定是否需要 CARLA ──
    need_carla = not args.skip_collect
    carla_py = None

    if need_carla:
        carla_py, info = find_carla_python()
        if carla_py is None:
            # 找不到 CARLA Python，检查是否有已有数据可以评估
            print(f"\n[WARN] {info}")
            print()
            datasets = discover_datasets(DATA_DIR)
            if datasets:
                print(f"[INFO] 已有 {len(datasets)} 个数据集，自动跳过采集，直接评估")
                need_carla = False
            else:
                print("[ERROR] 无法进行数据采集，且没有已有数据可供评估")
                sys.exit(1)
        else:
            print(f"[INFO] CARLA Python: {info}")

    # ── 执行流程 ──
    start_time = time.time()
    success = True

    if args.collect_only:
        if carla_py is None:
            carla_py, _ = find_carla_python()
        if carla_py is None:
            print("[ERROR] 需要 CARLA Python 环境")
            sys.exit(1)
        success = step_collect(args.host, args.port, carla_py, keep_data=args.keep_data, map_name=args.map)
    elif args.skip_collect:
        success = step_ablate()
    else:
        # 完整流程：采集 + 评估
        if need_carla and carla_py:
            success = step_collect(args.host, args.port, carla_py, keep_data=args.keep_data, map_name=args.map)
            if not success:
                print("\n[WARN] 数据采集未完全成功，尝试继续评估...")
        success = step_ablate() and success

    # ── 结果汇总 ──
    elapsed = time.time() - start_time
    mins, secs = divmod(int(elapsed), 60)
    print(f"\n总耗时: {mins} 分 {secs} 秒")

    if success:
        print_results()
        print("\n[SUCCESS] NeuroSLAM 流程全部完成!")
    else:
        print("\n[FAILED] 流程中断，请检查上方错误信息")
        sys.exit(1)


if __name__ == '__main__':
    main()