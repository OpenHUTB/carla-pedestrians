#!/usr/bin/env python3
"""5000 帧图像采集监督器：管理 CARLA 服务器生命周期 + 断点续采。

背景：CARLA 服务器在长时采集（~3000 帧量级）时偶发段错误崩溃；服务器
崩溃后客户端抛出 C++ 层 carla::client::TimeoutException（不是 Python
Exception 子类，进程内 except 无法捕获，采集进程被 std::terminate 直接
杀死）。因此"进程内容错"无法覆盖服务器崩溃，必须由外部监督器兜底：
本监督器负责
  1) 启动/重启 CARLA 服务器；
  2) 以子进程运行 IMU_Vision_Fusion_EKF.py（已支持断点续采：自动检测
     已采集帧数，数据文件追加写入，图像编号从断点继续）；
  3) 采集进程崩溃/挂起/未跑满时，重启服务器并从断点继续，
     循环直到跑满 5000 帧或达到最大尝试次数。

用法（用 CARLA 自带 venv 的 python 运行）:
    python3 collect_5000.py [--host localhost] [--port 2000]
        [--carla-bin /path/to/CarlaUE4.sh] [--map Town05]
        [--max-attempts 10]

注意：监督器全权管理服务器进程；若启动时已有服务器在监听端口，第一轮
直接复用；后续重启统一由监督器拉起。
"""
import argparse
import os
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
COLLECTOR = os.path.join(HERE, 'IMU_Vision_Fusion_EKF.py')
MAX_FRAMES = 5000
DEFAULT_CARLA_BIN = '/home/yangrb/下载/carla/CARLA_0.9.16/CarlaUE4.sh'
SERVER_PROC_PAT = 'CarlaUE4-Linux-Shipping'
# 看门狗：连续多少秒数据文件无新增行则判定采集挂起
HANG_GRACE_SECONDS = 240


def data_dir(map_name):
    return os.path.normpath(os.path.join(HERE, '..', 'data', f'{map_name}Data_IMU_Fusion'))


def count_frames(out_dir):
    """已采集完整帧数 = min(四个数据文件(行数-表头)的最小值, 编号图像数)。

    与采集器 detect_resume_count 口径一致：提前退出的补帧兜底会把数据
    文件补满 5001 行，若只数数据行会误判"已跑满"而停止监督。
    """
    files = ['ground_truth.txt', 'fusion_pose.txt',
             'visual_odometry.txt', 'aligned_imu.txt']
    counts = []
    for fname in files:
        fpath = os.path.join(out_dir, fname)
        if not os.path.isfile(fpath) or os.path.getsize(fpath) == 0:
            return 0
        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                n = sum(1 for _ in f)
        except Exception:
            return 0
        counts.append(n - 1)
    try:
        img_n = sum(1 for f in os.listdir(out_dir)
                    if len(f) == 8 and f[:4].isdigit() and f.endswith('.png'))
    except OSError:
        img_n = 0
    return max(0, min(min(counts), img_n))


def port_open(port, host='localhost'):
    try:
        with socket.create_connection((host, port), timeout=2):
            return True
    except Exception:
        return False


def wait_port(port, host, timeout=300):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if port_open(port, host):
            return True
        time.sleep(5)
    return False


def kill_server():
    """杀掉所有 CARLA 服务器进程（pkill 按进程名，最可靠）"""
    try:
        subprocess.run(['pkill', '-f', SERVER_PROC_PAT],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < 60:
        alive = subprocess.run(['pgrep', '-f', SERVER_PROC_PAT],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL).returncode == 0
        if not alive:
            return True
        time.sleep(3)
    # SIGKILL 兜底
    try:
        subprocess.run(['pkill', '-9', '-f', SERVER_PROC_PAT],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    time.sleep(3)
    return subprocess.run(['pgrep', '-f', SERVER_PROC_PAT],
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode != 0


def start_server(carla_bin, port, server_log):
    if port_open(port):
        print('[SERVER] 端口已监听，复用现有 CARLA 服务器')
        return None
    print(f'[SERVER] 启动 CARLA: {carla_bin} -port={port}')
    server_log.write(f'\n===== server start {time.strftime("%F %T")} =====\n')
    server_log.flush()
    proc = subprocess.Popen(
        [carla_bin, '-RenderOffScreen', '-quality-level=Low', f'-port={port}'],
        stdout=server_log, stderr=subprocess.STDOUT,
        start_new_session=True, cwd=HERE)
    return proc


def kill_proc_tree(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass
    t0 = time.time()
    while time.time() - t0 < 15 and proc.poll() is None:
        time.sleep(1)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        try:
            proc.wait(timeout=10)
        except Exception:
            pass


def run_attempt(attempt, host, port, out_dir, attempt_log, fresh=False):
    """运行一轮采集子进程，带"数据无增长"挂起看门狗。

    返回 (状态, 退出码)：状态 ∈ {'done','crash','hang'}
    fresh=True 时采集器加 --fresh（备份旧数据后全新采集）
    """
    attempt_log.write(f'\n===== attempt {attempt} start {time.strftime("%F %T")} =====\n')
    attempt_log.flush()
    cmd = [sys.executable, COLLECTOR, '--headless',
           '--host', host, '--port', str(port)]
    if fresh:
        cmd.append('--fresh')
    proc = subprocess.Popen(
        cmd, cwd=HERE, stdout=attempt_log, stderr=subprocess.STDOUT,
        start_new_session=True)
    last = count_frames(out_dir)
    last_change = time.time()
    print(f'[RUN {attempt}] 采集子进程启动 (pid={proc.pid})，当前进度 {last}/{MAX_FRAMES}')
    while proc.poll() is None:
        time.sleep(20)
        cur = count_frames(out_dir)
        if cur > last:
            last = cur
            last_change = time.time()
        if time.time() - last_change > HANG_GRACE_SECONDS:
            print(f'[RUN {attempt}] 已 {HANG_GRACE_SECONDS}s 无新增帧 (进度 {last}/{MAX_FRAMES})，判定挂起，杀掉子进程')
            kill_proc_tree(proc)
            return 'hang', -1
    rc = proc.returncode
    if rc == 0:
        return 'done', rc
    return 'crash', rc


def main():
    parser = argparse.ArgumentParser(description='5000 帧采集监督器（CARLA 服务器生命周期 + 断点续采）')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--map', default='Town05')
    parser.add_argument('--carla-bin', default=DEFAULT_CARLA_BIN)
    parser.add_argument('--max-attempts', type=int, default=10)
    parser.add_argument('--fresh', action='store_true',
                        help='首轮全新采集（备份并清空旧输出目录）；'
                             '默认自动断点续采')
    args = parser.parse_args()

    out_dir = data_dir(args.map)
    os.makedirs(out_dir, exist_ok=True)
    server_log = open('/tmp/carla_server_supervisor.log', 'a', encoding='utf-8')
    attempt_log = open('/tmp/collect_supervisor.log', 'a', encoding='utf-8')

    print(f'监督器启动: map={args.map}, out={out_dir}, max_attempts={args.max_attempts}')
    attempts = 0
    for attempts in range(1, args.max_attempts + 1):
        done_frames = count_frames(out_dir)
        if done_frames >= MAX_FRAMES:
            print(f'[OK] 已跑满 {MAX_FRAMES} 帧（在第 {attempts - 1} 轮后），监督结束')
            break

        # 1) 确保服务器存活
        if not port_open(args.port, args.host):
            kill_server()  # 清理可能残留的半死进程
            start_server(args.carla_bin, args.port, server_log)
            if not wait_port(args.port, args.host, timeout=300):
                print(f'[FATAL] 第 {attempts} 轮 CARLA 服务器启动失败（300s 端口未监听）')
                time.sleep(15)
                continue
            print(f'[OK] CARLA 服务器就绪 (port={args.port})')

        # 2) 运行采集（断点续采自动从 done_frames 继续；--fresh 时首轮全新）
        status, rc = run_attempt(attempts, args.host, args.port, out_dir,
                                 attempt_log, fresh=(args.fresh and attempts == 1))
        now_frames = count_frames(out_dir)
        print(f'[RUN {attempts}] 结束: status={status}, rc={rc}, 进度 {now_frames}/{MAX_FRAMES}')

        if now_frames >= MAX_FRAMES:
            print(f'[OK] 第 {attempts} 轮跑满 {MAX_FRAMES} 帧，监督结束')
            break
        if now_frames == done_frames:
            print(f'[WARN] 第 {attempts} 轮零进展（进度仍为 {now_frames}），重启服务器后续采')
        else:
            print(f'[INFO] 第 {attempts} 轮新增 {now_frames - done_frames} 帧，重启服务器后续采')
        # 3) 重启服务器（崩溃/段错误后必须重启才能继续）
        kill_server()
        time.sleep(5)
    else:
        print(f'[FATAL] {args.max_attempts} 轮后仍未跑满 5000 帧（当前 {count_frames(out_dir)} 帧），请人工检查')
        sys.exit(1)

    server_log.close()
    attempt_log.close()


if __name__ == '__main__':
    main()
