#!/usr/bin/env python3
"""地图加载验证：起 CARLA 服务器 → 客户端 load_world(map) → 同步模式跑 N 个 tick。

用途：在正式采集前验证某张地图（尤其是本机首次加载/资产合并后的地图）
能完整加载并保持稳定。服务器崩溃（段错误）时客户端抛异常，本脚本以非零
退出码报告。

用法（CARLA venv 的 python）:
    CARLA_BIN=/path/to/CarlaUE4.sh python3 verify_map_load.py --map Town07
        [--port 2000] [--host localhost] [--ticks 100]

退出码: 0=加载且 tick 稳定; 非零=失败（最后打印失败原因）。
无论成败，退出前都会杀掉自己启动的服务器。
"""
import argparse
import os
import socket
import subprocess
import sys
import time

SERVER_PROC_PAT = 'CarlaUE4-Linux-Shipping'


def port_open(port, host):
    try:
        with socket.create_connection((host, port), timeout=2):
            return True
    except Exception:
        return False


def kill_server():
    try:
        subprocess.run(['pkill', '-f', SERVER_PROC_PAT],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < 60:
        r = subprocess.run(['pgrep', '-f', SERVER_PROC_PAT],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode != 0:
            break
        time.sleep(2)
    else:
        try:
            subprocess.run(['pkill', '-9', '-f', SERVER_PROC_PAT],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--ticks', type=int, default=100)
    ap.add_argument('--port-timeout', type=int, default=300)
    ap.add_argument('--extra-flags', nargs='*', default=[],
                    help='附加服务器启动参数, 如 -RenderOffScreen -quality-level=Low '
                         '(与 collect_5000.py 同口径)')
    args = ap.parse_args()

    carla_bin = os.environ.get('CARLA_BIN',
                               '/home/yangrb/下载/carla/CARLA_0.9.16/CarlaUE4.sh')
    if not os.path.isfile(carla_bin):
        print(f'[FATAL] CARLA_BIN 不存在: {carla_bin}')
        return 2
    if port_open(args.port, args.host):
        print(f'[FATAL] 端口 {args.port} 已被占用，先清理再验证')
        return 2

    log_path = f'/tmp/verify_carla_{args.map}.log'
    server_cmd = [carla_bin] + list(args.extra_flags) + [f'-port={args.port}']
    print(f'[SERVER] 启动: {" ".join(server_cmd)}  (日志 {log_path})')
    with open(log_path, 'wb') as logf:
        proc = subprocess.Popen(server_cmd, stdout=logf, stderr=subprocess.STDOUT)
    t0 = time.time()
    ready = False
    while time.time() - t0 < args.port_timeout:
        if proc.poll() is not None:
            print(f'[FATAL] 服务器进程提前退出 rc={proc.returncode}，'
                  f'最后日志:')
            with open(log_path, 'r', errors='replace') as f:
                tail = f.read()[-2000:]
            print(tail)
            return 1
        if port_open(args.port, args.host):
            ready = True
            break
        time.sleep(5)
    if not ready:
        print(f'[FATAL] 端口 {args.port} 在 {args.port_timeout}s 内未就绪')
        kill_server()
        return 1
    print(f'[OK] 服务器端口就绪 ({time.time() - t0:.0f}s)')

    rc = 1
    try:
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        t1 = time.time()
        print(f'[MAP] load_world({args.map}) ...')
        world = client.load_world(args.map)
        snap = world.get_snapshot()
        print(f'[OK] 地图加载完成 ({time.time() - t1:.0f}s), frame={snap.frame}')
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        t2 = time.time()
        for i in range(args.ticks):
            world.tick()
        print(f'[OK] {args.ticks} 个同步 tick 稳定 '
              f'({time.time() - t2:.1f}s)')
        rc = 0
    except Exception as e:
        print(f'[FATAL] 验证失败: {type(e).__name__}: {e}')
        with open(log_path, 'r', errors='replace') as f:
            tail = f.read()[-2000:]
        print('--- 服务器日志尾部 ---')
        print(tail)
    finally:
        print('[CLEANUP] 关闭服务器')
        kill_server()
    return rc


if __name__ == '__main__':
    sys.exit(main())
