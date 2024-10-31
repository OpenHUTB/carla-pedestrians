import string
import subprocess
import os
import time
from omegaconf import OmegaConf
import random
# os.environ.get('CUDA_VISIBLE_DEVICES')

import logging
log = logging.getLogger(__name__)


def kill_carla():
    kill_process = subprocess.Popen('killall -9 -r CarlaUE4-Linux', shell=True)
    kill_process.wait()
    time.sleep(1)
    log.info("Kill Carla Servers!")


class CarlaServerManager:
    def __init__(self, carla_sh_str, port=2000, configs=None, t_sleep=5):
        self._carla_sh_str = carla_sh_str
        # self._root_save_dir = root_save_dir
        self._t_sleep = t_sleep
        self.env_configs = []
        self._docker_id = ''

        if configs is None:
            cfg = {
                'gpu': 0,
                'port': port,
            }
            self.env_configs.append(cfg)
        else:
            for cfg in configs:
                for gpu in cfg['gpu']:
                    single_env_cfg = OmegaConf.to_container(cfg)
                    single_env_cfg['gpu'] = gpu
                    single_env_cfg['port'] = port
                    self.env_configs.append(single_env_cfg)
                    port += 5

    def start(self):
        # kill_carla()
        self._docker_id = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(12))
        for cfg in self.env_configs:
            # Temporary config file
            my_env = os.environ.copy()
            my_env["NV_GPU"] = str(cfg['gpu'])
            cmd = ['docker', 'run', '--name', self._docker_id, '--rm', '-d', '-p',
                   f'{cfg["port"]}-{cfg["port"]+2}:{cfg["port"]}-{cfg["port"]+2}', '--gpus', f'device={cfg["gpu"]}',
                   '-it', 'carlasim/carla:0.9.13', '/bin/bash', './CarlaUE4.sh',
                   f'-quality-level=Low', '-RenderOffScreen', '-nosound', f'-carla-port={cfg["port"]}']
            log.info(cmd)
            # log_file = self._root_save_dir / f'server_{cfg["port"]}.log'
            # server_process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid, stdout=open(log_file, "w"))
            server_process = subprocess.Popen(cmd, shell=False, stdout=subprocess.PIPE, env=my_env)
        time.sleep(self._t_sleep)

    def stop(self):
        kill_carla()
        time.sleep(self._t_sleep)
        log.info(f"Kill Carla Servers!")
