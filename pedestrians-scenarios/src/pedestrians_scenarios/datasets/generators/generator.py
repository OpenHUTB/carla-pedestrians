import ast
import glob
import logging
import multiprocessing as mp
import os
import time
from typing import Iterable, Tuple

import numpy as np
from tqdm.auto import tqdm

from .batch_generator import (BatchGenerator, ExamplePedestrianProfiles,
                              PedestrianProfile, StandardDistribution)


class Generator(object):
    """
    Common class for various datasets generation. Handles managing/saving etc.
    """
    batch_generator = BatchGenerator

    def __init__(self,
                 outputs_dir: str = './datasets',
                 number_of_clips: int = 512,
                 clip_length_in_frames: int = 900,
                 pedestrian_distributions: Iterable[Tuple[PedestrianProfile, float]] = (
                     (ExamplePedestrianProfiles.adult_female.value, 0.25),
                     (ExamplePedestrianProfiles.adult_male.value, 0.25),
                     (ExamplePedestrianProfiles.child_female.value, 0.25),
                     (ExamplePedestrianProfiles.child_male.value, 0.25)
                 ),
                 camera_distances_distributions: Iterable[Iterable[StandardDistribution]] = ((
                     StandardDistribution(-7.0, 2.0),
                     StandardDistribution(0.0, 0.25),
                     StandardDistribution(1.0, 0.25)
                 ),),
                 batch_size: int = 1,
                 camera_fov: float = 90.0,
                 camera_image_size: Tuple[int, int] = (1600, 600),
                 failure_multiplier: int = 2,
                 overwrite: bool = False,
                 **kwargs
                 ) -> None:
        self._outputs_dir = outputs_dir
        # Ensure that the output directory exists AND is empty
        if os.path.exists(self._outputs_dir):
            if os.listdir(self._outputs_dir) and not overwrite:
                raise ValueError(
                    f'Output directory {self._outputs_dir} is not empty.')
        os.makedirs(self._outputs_dir, exist_ok=overwrite)

        # handle complex config data
        self._camera_distances_distributions = self.__parse_camera_position_distributions(
            camera_distances_distributions)
        self._pedestrian_distributions = self.__parse_pedestrian_distributions(
            pedestrian_distributions)

        self._number_of_clips = number_of_clips
        self._clip_length_in_frames = clip_length_in_frames
        self._batch_size = batch_size
        self._camera_fov = camera_fov
        self._camera_image_size = camera_image_size

        self._total_batches = np.ceil(
            self._number_of_clips/self._batch_size).astype(int)
        self._kwargs = kwargs

        self._failure_multiplier = failure_multiplier

        mp.set_start_method('spawn')

    def __parse_camera_position_distributions(self, camera_position_distributions):
        converted_camera_position_distributions = []
        for i, camera in enumerate(camera_position_distributions):
            if len(camera) != 3:
                raise ValueError(
                    f'Camera position distribution {i} should have exactly 3 elements (for x,y,z), got {len(distribution)}')
            camera_distributions = []
            for distribution in camera:
                if isinstance(distribution, StandardDistribution):
                    camera_distributions.append(distribution)
                else:
                    camera_distributions.append(StandardDistribution(*distribution))
            converted_camera_position_distributions.append(camera_distributions)
        return converted_camera_position_distributions

    def __parse_pedestrian_distributions(self, pedestrian_distributions):
        converted_pedestrian_distributions = []
        for distribution in pedestrian_distributions:
            profile, weight = distribution
            if isinstance(profile, PedestrianProfile):
                converted_pedestrian_distributions.append((profile, weight))
            elif isinstance(profile, str):
                converted_pedestrian_distributions.append(
                    (ExamplePedestrianProfiles[profile].value, weight))
            else:
                assert len(
                    profile) == 4, f'Pedestrian profile should have exactly 4 elements, got {len(profile)}'
                (age, gender, (walking_speed_mean, walking_speed_std),
                 (crossing_speed_mean, crossing_speed_std)) = profile
                converted_pedestrian_distributions.append(
                    (PedestrianProfile(
                        age, gender,
                        StandardDistribution(walking_speed_mean, walking_speed_std),
                        StandardDistribution(crossing_speed_mean, crossing_speed_std)
                    ), weight))
        return converted_pedestrian_distributions

    @staticmethod
    def add_cli_args(parser):
        subparser = parser.add_argument_group('Generator')

        subparser.add_argument('--outputs_dir', default=None, type=str,
                               help='Directory to store outputs (default: ./datasets).')
        subparser.add_argument('--number_of_clips', type=int, default=512,
                               help='Total number of clips to generate.')
        subparser.add_argument('--clip_length_in_frames', type=int, default=900,
                               help='Length of each clip in frames.')
        subparser.add_argument('--batch_size', type=int, default=1,
                               help='Number of clips in each batch.')
        subparser.add_argument('--camera_fov', type=float, default=90.0,
                               help='Camera horizontal FOV in degrees.')
        subparser.add_argument('--camera_image_size', type=ast.literal_eval, default='(1600,600)',
                               help='Camera image size in pixels as a (width, height) tuple (default: (1600,600)).')
        subparser.add_argument('--waypoint_jitter_scale', type=float,
                               default=1.0, help='Scale of jitter applied to waypoints.')
        subparser.add_argument('--failure_multiplier', type=int,
                               default=2, help='Multiplier for number of clips to generate in case of failure.')
        subparser.add_argument('--overwrite', action='store_true', default=False)

        return parser

    def generate(self) -> None:
        """
        Generate the dataset.
        """

        outfile = os.path.join(self._outputs_dir, 'data.csv')
        outfile_lock = mp.Lock()
        results_queue = mp.Queue()

        generated_clips_count = []
        batch_idx = 0
        failed = 0
        server_failed = 0

        with tqdm(total=self._number_of_clips, desc='Clips', position=0, postfix={'failed': 0}) as pbar:
            while sum(generated_clips_count) < self._number_of_clips and batch_idx < self._failure_multiplier*self._total_batches:
                failed = batch_idx * self._batch_size - sum(generated_clips_count)
                pbar.set_postfix(failed=failed)

                # TODO: this has the potential for a 'real' multiprocessing if multiple servers are available
                seed = self._kwargs.get('seed', None)
                batch_generation_process = self.batch_generator(
                    outfile=outfile,
                    outfile_lock=outfile_lock,
                    outputs_dir=self._outputs_dir,
                    queue=results_queue,
                    batch_idx=batch_idx,
                    batch_size=self._batch_size,
                    clip_length_in_frames=self._clip_length_in_frames,
                    pedestrian_distributions=self._pedestrian_distributions,
                    camera_distances_distributions=self._camera_distances_distributions,
                    camera_fov=self._camera_fov,
                    camera_image_size=self._camera_image_size,
                    **{
                        **self._kwargs,
                        'seed': seed + batch_idx if seed is not None else None
                    }
                )
                batch_generation_process.start()
                batch_generation_process.join()

                batch_idx += 1

                if results_queue.empty():
                    logging.getLogger(__name__).warning(
                        f'Process failed for batch {batch_idx}')
                    server_failed += 1
                    # try to remove 'core.*' files
                    for core_file in glob.glob(os.path.join(os.getcwd(), 'core.*')):
                        os.remove(core_file)
                    # assume that the server will restart
                    time.sleep(float(os.getenv('CARLA_SERVER_START_PERIOD', '30.0')))
                    continue

                # append number of clips generated in this batch
                generated_clips_count.append(results_queue.get())

                if generated_clips_count[-1] > 0:
                    pbar.update(generated_clips_count[-1])

        logging.getLogger(__name__).info(
            f'Generated {sum(generated_clips_count)} clips out of desired {self._number_of_clips}. Batch generation failed {failed} times, including {server_failed} server failures/timeouts.')
