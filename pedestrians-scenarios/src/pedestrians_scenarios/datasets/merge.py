import logging
import pandas as pd
import os
import shutil
import re
from tqdm.auto import tqdm


def add_cli_args(parser):
    parser.add_argument(
        '--input_dirs',
        type=str,
        nargs='+',
        default=[],
        help='Directories to merge.'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        help='Directory to save merged data.'
    )
    return parser


def command(input_dirs, output_dir, **kwargs):
    """Command line interface for generating datasets.

    :param kwargs: parsed command line arguments
    :type kwargs: Dict
    """
    clips_dir = os.path.join(output_dir, 'clips')

    os.makedirs(clips_dir)

    dfs = []
    name_regex = re.compile(
        r'(?P<batch_idx>\d+)-(?P<clip_idx>\d+)-(?P<pedestrian_idx>\d+)-(?P<camera_idx>\d+)\.mp4')

    for in_dir in tqdm(input_dirs, desc='Input directories'):
        df = pd.read_csv(in_dir + "/data.csv")

        if 'frame.pedestrian.is_crossing' in df.columns:
            df.loc[:, 'frame.pedestrian.is_crossing'] = df.loc[:,
                                                               'frame.pedestrian.is_crossing'].astype(bool)
        else:
            # old datasets didn't have it
            df['frame.pedestrian.is_crossing'] = True

        if 'camera.width' not in df.columns:
            df['camera.width'] = 800

        if 'camera.height' not in df.columns:
            df['camera.height'] = 600

        # remove unneeded frame.pedestrian.id column
        if 'frame.pedestrian.id' in df.columns:
            df.drop(columns=['frame.pedestrian.id'], inplace=True)

        clip_groups = df.groupby(by=['id', 'camera.recording'])
        for (clip_id, clip_path), index in tqdm(clip_groups.groups.items(), leave=False, desc='Clips'):
            name_regex_match = name_regex.match(os.path.basename(clip_path))
            if name_regex_match:
                new_name = '{}-{}-{}.mp4'.format(
                    clip_id,
                    name_regex_match.group('pedestrian_idx'),
                    name_regex_match.group('camera_idx'),
                )
                new_path = os.path.join(clips_dir, new_name)
                if not os.path.exists(new_path):
                    # if file already exists, don't copy it - duplicates will be dropped later
                    shutil.copyfile(os.path.join(in_dir, clip_path), new_path)
                    df.loc[index, 'camera.recording'] = os.path.join('clips', new_name)
            else:
                new_path = os.path.join(clips_dir, os.path.basename(clip_path))
                if not os.path.exists(new_path):
                    # if file already exists, don't copy it - duplicates will be dropped later
                    shutil.copyfile(os.path.join(in_dir, clip_path), new_path)
        dfs.append(df)

    logging.getLogger(__name__).info('Merging & postprocessing dataframes...')
    df = pd.concat(dfs)
    df.drop_duplicates(subset=['id', 'camera.idx', 'pedestrian.idx', 'frame.idx'], keep='first', inplace=True)

    df['frame.pedestrian.is_crossing'].fillna(True, inplace=True)
    df['camera.width'].fillna(800, inplace=True)
    df['camera.height'].fillna(600, inplace=True)

    logging.getLogger(__name__).info('Saving dataframe...')
    df.to_csv(os.path.join(output_dir, 'data.csv'), index=False, header=True)
