import glob
import logging
import os
from typing import List

import numpy as np
import pims
from pandas.core.frame import DataFrame
import torch

from tqdm.auto import tqdm
from pedestrians_video_2_carla.data.base.mixins.dataset.video_mixin import VideoMixin
from pedestrians_video_2_carla.transforms.video.video_to_resnet import VideoToResNet

from pedestrians_video_2_carla.data.openpose.skeleton import COCO_SKELETON
from pedestrians_video_2_carla.data.openpose.datamodules.jaad_openpose_datamodule import JAADOpenPoseDataModule
from pedestrians_video_2_carla.data.openpose.constants import JAAD_DIR
from pedestrians_video_2_carla.data.unipose.constants import UNIPOSE_MODEL_DIR


class JAADUniPoseDataModule(JAADOpenPoseDataModule):
    """
    An attempt to see how changing the pose extractor affects the results.
    This in theory works, but the results are so poor that it makes no sense to use it (or optimize its inner workings).
    Replacing UniPoseCOCO with UniPoseLSTM was even worse.
    Code is kept here for future reference (e.g. when better pose extractor will be found).

    Potential optimizations should include:
    - going through all the videos only once & caching the results (as OpenPose-compatible keypoints.json files)
    - then leveraging the rest of OpenPoseDataModule's functionality to extract the keypoints for specific clips
    - histogram equalization of the images (to improve the results)
    - better normalization of the images (to improve the results)
    """

    def __init__(self,
                 **kwargs
                 ):
        super().__init__(**kwargs)

        # override openpose dir with unipose dumped data
        self._unipose_model_path = os.path.join(UNIPOSE_MODEL_DIR, 'UniPose_COCO.pth')

        self._target_size = 368
        self._video_transform = VideoToResNet(target_size=self._target_size)

    @staticmethod
    def add_subclass_specific_args(parent_parser):
        parent_parser = JAADOpenPoseDataModule.add_subclass_specific_args(parent_parser)

        parser = parent_parser.add_argument_group('JAADUniPose DataModule')
        parser.set_defaults(
            data_nodes=COCO_SKELETON
        )

        return parent_parser

    def _extract_additional_data(self, clips: List[DataFrame]):
        """
        Extract skeleton data from keypoint files. This potentially modifies data in place!

        :param clips: List of DataFrames
        :type clips: List[DataFrame]
        """

        # only load it when we really need it
        unipose_model = self._load_unipose_model(self._unipose_model_path)

        # TODO: make this configurable, but for now we only use CPU since it doesn't fit into memory
        device = torch.device('cpu')
        unipose_model = unipose_model.to(device)

        updated_clips = []
        with torch.no_grad():
            for clip in tqdm(clips, desc='Extracting skeleton data', leave=False):
                info = self._extract_unipose_keypoints(unipose_model, device, clip)
                if info is not None:
                    updated_clips.append(info)

        return updated_clips

    def _extract_unipose_keypoints(self, unipose_model, device, clip):
        pedestrian_info = clip.reset_index().sort_values('frame')

        video_id = pedestrian_info.iloc[0]['video']
        pedestrian_id = pedestrian_info.iloc[0]['id']
        clip_id = pedestrian_info.iloc[0]['clip']
        start_frame = int(pedestrian_info.iloc[0]['frame'])
        end_frame = int(pedestrian_info.iloc[-1]['frame'] + 1)

        paths = glob.glob(os.path.join(
            self.datasets_dir, JAAD_DIR, 'videos', '{}.*'.format(os.path.splitext(video_id)[0])))

        if len(paths) != 1:
            # no video or multiple candidates - skip
            logging.getLogger(__name__).warn(
                "Clip extraction failed for {}, {}, {}".format(
                    video_id,
                    pedestrian_id,
                    clip_id))
            return None

        bboxes = pedestrian_info[['x1', 'y1', 'x2', 'y2']
                                 ].to_numpy().reshape((-1, 2, 2))

        with pims.PyAVReaderIndexed(paths[0]) as video:
            clip = video[start_frame:end_frame]
            clip_frames = np.array(clip)

        canvas, shifts = VideoMixin.crop_bbox(
            clip_frames, bboxes, target_size=self._target_size)

        # right now we have a canvas with all the frames of the clip
        # that hopefully contains the pedestrian in the center
        # it is now time to run the unipose model

        # prepare the input
        tensor_canvas = self._video_transform(canvas).to(device)

        heatmaps = unipose_model(tensor_canvas)

        clip_keypoints = self._keypoints_from_heatmaps(
            heatmaps.cpu().numpy(),
            canvas.shape[-3:-1],
            shifts
        )

        for idx, keypoints in enumerate(clip_keypoints):
            pedestrian_info.at[pedestrian_info.index[idx],
                               'keypoints'] = keypoints

        # everything went well, append the clip to the list
        return pedestrian_info

    def _load_unipose_model(self, unipose_model_path):
        """
        Load the Unipose model.
        """
        from pedestrians_video_2_carla.data.unipose.unipose_coco import UniPoseCOCO

        return UniPoseCOCO(unipose_model_path)

    def _keypoints_from_heatmaps(self, heatmaps, bbox_size, bbox_shifts):
        """
        Convert heatmaps to keypoints.
        """
        keypoints = np.zeros((len(heatmaps), len(self.data_nodes), 3), dtype=np.float32)
        (bbox_width, bbox_height) = bbox_size

        for i, heatmap in enumerate(heatmaps):
            (x_shift, y_shift) = bbox_shifts[i]
            kpts = []
            for m in heatmap[1:]:
                h, w = np.unravel_index(m.argmax(), m.shape)
                x = int(w * bbox_width / m.shape[1]) + x_shift
                y = int(h * bbox_height / m.shape[0]) + y_shift
                c = m.max()
                if c > 0:
                    kpts.append((x, y, c))
                else:
                    kpts.append((0, 0, 0))

            # points are detected in the order other than standard COCO order
            # this is a bit of a guesswork, because I couldn't find a reference
            # no neck or nose
            unipose_order = tuple([n.value for n in (
                COCO_SKELETON.LEye,  # 0
                COCO_SKELETON.REye,  # 1
                COCO_SKELETON.LEar,  # 2
                COCO_SKELETON.REar,  # 3
                COCO_SKELETON.LShoulder,  # 4
                COCO_SKELETON.RShoulder,  # 5
                COCO_SKELETON.LElbow,  # 6
                COCO_SKELETON.RElbow,  # 7
                COCO_SKELETON.LWrist,  # 8
                COCO_SKELETON.RWrist,  # 9
                COCO_SKELETON.LHip,  # 10
                COCO_SKELETON.RHip,  # 11
                COCO_SKELETON.LKnee,  # 12
                COCO_SKELETON.RKnee,  # 13
                COCO_SKELETON.LAnkle,  # 14
                COCO_SKELETON.RAnkle,  # 15
            )])
            keypoints[i, unipose_order] = kpts

        return keypoints.tolist()


if __name__ == "__main__":
    dm = JAADUniPoseDataModule(input_nodes=COCO_SKELETON,
                               fast_dev_run=True, clip_length=180, clip_offset=1800)
    dm.prepare_data()
