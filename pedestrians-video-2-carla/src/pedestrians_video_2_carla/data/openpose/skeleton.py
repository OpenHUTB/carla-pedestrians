from typing import Dict, List, Tuple

from pedestrians_video_2_carla.data.base.skeleton import register_skeleton, Skeleton
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON


class BODY_25_SKELETON(Skeleton):
    Nose = 0
    Neck = 1
    RShoulder = 2
    RElbow = 3
    RWrist = 4
    LShoulder = 5
    LElbow = 6
    LWrist = 7
    MidHip = 8
    RHip = 9
    RKnee = 10
    RAnkle = 11
    LHip = 12
    LKnee = 13
    LAnkle = 14
    REye = 15
    LEye = 16
    REar = 17
    LEar = 18
    LBigToe = 19
    LSmallToe = 20
    LHeel = 21
    RBigToe = 22
    RSmallToe = 23
    RHeel = 24

    @classmethod
    def get_colors(cls) -> Dict['BODY_25_SKELETON', Tuple[int, int, int, int]]:
        # try to match OpenPose color scheme for easier visual comparison
        return {
            BODY_25_SKELETON.Nose: (255, 0, 85, 255),
            BODY_25_SKELETON.Neck: (255, 0, 0, 192),
            BODY_25_SKELETON.RShoulder: (255, 85, 0, 255),
            BODY_25_SKELETON.RElbow: (255, 170, 0, 255),
            BODY_25_SKELETON.RWrist: (255, 255, 0, 255),
            BODY_25_SKELETON.LShoulder: (170, 255, 0, 255),
            BODY_25_SKELETON.LElbow: (85, 255, 0, 255),
            BODY_25_SKELETON.LWrist: (0, 255, 0, 255),
            BODY_25_SKELETON.MidHip: (255, 0, 0, 255),
            BODY_25_SKELETON.RHip: (0, 255, 85, 255),
            BODY_25_SKELETON.RKnee: (0, 255, 170, 255),
            BODY_25_SKELETON.RAnkle: (0, 255, 255, 255),
            BODY_25_SKELETON.LHip: (0, 170, 255, 255),
            BODY_25_SKELETON.LKnee: (0, 85, 255, 255),
            BODY_25_SKELETON.LAnkle: (0, 0, 255, 255),
            BODY_25_SKELETON.REye: (255, 0, 170, 255),
            BODY_25_SKELETON.LEye: (170, 0, 255, 255),
            BODY_25_SKELETON.REar: (255, 0, 255, 255),
            BODY_25_SKELETON.LEar: (85, 0, 255, 255),
            BODY_25_SKELETON.LBigToe: (0, 0, 255, 255),
            BODY_25_SKELETON.LSmallToe: (0, 0, 255, 255),
            BODY_25_SKELETON.LHeel: (0, 0, 255, 255),
            BODY_25_SKELETON.RBigToe: (0, 255, 255, 255),
            BODY_25_SKELETON.RSmallToe: (0, 255, 255, 255),
            BODY_25_SKELETON.RHeel: (0, 255, 255, 255),
        }

    @classmethod
    def get_neck_point(cls) -> 'BODY_25_SKELETON':
        return BODY_25_SKELETON.Neck

    @classmethod
    def get_hips_point(cls) -> 'BODY_25_SKELETON':
        return BODY_25_SKELETON.MidHip

    @classmethod
    def get_flip_mask(cls) -> Tuple[int]:
        return (
            BODY_25_SKELETON.Nose.value,
            BODY_25_SKELETON.Neck.value,
            BODY_25_SKELETON.LShoulder.value,
            BODY_25_SKELETON.LElbow.value,
            BODY_25_SKELETON.LWrist.value,
            BODY_25_SKELETON.RShoulder.value,
            BODY_25_SKELETON.RElbow.value,
            BODY_25_SKELETON.RWrist.value,
            BODY_25_SKELETON.MidHip.value,
            BODY_25_SKELETON.LHip.value,
            BODY_25_SKELETON.LKnee.value,
            BODY_25_SKELETON.LAnkle.value,
            BODY_25_SKELETON.RHip.value,
            BODY_25_SKELETON.RKnee.value,
            BODY_25_SKELETON.RAnkle.value,
            BODY_25_SKELETON.LEye.value,
            BODY_25_SKELETON.REye.value,
            BODY_25_SKELETON.LEar.value,
            BODY_25_SKELETON.REar.value,
            BODY_25_SKELETON.RBigToe.value,
            BODY_25_SKELETON.RSmallToe.value,
            BODY_25_SKELETON.RHeel.value,
            BODY_25_SKELETON.LBigToe.value,
            BODY_25_SKELETON.LSmallToe.value,
            BODY_25_SKELETON.LHeel.value,
        )

    @classmethod
    def get_edges(cls) -> List[Tuple['BODY_25_SKELETON', 'BODY_25_SKELETON']]:
        return [
            (BODY_25_SKELETON.Nose, BODY_25_SKELETON.Neck),
            (BODY_25_SKELETON.Neck, BODY_25_SKELETON.RShoulder),
            (BODY_25_SKELETON.Neck, BODY_25_SKELETON.LShoulder),
            (BODY_25_SKELETON.RShoulder, BODY_25_SKELETON.RElbow),
            (BODY_25_SKELETON.RElbow, BODY_25_SKELETON.RWrist),
            (BODY_25_SKELETON.LShoulder, BODY_25_SKELETON.LElbow),
            (BODY_25_SKELETON.LElbow, BODY_25_SKELETON.LWrist),
            (BODY_25_SKELETON.Neck, BODY_25_SKELETON.MidHip),
            (BODY_25_SKELETON.MidHip, BODY_25_SKELETON.RHip),
            (BODY_25_SKELETON.RHip, BODY_25_SKELETON.RKnee),
            (BODY_25_SKELETON.RKnee, BODY_25_SKELETON.RAnkle),
            (BODY_25_SKELETON.MidHip, BODY_25_SKELETON.LHip),
            (BODY_25_SKELETON.LHip, BODY_25_SKELETON.LKnee),
            (BODY_25_SKELETON.LKnee, BODY_25_SKELETON.LAnkle),
            (BODY_25_SKELETON.Nose, BODY_25_SKELETON.REye),
            (BODY_25_SKELETON.REye, BODY_25_SKELETON.REar),
            (BODY_25_SKELETON.Nose, BODY_25_SKELETON.LEye),
            (BODY_25_SKELETON.LEye, BODY_25_SKELETON.LEar),
            (BODY_25_SKELETON.LAnkle, BODY_25_SKELETON.LHeel),
            (BODY_25_SKELETON.RAnkle, BODY_25_SKELETON.RHeel),
            (BODY_25_SKELETON.LAnkle, BODY_25_SKELETON.LBigToe),
            (BODY_25_SKELETON.LBigToe, BODY_25_SKELETON.LSmallToe),
            (BODY_25_SKELETON.LAnkle, BODY_25_SKELETON.LSmallToe),
            (BODY_25_SKELETON.RAnkle, BODY_25_SKELETON.RBigToe),
            (BODY_25_SKELETON.RBigToe, BODY_25_SKELETON.RSmallToe),
            (BODY_25_SKELETON.RAnkle, BODY_25_SKELETON.RSmallToe),
        ]


class COCO_SKELETON(Skeleton):
    Nose = 0
    Neck = 1
    RShoulder = 2
    RElbow = 3
    RWrist = 4
    LShoulder = 5
    LElbow = 6
    LWrist = 7
    RHip = 8
    RKnee = 9
    RAnkle = 10
    LHip = 11
    LKnee = 12
    LAnkle = 13
    REye = 14
    LEye = 15
    REar = 16
    LEar = 17

    @classmethod
    def get_colors(cls) -> Dict['COCO_SKELETON', Tuple[int, int, int, int]]:
        # try to match OpenPose color scheme for easier visual comparison
        return {
            COCO_SKELETON.Nose: (255, 0, 85, 255),
            COCO_SKELETON.Neck: (255, 0, 0, 192),
            COCO_SKELETON.RShoulder: (255, 85, 0, 255),
            COCO_SKELETON.RElbow: (255, 170, 0, 255),
            COCO_SKELETON.RWrist: (255, 255, 0, 255),
            COCO_SKELETON.LShoulder: (170, 255, 0, 255),
            COCO_SKELETON.LElbow: (85, 255, 0, 255),
            COCO_SKELETON.LWrist: (0, 255, 0, 255),
            COCO_SKELETON.RHip: (0, 255, 85, 255),
            COCO_SKELETON.RKnee: (0, 255, 170, 255),
            COCO_SKELETON.RAnkle: (0, 255, 255, 255),
            COCO_SKELETON.LHip: (0, 170, 255, 255),
            COCO_SKELETON.LKnee: (0, 85, 255, 255),
            COCO_SKELETON.LAnkle: (0, 0, 255, 255),
            COCO_SKELETON.REye: (255, 0, 170, 255),
            COCO_SKELETON.LEye: (170, 0, 255, 255),
            COCO_SKELETON.REar: (255, 0, 255, 255),
            COCO_SKELETON.LEar: (85, 0, 255, 255),
        }

    @classmethod
    def get_neck_point(cls) -> 'COCO_SKELETON':
        return COCO_SKELETON.Neck

    @classmethod
    def get_hips_point(cls) -> List['COCO_SKELETON']:
        return [COCO_SKELETON.LHip, COCO_SKELETON.RHip]

    @classmethod
    def get_flip_mask(cls) -> Tuple[int]:
        return (
            COCO_SKELETON.Nose.value,
            COCO_SKELETON.Neck.value,
            COCO_SKELETON.LShoulder.value,
            COCO_SKELETON.LElbow.value,
            COCO_SKELETON.LWrist.value,
            COCO_SKELETON.RShoulder.value,
            COCO_SKELETON.RElbow.value,
            COCO_SKELETON.RWrist.value,
            COCO_SKELETON.LHip.value,
            COCO_SKELETON.LKnee.value,
            COCO_SKELETON.LAnkle.value,
            COCO_SKELETON.RHip.value,
            COCO_SKELETON.RKnee.value,
            COCO_SKELETON.RAnkle.value,
            COCO_SKELETON.LEye.value,
            COCO_SKELETON.REye.value,
            COCO_SKELETON.LEar.value,
            COCO_SKELETON.REar.value,
        )

    @classmethod
    def get_edges(cls) -> List[Tuple['COCO_SKELETON', 'COCO_SKELETON']]:
        return [
            (COCO_SKELETON.Neck, COCO_SKELETON.Nose),
            (COCO_SKELETON.Neck, COCO_SKELETON.RShoulder),
            (COCO_SKELETON.Neck, COCO_SKELETON.LShoulder),
            (COCO_SKELETON.RShoulder, COCO_SKELETON.RElbow),
            (COCO_SKELETON.RElbow, COCO_SKELETON.RWrist),
            (COCO_SKELETON.LShoulder, COCO_SKELETON.LElbow),
            (COCO_SKELETON.LElbow, COCO_SKELETON.LWrist),
            (COCO_SKELETON.Neck, COCO_SKELETON.RHip),
            (COCO_SKELETON.RHip, COCO_SKELETON.RKnee),
            (COCO_SKELETON.RKnee, COCO_SKELETON.RAnkle),
            (COCO_SKELETON.Neck, COCO_SKELETON.LHip),
            (COCO_SKELETON.LHip, COCO_SKELETON.LKnee),
            (COCO_SKELETON.LKnee, COCO_SKELETON.LAnkle),
            (COCO_SKELETON.Nose, COCO_SKELETON.REye),
            (COCO_SKELETON.REye, COCO_SKELETON.REar),
            (COCO_SKELETON.Nose, COCO_SKELETON.LEye),
            (COCO_SKELETON.LEye, COCO_SKELETON.LEar),
        ]


register_skeleton('BODY_25_SKELETON', BODY_25_SKELETON, [
    (CARLA_SKELETON.crl_hips__C, BODY_25_SKELETON.MidHip),
    (CARLA_SKELETON.crl_arm__L, BODY_25_SKELETON.LShoulder),
    (CARLA_SKELETON.crl_foreArm__L, BODY_25_SKELETON.LElbow),
    (CARLA_SKELETON.crl_hand__L, BODY_25_SKELETON.LWrist),
    (CARLA_SKELETON.crl_neck__C, BODY_25_SKELETON.Neck),
    (CARLA_SKELETON.crl_Head__C, BODY_25_SKELETON.Nose),
    (CARLA_SKELETON.crl_arm__R, BODY_25_SKELETON.RShoulder),
    (CARLA_SKELETON.crl_foreArm__R, BODY_25_SKELETON.RElbow),
    (CARLA_SKELETON.crl_hand__R, BODY_25_SKELETON.RWrist),
    (CARLA_SKELETON.crl_eye__L, BODY_25_SKELETON.LEye),
    (CARLA_SKELETON.crl_eye__R, BODY_25_SKELETON.REye),
    (CARLA_SKELETON.crl_thigh__R, BODY_25_SKELETON.RHip),
    (CARLA_SKELETON.crl_leg__R, BODY_25_SKELETON.RKnee),
    (CARLA_SKELETON.crl_foot__R, BODY_25_SKELETON.RAnkle),
    (CARLA_SKELETON.crl_toe__R, BODY_25_SKELETON.RBigToe),
    (CARLA_SKELETON.crl_toeEnd__R, BODY_25_SKELETON.RSmallToe),
    (CARLA_SKELETON.crl_thigh__L, BODY_25_SKELETON.LHip),
    (CARLA_SKELETON.crl_leg__L, BODY_25_SKELETON.LKnee),
    (CARLA_SKELETON.crl_foot__L, BODY_25_SKELETON.LAnkle),
    (CARLA_SKELETON.crl_toe__L, BODY_25_SKELETON.LBigToe),
    (CARLA_SKELETON.crl_toeEnd__L, BODY_25_SKELETON.LSmallToe),
])

register_skeleton('COCO_SKELETON', COCO_SKELETON, [
    (CARLA_SKELETON.crl_arm__L, COCO_SKELETON.LShoulder),
    (CARLA_SKELETON.crl_foreArm__L, COCO_SKELETON.LElbow),
    (CARLA_SKELETON.crl_hand__L, COCO_SKELETON.LWrist),
    (CARLA_SKELETON.crl_neck__C, COCO_SKELETON.Neck),
    (CARLA_SKELETON.crl_Head__C, COCO_SKELETON.Nose),
    (CARLA_SKELETON.crl_arm__R, COCO_SKELETON.RShoulder),
    (CARLA_SKELETON.crl_foreArm__R, COCO_SKELETON.RElbow),
    (CARLA_SKELETON.crl_hand__R, COCO_SKELETON.RWrist),
    (CARLA_SKELETON.crl_eye__L, COCO_SKELETON.LEye),
    (CARLA_SKELETON.crl_eye__R, COCO_SKELETON.REye),
    (CARLA_SKELETON.crl_thigh__R, COCO_SKELETON.RHip),
    (CARLA_SKELETON.crl_leg__R, COCO_SKELETON.RKnee),
    (CARLA_SKELETON.crl_foot__R, COCO_SKELETON.RAnkle),
    (CARLA_SKELETON.crl_thigh__L, COCO_SKELETON.LHip),
    (CARLA_SKELETON.crl_leg__L, COCO_SKELETON.LKnee),
    (CARLA_SKELETON.crl_foot__L, COCO_SKELETON.LAnkle),
])
