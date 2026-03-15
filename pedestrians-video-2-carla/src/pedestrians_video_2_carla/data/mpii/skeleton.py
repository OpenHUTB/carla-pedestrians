from typing import Dict, List, Tuple
from pedestrians_video_2_carla.data.base.skeleton import Skeleton, register_skeleton
from pedestrians_video_2_carla.data.carla.skeleton import CARLA_SKELETON


class MPII_SKELETON(Skeleton):
    RAnkle = 0
    RKnee = 1
    RHip = 2
    LHip = 3
    LKnee = 4
    LAnkle = 5
    Pelvis = 6
    Thorax = 7
    Neck = 8
    Head = 9
    RWrist = 10
    RElbow = 11
    RShoulder = 12
    LShoulder = 13
    LElbow = 14
    LWrist = 15

    @classmethod
    def get_colors(cls) -> Dict['MPII_SKELETON', Tuple[int, int, int, int]]:
        # try to match OpenPose color scheme for easier visual comparison
        return {
            MPII_SKELETON.RAnkle: (0, 255, 255, 255),
            MPII_SKELETON.RKnee: (0, 255, 170, 255),
            MPII_SKELETON.RHip: (0, 255, 85, 255),
            MPII_SKELETON.LHip: (0, 170, 255, 255),
            MPII_SKELETON.LKnee: (0, 85, 255, 255),
            MPII_SKELETON.LAnkle: (0, 0, 255, 255),
            MPII_SKELETON.Pelvis: (255, 0, 0, 255),
            MPII_SKELETON.Thorax: (255, 0, 0, 192),
            MPII_SKELETON.Neck: (255, 0, 0, 192),
            MPII_SKELETON.Head: (255, 0, 85, 255),
            MPII_SKELETON.RWrist: (255, 255, 0, 255),
            MPII_SKELETON.RElbow: (255, 170, 0, 255),
            MPII_SKELETON.RShoulder: (255, 85, 0, 255),
            MPII_SKELETON.LShoulder: (170, 255, 0, 255),
            MPII_SKELETON.LElbow: (85, 255, 0, 255),
            MPII_SKELETON.LWrist: (0, 255, 0, 255),
        }

    @classmethod
    def get_neck_point(cls) -> 'MPII_SKELETON':
        return MPII_SKELETON.Neck

    @classmethod
    def get_hips_point(cls) -> 'MPII_SKELETON':
        return MPII_SKELETON.Pelvis

    @classmethod
    def get_flip_mask(cls) -> Tuple[int]:
        return (
            MPII_SKELETON.LAnkle,
            MPII_SKELETON.LKnee,
            MPII_SKELETON.LHip,
            MPII_SKELETON.RHip,
            MPII_SKELETON.RKnee,
            MPII_SKELETON.RAnkle,
            MPII_SKELETON.Pelvis,
            MPII_SKELETON.Thorax,
            MPII_SKELETON.Neck,
            MPII_SKELETON.Head,
            MPII_SKELETON.LWrist,
            MPII_SKELETON.LElbow,
            MPII_SKELETON.LShoulder,
            MPII_SKELETON.RShoulder,
            MPII_SKELETON.RElbow,
            MPII_SKELETON.RWrist,
        )

    @classmethod
    def get_edges(cls) -> List[Tuple['MPII_SKELETON', 'MPII_SKELETON']]:
        return [
            (MPII_SKELETON.Head, MPII_SKELETON.Neck),
            (MPII_SKELETON.Neck, MPII_SKELETON.RShoulder),
            (MPII_SKELETON.Neck, MPII_SKELETON.LShoulder),
            (MPII_SKELETON.RShoulder, MPII_SKELETON.RElbow),
            (MPII_SKELETON.RElbow, MPII_SKELETON.RWrist),
            (MPII_SKELETON.LShoulder, MPII_SKELETON.LElbow),
            (MPII_SKELETON.LElbow, MPII_SKELETON.LWrist),
            (MPII_SKELETON.Neck, MPII_SKELETON.Thorax),
            (MPII_SKELETON.Thorax, MPII_SKELETON.Pelvis),
            (MPII_SKELETON.Pelvis, MPII_SKELETON.RHip),
            (MPII_SKELETON.RHip, MPII_SKELETON.RKnee),
            (MPII_SKELETON.RKnee, MPII_SKELETON.RAnkle),
            (MPII_SKELETON.Pelvis, MPII_SKELETON.LHip),
            (MPII_SKELETON.LHip, MPII_SKELETON.LKnee),
            (MPII_SKELETON.LKnee, MPII_SKELETON.LAnkle),
        ]


register_skeleton('MPII_SKELETON', MPII_SKELETON, [
    (CARLA_SKELETON.crl_arm__L, MPII_SKELETON.LShoulder),
    (CARLA_SKELETON.crl_foreArm__L, MPII_SKELETON.LElbow),
    (CARLA_SKELETON.crl_hand__L, MPII_SKELETON.LWrist),
    (CARLA_SKELETON.crl_neck__C, MPII_SKELETON.Neck),
    (CARLA_SKELETON.crl_Head__C, MPII_SKELETON.Head),
    (CARLA_SKELETON.crl_arm__R, MPII_SKELETON.RShoulder),
    (CARLA_SKELETON.crl_foreArm__R, MPII_SKELETON.RElbow),
    (CARLA_SKELETON.crl_hand__R, MPII_SKELETON.RWrist),
    (CARLA_SKELETON.crl_hips__C, MPII_SKELETON.Pelvis),
    (CARLA_SKELETON.crl_thigh__R, MPII_SKELETON.RHip),
    (CARLA_SKELETON.crl_leg__R, MPII_SKELETON.RKnee),
    (CARLA_SKELETON.crl_foot__R, MPII_SKELETON.RAnkle),
    (CARLA_SKELETON.crl_thigh__L, MPII_SKELETON.LHip),
    (CARLA_SKELETON.crl_leg__L, MPII_SKELETON.LKnee),
    (CARLA_SKELETON.crl_foot__L, MPII_SKELETON.LAnkle),
])
