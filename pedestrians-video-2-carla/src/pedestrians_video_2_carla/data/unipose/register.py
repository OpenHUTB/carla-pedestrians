from pedestrians_video_2_carla.data import register_datamodule
from pedestrians_video_2_carla.data.unipose.jaad_unipose_datamodule import JAADUniPoseDataModule

register_datamodule("JAADUniPose", JAADUniPoseDataModule)
