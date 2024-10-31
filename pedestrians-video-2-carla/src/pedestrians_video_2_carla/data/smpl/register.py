from pedestrians_video_2_carla.data import register_datamodule
from pedestrians_video_2_carla.data.smpl.amass_datamodule import AMASSDataModule

register_datamodule("AMASS", AMASSDataModule)
