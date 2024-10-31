from pedestrians_video_2_carla.data import register_datamodule
from .mpii_datamodule import MPIIDataModule

register_datamodule("MPII", MPIIDataModule)
