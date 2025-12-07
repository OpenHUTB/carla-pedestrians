from pedestrians_scenarios.karma.pose.skeleton import CARLA_SKELETON
from pedestrians_video_2_carla.data.base.skeleton import register_skeleton

register_skeleton('CARLA_SKELETON', CARLA_SKELETON, [
    (k, k) for k in CARLA_SKELETON
])
