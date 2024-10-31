from enum import Enum


class BaseTransforms(Enum):
    none = 0
    hips_neck = 1
    bbox = 2
    hips_neck_bbox = 3

    user_defined = 100
