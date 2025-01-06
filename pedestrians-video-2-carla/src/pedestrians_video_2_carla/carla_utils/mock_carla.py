"""
    此模块包含模拟 carla 类，这些类对于其余代码的运行必不可少。（没有没有安装carla模块，就只在这复制，不在模拟器里执行）
    它适用于只需要导入 carla 但不执行实际 Carla 相关代码的模块，
    以及仅设置基本变换Transforms、位置Locations和旋转Rotations的模块。
"""


class Transform(object):
    """
        此类是 carla.Transform 类的模拟。它仅用于其余代码的运行。
    """

    def __init__(self, location=None, rotation=None):
        self.location = location if location is not None else Location()
        self.rotation = rotation if rotation is not None else Rotation()


class Location(object):
    """
        此类是 carla.Location 类的模拟。它仅用于其余代码的运行。
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z


class Rotation(object):
    """
        此类是 carla.Rotation 类的模拟。它仅用于其余代码的运行。
    """

    def __init__(self, pitch: float = 0.0, yaw: float = 0.0, roll: float = 0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
