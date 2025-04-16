# 参考：https://github.com/OpenHUTB/carla_doc/blob/master/src/tutorial/03_RGB_camera.py
import os.path
import shutil

import carla

import random
import queue

import cv2
import numpy as np


# 第一部分

# 连接到 Carla
client = carla.Client('localhost', 2000)
world = client.get_world()


# 获取世界中所有交通灯
traffic_lights = world.get_actors().filter('traffic.traffic_light*')

# 将所有交通灯设置为绿灯
for tl in traffic_lights:
    tl.set_state(carla.TrafficLightState.Green)
    tl.freeze(True)  # 冻结状态，防止自动变化


# 从库中获得一个车辆
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# 得到一个生成点
spawn_points = world.get_map().get_spawn_points()

# 生成一辆车
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# 自动驾驶
vehicle.set_autopilot(True) 

# 获得世界的观察者
spectator = world.get_spectator() 

# 第二部分

# 创建一个漂浮在车辆后面的摄像头
camera_init_trans = carla.Transform(carla.Location(x=0.2, y=0, z=4.2), carla.Rotation(pitch=-20))

# 创建一个RGB相机
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(rgb_camera_bp, camera_init_trans, attach_to=vehicle)


# 回调将传感器数据存储在字典中，以供回调外部使用
def camera_callback(image, rgb_image_queue):
    rgb_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))


# 获得相机纬度并初始化字典
image_w = rgb_camera_bp.get_attribute("image_size_x").as_int()
image_h = rgb_camera_bp.get_attribute("image_size_y").as_int()

# 开始相机记录
rgb_image_queue = queue.Queue()
camera.listen(lambda image: camera_callback(image, rgb_image_queue))

# 为了渲染的 OpenCV 命名窗口
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)


# 清除生成的车辆和相机
def clear():
    vehicle.destroy()
    print('Vehicle Destroyed.')
    
    camera.stop()
    camera.destroy()
    print('Camera Destroyed. Bye!')

    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()

    cv2.destroyAllWindows()


def save_image_with_resolution_cv(input_img, output_path, target_width, target_height):
    """
    使用OpenCV将图片保存为指定分辨率
    """
    resized_img = cv2.resize(input_img, (target_width, target_height), interpolation=cv2.INTER_LANCZOS4)
    cv2.imwrite(output_path, resized_img)
    print(f"图片已保存为 {target_width}x{target_height} 分辨率到 {output_path}")


# 经过多少帧的id
img_idx = 0
# 保存图片的id
save_idx = 0
output_dir = '../data/01_NeuroSLAM_Datasets/00_CarlaData/'

if not os.path.exists(output_dir):
    os.mkdir(output_dir)
    print(f"Directory {output_dir} has been created.")
else:
    shutil.rmtree(output_dir)
    print(f"Directory {output_dir} has been remove.")
    os.mkdir(output_dir)
    print(f"Directory {output_dir} has been created.")
# 主循环
while True:
    try:
        # 观察者移动到车辆上方
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4, z=50)), carla.Rotation(yaw=-180, pitch=-90))
        spectator.set_transform(transform)

        cur_img = rgb_image_queue.get()
        # 显示 RGB 相机图像
        cv2.imshow('RGB Camera', cur_img)

        img_idx = img_idx + 1
        if img_idx % 5 == 0:
            save_idx = save_idx + 1
            save_image_with_resolution_cv(cur_img, f"{output_dir}{save_idx:04d}.png", 160, 120)

        if save_idx >= 5000:
            break

        # 如果用户按 'q' 则退出
        if cv2.waitKey(1) == ord('q'):
            clear()
            break

    except KeyboardInterrupt as e:
        clear()
        break
