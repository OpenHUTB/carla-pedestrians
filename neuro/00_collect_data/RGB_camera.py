# 参考：https://github.com/OpenHUTB/carla_doc/blob/master/src/tutorial/03_RGB_camera.py
import os.path
import shutil

import carla

import random
import queue
import time

import cv2
import numpy as np


# 第一部分

# 连接到 Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# 设置仿真器到同步模式
# settings = world.get_settings()
# settings.synchronous_mode = True  # 启用同步模式
# settings.fixed_delta_seconds = 0.05
# world.apply_settings(settings)


# 获取世界中所有交通灯
traffic_lights = world.get_actors().filter('traffic.traffic_light*')

# 将所有交通灯设置为绿灯（为了防止在收集数据过程中车辆的停顿）
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


## IMU 数据获取
# 定义 IMU 刷新速率和对应等待时间（两次数据的最小间隔时间）
imu_rate  = 60  # 采样率
imu_per   = 1 / imu_rate  # IMU period

# 保存结果的时间
save_time = 100000

# 保存数据所需的列表长度
imu_len   = save_time * imu_rate

imu_std_dev_a     = 0.1      # IMU 加速度计 accel 标准差： 0.1
imu_std_dev_g     = 0.001    # gyro 陀螺仪

# 为传感器重置数据内存
imu_list = []
real_pos = []


# IMU 数据监听
def imu_listener(data, imu_queue):
    if (len(imu_list) < imu_len):
        accel = data.accelerometer
        gyro = data.gyroscope

        imu_list.append(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), data.timestamp))
        imu_queue.put(data)
        # print(accel)


# 生成 1 个 IMU 传感器
imu_bp = world.get_blueprint_library().find("sensor.other.imu")
imu_bp.set_attribute('sensor_tick', '0.1')
imu_bp.set_attribute('noise_accel_stddev_y', str(imu_std_dev_a))
imu_bp.set_attribute('noise_accel_stddev_x', str(imu_std_dev_a))
imu_bp.set_attribute('noise_accel_stddev_z', str(imu_std_dev_a))
imu_bp.set_attribute('noise_gyro_stddev_y',  str(imu_std_dev_g))
imu_bp.set_attribute('noise_gyro_stddev_x',  str(imu_std_dev_g))
imu_bp.set_attribute('noise_gyro_stddev_z',  str(imu_std_dev_g))
# imu_tf = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))


# 生成传感器 Spawning the sensor and appending to list
imu_sensor = world.spawn_actor(imu_bp, camera_init_trans, attach_to=vehicle)

# 开始IMU记录
imu_queue = queue.Queue()
# imu_sensor.listen(lambda image: camera_callback(image, rgb_image_queue))
imu_sensor.listen(lambda imu: imu_listener(imu, imu_queue))
# imu_sensor.listen(imu_listener)


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


def save_IMU_to_file(file_name, contents):
    with open(file_name, 'a+') as fh:
        fh.write(contents)
        fh.close()


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

        if not rgb_image_queue.empty():
            cur_img = rgb_image_queue.get()
        # 显示 RGB 相机图像
            cv2.imshow('RGB Camera', cur_img)

            img_idx = img_idx + 1
            if img_idx % 5 == 0:
                save_idx = save_idx + 1
                save_image_with_resolution_cv(cur_img, f"{output_dir}{save_idx:04d}.png", 160, 120)

        if not imu_queue.empty():
            cur_imu = imu_queue.get()
            print(cur_imu)
            save_IMU_to_file(f"{output_dir}IMU.txt", "%d, %5f, %5f, %5f, %5f, %5f, %5f, %5f\n" % (cur_imu.frame, cur_imu.timestamp,
                cur_imu.accelerometer.x, cur_imu.accelerometer.y, cur_imu.accelerometer.z,
                cur_imu.gyroscope.x, cur_imu.gyroscope.y, cur_imu.gyroscope.z)
                             )

        if save_idx >= 5000:
            break

        # 如果用户按 'q' 则退出
        if cv2.waitKey(1) == ord('q'):
            clear()
            break
        # time.sleep(0.005)
        # world.tick()

    except KeyboardInterrupt as e:
        settings = world.get_settings()
        settings.synchronous_mode = False  # 禁用异步模式
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        clear()
        break
