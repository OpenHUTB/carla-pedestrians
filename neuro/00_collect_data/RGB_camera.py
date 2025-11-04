import os.path
import shutil
import math  

import carla

import random
import queue
import time

import cv2
import numpy as np


# -------------------------- 新增1：相机防过曝配置参数 --------------------------
EXPOSURE_MODE = "manual"          
EXPOSURE_COMPENSATION = "0"       
FSTOP = "4.0"                     
ISO = "400"                       
GAMMA = "2.2"                     
BLOOM_INTENSITY = "0.0"            
CHROMATIC_ABERRATION = "0.0"      
LENS_FLARE = "0.0"                
# ---------------------------------------------------------------------------------------------------


# -------------------------- 新增2：全局变量（碰撞重生标记） --------------------------
need_respawn = False  
# ---------------------------------------------------------------------------------------------------


# 第一部分：连接Carla（新增：地图加载延迟+同步模式配置）
client = carla.Client('localhost', 2000)
client.set_timeout(20.0)  # 新增：延长超时时间，避免连接失败
world = client.get_world()

# -------------------------- 新增3：地图加载延迟+同步模式启用（核心稳定项） --------------------------
# 确保地图完全加载（避免传感器绑定到未就绪资源）
current_map = world.get_map().name
print(f"当前地图：{current_map}，等待1.5秒确保加载完成...")
time.sleep(1.5)  # 给地图资源初始化留时间

# 启用同步模式（避免传感器数据与世界帧异步导致崩溃）
settings = world.get_settings()
settings.synchronous_mode = True  # 强制同步：世界帧与传感器数据对齐
settings.fixed_delta_seconds = 0.05  # 固定帧间隔（与传感器采样频率匹配）
world.apply_settings(settings)
print("已启用CARLA同步模式，帧间隔0.05秒")
# ---------------------------------------------------------------------------------------------------

traffic_lights = world.get_actors().filter('traffic.traffic_light*')
for tl in traffic_lights:
    tl.set_state(carla.TrafficLightState.Green)
    tl.freeze(True)

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
spawn_points = world.get_map().get_spawn_points()
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
vehicle.set_autopilot(True)
spectator = world.get_spectator() 


# 第二部分：相机配置、回调（原逻辑不变）
camera_init_trans = carla.Transform(carla.Location(x=0.2, y=0, z=4.2), carla.Rotation(pitch=-20))
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
# 补充相机分辨率（原代码未显式设置，避免默认分辨率不兼容）
rgb_camera_bp.set_attribute("image_size_x", "640")
rgb_camera_bp.set_attribute("image_size_y", "480")
rgb_camera_bp.set_attribute("sensor_tick", "0.05")  # 与同步帧间隔一致
# 防过曝参数（原逻辑不变）
rgb_camera_bp.set_attribute("exposure_mode", EXPOSURE_MODE)
rgb_camera_bp.set_attribute("exposure_compensation", EXPOSURE_COMPENSATION)
rgb_camera_bp.set_attribute("fstop", FSTOP)
rgb_camera_bp.set_attribute("iso", ISO)
rgb_camera_bp.set_attribute("gamma", GAMMA)
rgb_camera_bp.set_attribute("bloom_intensity", BLOOM_INTENSITY)
rgb_camera_bp.set_attribute("chromatic_aberration_intensity", CHROMATIC_ABERRATION)
rgb_camera_bp.set_attribute("lens_flare_intensity", LENS_FLARE)
camera = world.spawn_actor(rgb_camera_bp, camera_init_trans, attach_to=vehicle)

def camera_callback(image, rgb_image_queue):
    rgb_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

image_w = rgb_camera_bp.get_attribute("image_size_x").as_int()
image_h = rgb_camera_bp.get_attribute("image_size_y").as_int()
rgb_image_queue = queue.Queue()
camera.listen(lambda image: camera_callback(image, rgb_image_queue))


## IMU 数据获取（原逻辑不变，仅修正sensor_tick与同步帧匹配）
imu_rate  = 60
imu_per   = 1 / imu_rate
save_time = 100000
imu_len   = save_time * imu_rate
imu_std_dev_a     = 0.1
imu_std_dev_g     = 0.001
imu_list = []
real_pos = []

def imu_listener(data, imu_queue):
    if (len(imu_list) < imu_len):
        accel = data.accelerometer
        gyro = data.gyroscope
        imu_list.append(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), data.timestamp))
        imu_queue.put(data)

imu_bp = world.get_blueprint_library().find("sensor.other.imu")
imu_bp.set_attribute('sensor_tick', str(imu_per))  # 修正：与IMU速率60Hz匹配（原0.1秒=10Hz，不兼容）
imu_bp.set_attribute('noise_accel_stddev_y', str(imu_std_dev_a))
imu_bp.set_attribute('noise_accel_stddev_x', str(imu_std_dev_a))
imu_bp.set_attribute('noise_accel_stddev_z', str(imu_std_dev_a))
imu_bp.set_attribute('noise_gyro_stddev_y',  str(imu_std_dev_g))
imu_bp.set_attribute('noise_gyro_stddev_x',  str(imu_std_dev_g))
imu_bp.set_attribute('noise_gyro_stddev_z',  str(imu_std_dev_g))
imu_sensor = world.spawn_actor(imu_bp, camera_init_trans, attach_to=vehicle)
imu_queue = queue.Queue()
imu_sensor.listen(lambda imu: imu_listener(imu, imu_queue))


# -------------------------- 碰撞回调（原逻辑不变） --------------------------
def collision_callback(event):
    global need_respawn
    impulse = event.normal_impulse
    impulse_mag = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
    
    if (event.actor.id == vehicle.id or event.other_actor.id == vehicle.id) and vehicle.is_alive:
        if impulse_mag > 10.0:  
            print(f"\n检测到有效碰撞（冲量: {impulse_mag:.2f}）！碰撞对象: {event.other_actor.type_id}")
            need_respawn = True

collision_bp = world.get_blueprint_library().find('sensor.other.collision')
collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
collision_sensor.listen(collision_callback)
print("碰撞传感器初始化完成，已开启碰撞检测")
# ---------------------------------------------------------------------------------------------------


# -------------------------- 重生函数（原逻辑不变，仅补充同步模式重置） --------------------------
def respawn_vehicle():
    global vehicle, camera, imu_sensor, collision_sensor, need_respawn

    # 先停止所有传感器监听
    if camera and camera.is_alive:
        camera.stop()
    if imu_sensor and imu_sensor.is_alive:
        imu_sensor.stop()
    if collision_sensor and collision_sensor.is_alive:
        collision_sensor.stop()
    time.sleep(0.5)

    # 强制销毁所有传感器
    try:
        camera.destroy()
    except:
        pass
    try:
        imu_sensor.destroy()
    except:
        pass
    try:
        collision_sensor.destroy()
    except:
        pass
    camera = None
    imu_sensor = None
    collision_sensor = None
    time.sleep(0.5)

    # 强制销毁车辆+清理残留
    try:
        vehicle.destroy()
    except:
        pass
    for actor in world.get_actors().filter(vehicle_bp.id):
        try:
            actor.destroy()
        except:
            pass
    vehicle = None
    time.sleep(1.5)

    print("旧车辆和传感器已彻底清理")

    # 重新生成车辆
    max_attempts = 10
    new_vehicle = None
    for attempt in range(max_attempts):
        chosen_spawn = random.choice(spawn_points)
        new_vehicle = world.try_spawn_actor(vehicle_bp, chosen_spawn)
        if new_vehicle:
            new_vehicle.set_autopilot(True)
            print(f"第{attempt+1}次尝试成功，新车辆生成于: ({chosen_spawn.location.x:.1f}, {chosen_spawn.location.y:.1f})")
            break
        print(f"第{attempt+1}次生成失败，换生成点重试...")
        time.sleep(0.5)
    if not new_vehicle:
        print("连续10次生成失败，程序退出")
        clear()
        exit()
    vehicle = new_vehicle

    # 重新生成传感器
    camera = world.spawn_actor(rgb_camera_bp, camera_init_trans, attach_to=vehicle)
    camera.listen(lambda image: camera_callback(image, rgb_image_queue))
    imu_sensor = world.spawn_actor(imu_bp, camera_init_trans, attach_to=vehicle)
    imu_sensor.listen(lambda imu: imu_listener(imu, imu_queue))
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
    collision_sensor.listen(collision_callback)

    print("新车辆的传感器已重新初始化，恢复数据采集\n")
    need_respawn = False

# ---------------------------------------------------------------------------------------------------


# 清理函数、图像/IMU保存函数（原逻辑不变）
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

def clear():
    # 新增：清理前先恢复同步模式为异步，避免CARLA残留设置
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    
    if vehicle and vehicle.is_alive:
        vehicle.destroy()
        print('Vehicle Destroyed.')
    if camera and camera.is_alive:
        camera.stop()
        camera.destroy()
        print('Camera Destroyed.')
    if imu_sensor and imu_sensor.is_alive:
        imu_sensor.stop()
        imu_sensor.destroy()
        print('IMU Sensor Destroyed.')
    if collision_sensor and collision_sensor.is_alive:
        collision_sensor.stop()
        collision_sensor.destroy()
        print('Collision Sensor Destroyed.')
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    cv2.destroyAllWindows()

def save_image_with_resolution_cv(input_img, output_path, target_width, target_height):
    resized_img = cv2.resize(input_img, (target_width, target_height), interpolation=cv2.INTER_LANCZOS4)
    cv2.imwrite(output_path, resized_img)
    print(f"图片已保存为 {target_width}x{target_height} 分辨率到 {output_path}")

def save_IMU_to_file(file_name, contents):
    with open(file_name, 'a+') as fh:
        fh.write(contents)
        fh.close()

img_idx = 0
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

while True:
    try:
        # 新增：同步模式必须调用world.tick()，触发世界帧更新
        world.tick()  # 核心：没有这行，传感器数据会卡死
        
        if need_respawn:
            respawn_vehicle()
        if vehicle and vehicle.is_alive:
            transform = carla.Transform(
                vehicle.get_transform().transform(carla.Location(x=-4, z=50)), 
                carla.Rotation(yaw=-180, pitch=-90)
            )
            spectator.set_transform(transform)
        if not rgb_image_queue.empty():
            cur_img = rgb_image_queue.get()
            cv2.imshow('RGB Camera', cur_img)
            img_idx = img_idx + 1
            if img_idx % 5 == 0:
                save_idx = save_idx + 1
                save_image_with_resolution_cv(cur_img, f"{output_dir}{save_idx:04d}.png", 160, 120)
        if not imu_queue.empty():
            cur_imu = imu_queue.get()
            print(cur_imu)
            save_IMU_to_file(f"{output_dir}IMU.txt", "%d, %5f, %5f, %5f, %5f, %5f, %5f, %5f\n" % (
                cur_imu.frame, cur_imu.timestamp,
                cur_imu.accelerometer.x, cur_imu.accelerometer.y, cur_imu.accelerometer.z,
                cur_imu.gyroscope.x, cur_imu.gyroscope.y, cur_imu.gyroscope.z
            ))
        if save_idx >= 5000:
            break
        if cv2.waitKey(1) == ord('q'):
            clear()
            break
    except KeyboardInterrupt as e:
        clear()  # 直接调用清理函数（已包含同步模式恢复）
        break
    except RuntimeError as e:
        if "destroyed actor" in str(e):
            print(f"\n捕获车辆销毁异常：{e}，触发强制重生...")
            need_respawn = True
            time.sleep(1)
        else:
            clear()
            break
