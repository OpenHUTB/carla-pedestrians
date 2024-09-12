import carla
import random
import time
import numpy as np
import pygame
import cv2

# 初始化 pygame
pygame.init()
display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
clock = pygame.time.Clock()


def main():
    # 连接到CARLA服务器 (默认是localhost:2000)
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # 加载一个小型的默认地图 (Town01 是较小的地图)
    # world = client.load_world('Town01')
    world = client.get_world()
    # 获取蓝图库，用于生成行人和摄像机
    blueprint_library = world.get_blueprint_library()

    # 设置天气和时间
    weather = carla.WeatherParameters(
        cloudiness=10.0,
        precipitation=0.0,
        sun_altitude_angle=70.0
    )
    world.set_weather(weather)

    # 删除世界中的所有车辆和行人
    vehicle_list = world.get_actors().filter('vehicle.*')
    for vehicle in vehicle_list:
        vehicle.destroy()

    walker_list = world.get_actors().filter('walker.*')
    for walker in walker_list:
        walker.destroy()

    # 生成一个行人
    walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))

    # 在地图上生成行人
    start_location = carla.Transform(carla.Location(x=0, y=0, z=1), carla.Rotation(yaw=0))  # 行人起点
    walker = world.try_spawn_actor(walker_bp, start_location)

    control = carla.WalkerControl(carla.Vector3D(x=5), speed=1)
    walker.apply_control(control)

    if walker is None:
        print("行人生成失败")
        return


    # --- 添加第三人称视角摄像机，并跟随行人 ---


    # #一号相机
    # camera_bp = blueprint_library.find('sensor.camera.rgb')
    #  # 摄像机放置在行人后方，并稍微向下倾斜
    # camera_transform = carla.Transform(
    #     carla.Location(x=-3.0, y=0, z=2.5),  # 摄像机放在行人后方3米，高度为2.5米
    #     carla.Rotation(pitch=-20)  # 向下倾斜20度，以确保看到行人
    # )
    # # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=walker)  # 将摄像机附加到行人
    # camera = world.spawn_actor(camera_bp, camera_transform)  #new


    # # 二号相机
    # camera_bp = blueprint_library.find('sensor.camera.rgb')
    # # 摄像机放置在行人后方，并稍微向下倾斜
    # camera_transform = carla.Transform(
    #     carla.Location(x=10, y=5, z=10),
    #     #carla.Rotation(pitch=-30)  # 向下倾斜30度，以确保看到行人
    #     carla.Rotation(-50,-90,0)
    # )
    # camera = world.spawn_actor(camera_bp, camera_transform)  # new


    # # 三号相机
    # camera_bp = blueprint_library.find('sensor.camera.rgb')
    # # 摄像机放置在行人后方，并稍微向下倾斜
    # camera_transform = carla.Transform(
    #     carla.Location(x=20, y=-5, z=10),
    #     # carla.Rotation(pitch=-50)  # 向下倾斜50度，以确保看到行人
    #     carla.Rotation(-50, 90, 0)
    # )
    # camera = world.spawn_actor(camera_bp, camera_transform)  # new


    # 四号相机
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    # 摄像机放置在行人后方，并稍微向下倾斜
    camera_transform = carla.Transform(
        carla.Location(x=30, y=5, z=10),
        # carla.Rotation(pitch=-30)  # 向下倾斜30度，以确保看到行人
        carla.Rotation(-50, -90, 0)
    )
    camera = world.spawn_actor(camera_bp, camera_transform)  # new


    # # 五号相机
    # camera_bp = blueprint_library.find('sensor.camera.rgb')
    # # 摄像机放置在行人后方，并稍微向下倾斜
    # camera_transform = carla.Transform(
    #     carla.Location(x=40, y=-5, z=10),
    #     # carla.Rotation(pitch=-30)  # 向下倾斜30度，以确保看到行人
    #     carla.Rotation(-50, 90, 0)
    # )
    # camera = world.spawn_actor(camera_bp, camera_transform)  # new


    # # 六号相机
    # camera_bp = blueprint_library.find('sensor.camera.rgb')
    # # 摄像机放置在行人后方，并稍微向下倾斜
    # camera_transform = carla.Transform(
    #     carla.Location(x=50, y=0, z=10),
    #     # carla.Rotation(pitch=-30)  # 向下倾斜30度，以确保看到行人
    #     carla.Rotation(-50, 180, 0)
    # )
    # camera = world.spawn_actor(camera_bp, camera_transform)  # new

    # 初始化 OpenCV 视频写入器，使用 mp4v 编码器
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter('output.mp4', fourcc, 20.0, (800, 600))  # 20帧每秒，800x600分辨率

    # 定义处理图像帧的逻辑
    def process_image(image):
        # 将图像数据转化为 numpy 数组
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # 将其转化为 (height, width, 4)

        # 转换为 BGR 格式用于保存（OpenCV 使用 BGR）
        image_bgr = array[:, :, :3][:, :, ::-1]

        # 写入帧到视频文件
        video_writer.write(image_bgr)

    # 开始监听摄像头的图像帧
    camera.listen(lambda image: process_image(image))






    # # 处理摄像机捕获的图像并在 pygame 中显示
    # def process_image(image):
    #     # 将CARLA的图像数据转化为pygame可以显示的格式
    #     array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    #     array = np.reshape(array, (image.height, image.width, 4))  # RGBA格式
    #     array = array[:, :, :3]  # 去掉Alpha通道，保留RGB
    #     array = array[:, :, ::-1]  # 将BGR格式转化为RGB
    #     surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))  # 转换为pygame的surface
    #     display.blit(surface, (0, 0))  # 在窗口显示图像
    #     pygame.display.flip()  # 刷新窗口
    #
    # # 设置摄像机监听图像数据
    # camera.listen(lambda image: process_image(image))

    # 程序运行一段时间，展示行人行走的效果
    try:
        while True:
            clock.tick_busy_loop(60)  # 保持60 FPS
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
    finally:
        # 清理演员（删除行人和控制器）
        # walker_controller.stop()
        # walker.destroy()
        # walker_controller.destroy()
        camera.destroy()  # 销毁摄像机


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n脚本被用户中断.')
    finally:
        pygame.quit()
