# 参考：https://github.com/Ashwin-Rajesh/Kalman_filter_carla
import matplotlib.pyplot as plt
import numpy as np

import time

import agent
import integrate
import kalman_filter


# 定义 IMU 刷新速率和对应等待时间（两次数据的最小间隔时间）
imu_rate  = 60  # 采样率
imu_per   = 1 / imu_rate

# 定义 GNSS 刷新速率和对应等待时间
gnss_rate = 5  # 采样率
gnss_per  = 1 / gnss_rate

# 保存结果的时间
save_time = 100

# 保存数据所需的列表长度
imu_len   = save_time * imu_rate
gnss_len  = save_time * gnss_rate

imu_std_dev_a     = 0.1      # IMU 标准差： 0.1
imu_std_dev_g     = 0.001
gnss_std_dev_geo  = 3e-5
gnss_std_dev_xy   = (np.pi * gnss_std_dev_geo / 180) * 6.357e6

print("GNSS std deviation : %.3fm"%gnss_std_dev_xy)  # 全球卫星导航系统（GNSS）的标准差：3.33


# 创建代理
a = agent.agent()
d = a.world.debug

# 生成车辆和传感器
a.spawn_vehicle(1)

# 有时自动驾驶仪未启动
a.vehicle.set_autopilot(True)

# 为传感器重置数据内存
imu_list = []
gnss_list = []
real_pos = []


# IMU 数据监听
def imu_listener(data):
    if (len(imu_list) < imu_len):
        accel = data.accelerometer
        gyro = data.gyroscope

        imu_list.append(((accel.x, accel.y, accel.z), (gyro.x, gyro.y, gyro.z), data.timestamp))
        print(accel)


# 全球导航卫星系统 数据监听
def gnss_listener(data):
    if (len(gnss_list) < gnss_len):
        x, y, z = a.gnss_to_xyz(data)
        rpos = data.transform.location

        gnss_list.append(((x, y, z), data.timestamp))
        real_pos.append(((rpos.x, rpos.y, rpos.z), data.timestamp))


# 生成传感器
a.spawn_imu(imu_per, imu_std_dev_a, imu_std_dev_g)
a.imu_reg_callback(imu_listener)
a.spawn_gnss(gnss_per, gnss_std_dev_geo)
a.gnss_reg_callback(gnss_listener)

# 设置自动驾驶模式
a.vehicle.set_autopilot(True)

init_vel = a.vehicle.get_velocity()
init_loc = a.vehicle.get_location()
init_rot = a.vehicle.get_transform().rotation
timestamp = a.world.get_snapshot().timestamp.elapsed_seconds

init_state = np.asarray([init_loc.x, init_loc.y, init_rot.yaw * np.pi / 180, init_vel.x, init_vel.y]).reshape(5, 1)
int_obj = integrate.imu_integrate(init_state, timestamp)

int_rvel_list = []
int_rpos_list = []
int_ryaw_list = []


def imu_int_listener(data):
    if (len(int_rvel_list) < imu_len):
        rvel = a.vehicle.get_velocity()
        rloc = a.vehicle.get_location()
        rrot = a.vehicle.get_transform().rotation
        timestamp = data.timestamp

        int_rvel_list.append(((rvel.x, rvel.y), timestamp))
        int_rpos_list.append(((rloc.x, rloc.y), timestamp))
        int_ryaw_list.append((rrot.yaw * np.pi / 180, timestamp))

        yaw_vel = data.gyroscope.z
        accel_x = data.accelerometer.x
        accel_y = data.accelerometer.y

        prev_pos = agent.carla.Location(int_obj.state[0][0], int_obj.state[1][0], rloc.z + 1)

        int_obj.update(np.asarray([accel_x, accel_y, yaw_vel]).reshape(3, 1), timestamp)

        pos = agent.carla.Location(int_obj.state[0][0], int_obj.state[1][0], rloc.z + 1)
        d.draw_line(prev_pos, pos, thickness=0.1, color=agent.carla.Color(10, 10, 255), life_time=3)


a.imu_reg_callback(imu_int_listener)

init_vel = a.vehicle.get_velocity()
init_loc = a.vehicle.get_location()
init_rot = a.vehicle.get_transform().rotation
timestamp = a.world.get_snapshot().timestamp.elapsed_seconds

imu_var_a = 0.05
imu_var_g = 0.01
gnss_var = 30

init_state = np.asarray([init_loc.x, init_loc.y, init_rot.yaw * np.pi / 180, init_vel.x, init_vel.y]).reshape(5, 1)
kal_obj = kalman_filter.kalman_filter(init_state, timestamp, imu_var_a, imu_var_g, gnss_var)

kal_rvel_list = []
kal_rpos_list = []
kal_ryaw_list = []
kal_gnss_list = []
kal_gact_list = []


def imu_kal_listener(data):
    if (len(kal_rvel_list) < imu_len):
        rvel = a.vehicle.get_velocity()
        rloc = a.vehicle.get_location()
        rrot = a.vehicle.get_transform().rotation
        timestamp = data.timestamp

        kal_rvel_list.append(((rvel.x, rvel.y), timestamp))
        kal_rpos_list.append(((rloc.x, rloc.y), timestamp))
        kal_ryaw_list.append((rrot.yaw * np.pi / 180, timestamp))

        yaw_vel = data.gyroscope.z
        accel_x = data.accelerometer.x
        accel_y = data.accelerometer.y

        prev_pos = agent.carla.Location(kal_obj.state[0][0], kal_obj.state[1][0], rloc.z + 1)

        kal_obj.update(np.asarray([accel_x, accel_y, yaw_vel]).reshape(3, 1), timestamp)

        pos = agent.carla.Location(kal_obj.state[0][0], kal_obj.state[1][0], rloc.z + 1)

        d.draw_line(prev_pos, pos, thickness=0.1, color=agent.carla.Color(10, 255, 10), life_time=3)


def gnss_kal_listener(data):
    if (len(kal_rvel_list) < imu_len):
        rloc = a.vehicle.get_location()
        timestamp = data.timestamp

        x, y, z = a.gnss_to_xyz(data)

        kal_gnss_list.append(((x, y), timestamp))
        kal_gact_list.append(((rloc.x, rloc.y), timestamp))

        prev_pos = agent.carla.Location(kal_obj.state[0][0], kal_obj.state[1][0], rloc.z + 1)
        kal_obj.measure(np.asarray([x, y]).reshape(2, 1), timestamp)
        pos = agent.carla.Location(kal_obj.state[0][0], kal_obj.state[1][0], rloc.z + 1)

        d.draw_line(prev_pos, pos, thickness=0.1, color=agent.carla.Color(10, 255, 10), life_time=2)
        d.draw_point(agent.carla.Location(x, y, z + 2), size=0.05, color=agent.carla.Color(255, 10, 10), life_time=3)


a.imu_reg_callback(imu_kal_listener)
a.gnss_reg_callback(gnss_kal_listener)

time.sleep(1)


## 显示输出
print("%d of %d"%(len(gnss_list), gnss_len))

rpos_xy = np.asarray([(x[0][0], x[0][1]) for x in kal_rpos_list])
gnss_xy = np.asarray([(x[0][0], x[0][1]) for x in kal_gnss_list])

# TypeError: int() argument must be a string, a bytes-like object or a number, not 'KeyboardModifier'
plt.scatter(-gnss_xy[:, 0], gnss_xy[:, 1], 0.3, label="GNSS data", color='red')
# plt.plot(-gnss_xy[:,0], gnss_xy[:,1], label="GNSS data")
plt.plot(-rpos_xy[:,0], rpos_xy[:,1], label="Real position")
plt.legend()
plt.show()


plt.plot([max(x[0][0],-10) for x in imu_list])
plt.title('x axis imu acceleration')
plt.show()
plt.plot([max(x[0][1], -10) for x in imu_list])
plt.title('y axis imu acceleration')
plt.show()
plt.plot([max(x[0][2], -10) for x in imu_list])
plt.title('z axis imu acceleration')
plt.show()


print("Velocity with IMU integration. Samples received : %d of %d"%(len(int_rvel_list), imu_len))

plt.plot([x[0][0] for x in int_rvel_list], label='actual')
plt.plot([x[0][3,0] for x in int_obj.states], label='predicted')
plt.title('x axis velocity')
plt.legend()
plt.show()

plt.plot([x[0][1] for x in int_rvel_list], label='actual')

plt.plot([x[0][4,0] for x in int_obj.states], label='predicted')
plt.title('y axis velocity')
plt.legend()
plt.show()

plt.plot([x[0] for x in int_ryaw_list], label='actual')
plt.plot([x[0][2,0] for x in int_obj.states], label='predicted')
plt.title('yaw')
plt.legend()
plt.show()


print("Position estimate with IMU integration. Samples received : %d of %d"%(len(int_rvel_list), imu_len))

rpos_xy = np.asarray([(x[0][0], x[0][1]) for x in int_rpos_list])
gnss_xy = np.asarray([(x[0][0], x[0][1]) for x in int_obj.states])

plt.plot(-gnss_xy[:,0], gnss_xy[:,1], label="IMU prediction")
plt.plot(-rpos_xy[:,0], rpos_xy[:,1], label="Real position")
plt.legend()
plt.show()

print("Time : %.2f to %.2f"%(int_rpos_list[0][1], int_rpos_list[-1][1]))
print("Time : %.2f to %.2f"%(int_obj.states[0][1], int_obj.states[-1][1]))


print("Velocity with Kalman filter. Samples received : %d of %d"%(len(int_rvel_list), imu_len))

plt.plot([x[0][0] for x in kal_rvel_list], label='actual')
plt.plot([x[0][3,0] for x in kal_obj.states if not x[2]], label='predicted')
plt.title('x axis velocity')
plt.legend()
plt.show()

plt.plot([x[0][1] for x in kal_rvel_list], label='actual')
plt.plot([x[0][4,0] for x in kal_obj.states if not x[2]], label='predicted')
plt.title('y axis velocity')
plt.legend()
plt.show()

plt.plot([x[0] for x in kal_ryaw_list], label='actual')
plt.plot([x[0][2,0] for x in kal_obj.states if not x[2]], label='predicted')
plt.title('yaw')
plt.legend()
plt.show()


print("Position estimate with kalman filter. Samples received : %d of %d"%(len(kal_rvel_list), imu_len))

k_rpos_xy = np.asarray([(x[0][0], x[0][1]) for x in kal_rpos_list])
k_pred_xy = np.asarray([(x[0][0], x[0][1]) for x in kal_obj.states if not x[2]])
k_gnss_xy = np.asarray([(x[0][0], x[0][1]) for x in kal_gnss_list])
k_gact_xy = np.asarray([(x[0][0], x[0][1]) for x in kal_gact_list])

plt.plot(-k_pred_xy[:,0], k_pred_xy[:,1,0], label="Kalman filter prediction", color='orange')
plt.plot(-k_rpos_xy[:,0], k_rpos_xy[:,1], label="Real position", color='green')
plt.scatter(-k_gnss_xy[:,0], k_gnss_xy[:,1], 0.3, label="GNSS data", color='red')
plt.legend()
plt.show()

# 保存IMU数据到CSV
import csv

# 1. 保存IMU数据
with open('imu_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'timestamp'])
    for data in imu_list:
        accel = data[0]
        gyro = data[1]
        ts = data[2]
        writer.writerow([accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], ts])

# 2. 保存GNSS数据
with open('gnss_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'z', 'timestamp'])
    for data in gnss_list:
        pos = data[0]
        ts = data[1]
        writer.writerow([pos[0], pos[1], pos[2], ts])

print("数据已保存到 imu_data.csv 和 gnss_data.csv")



print("Time : %.2f to %.2f"%(kal_rpos_list[0][1], kal_rpos_list[-1][1]))
print("Time : %.2f to %.2f"%(kal_obj.states[0][1], kal_obj.states[-1][1]))


