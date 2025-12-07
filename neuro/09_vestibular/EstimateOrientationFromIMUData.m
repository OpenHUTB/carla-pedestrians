%% 从IMU数据中估计朝向
% 加载 rpy_9axis.mat 文件，其中包含设备记录的加速度计、陀螺仪和磁力计传感器数据，
% 这些传感器数据分别沿俯仰（绕y轴）、偏航（绕z轴）和横滚（绕x轴）方向振动。
% 该文件还包含记录的采样率。

load 'rpy_9axis.mat' sensorData Fs

accelerometerReadings = sensorData.Acceleration;
gyroscopeReadings = sensorData.AngularVelocity;
%% 
% 创建一个 imufilterSystem 对象，将采样率设置为传感器数据的采样率。
% 指定抽取因子为 2，以降低算法的计算成本。

decim = 2;
fuse = imufilter('SampleRate',Fs,'DecimationFactor',decim);
%% 
% 将加速度计读数和陀螺仪读数传递给 imufilter 对象，
% fuse 函数输出传感器主体方向随时间变化的估计值。
% 默认情况下，方向以四元数向量的形式输出。

q = fuse(accelerometerReadings,gyroscopeReadings);
%% 
% 朝向由从父坐标系旋转到子坐标系所需的角位移定义。
% 绘制欧拉角方向随时间的变化图，单位为度。
% 
% imufilterfusion 可以正确估算相对于假定的朝北初始方向的方位变化。
% 然而，记录时设备的x轴指向南。
% 要正确估算相对于真实初始方向或相对于 NED（North East Down）坐标系 的方位，请使用 ahrsfilter 。

time = (0:decim:size(accelerometerReadings,1)-1)/Fs;

plot(time,eulerd(q,'ZYX','frame'))
title('Orientation Estimate')
legend('Z-axis', 'Y-axis', 'X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')
%% 
% 参考：https://ww2.mathworks.cn/help/fusion/ref/imufilter-system-object.html



