%% 使用合成数据的视觉惯性里程计
% 此示例演示如何使用惯性测量单元 (IMU) 和单目摄像头估计地面车辆的姿态（位置和方向）。
% 在本示例中，您将：
%
% # 创建包含车辆真实轨迹的驾驶场景。
% # 使用 IMU 和视觉里程计模型来生成测量值。
% # 融合这些测量值来估计车辆的姿态，然后显示结果。
%
% 视觉-惯性里程计(Visual-inertial odometry, VIO) 通过融合单目摄像头的视觉里程计姿态估计和 IMU 的姿态估计来估计姿态。
% IMU 在较短的时间间隔内能够返回准确的姿态估计，但由于集成了惯性传感器测量值，因此存在较大的漂移。
% 单目摄像头在较长的时间间隔内能够返回准确的姿态估计，但存在尺度不确定问题。
% 单目SLAM估计的轨迹和地图与真实的轨迹和地图相差一个因子，即尺度（Scale）。由于单目SLAM无法仅凭图像确定这个真实尺度，称为尺度不确定性（Scale Ambiguity）
% 鉴于这些互补的优缺点，使用视觉-惯性里程计融合这些传感器是一个合适的选择。此方法可用于无法获取 GPS 读数的场景，例如城市峡谷。

%% 创建带有轨迹的驾驶场景
% 创建一个包含以下内容的（自动驾驶工具箱）对象：drivingScenario
%
% * 车辆行驶的道路
% * 道路两旁的建筑物
% * 车辆的真实姿态
% * 车辆的估计姿态
%
% 车辆的真实姿态显示为实心蓝色长方体。估计姿态显示为透明蓝色长方体。
% 请注意，由于真实姿态和估计姿态存在重叠，因此估计姿态未出现在初始可视化中。
%
% 使用 System object 生成地面车辆的基线轨迹 waypointTrajectory。
% 注意，由于需要车辆的加速度，因此使用 waypointTrajectory 代替 drivingScenario/trajectory。
% 该轨迹使用一组航路点、到达时间和速度，以指定的采样率生成。

% 使用地面真实情况和估计的车辆姿势创建驾驶场景。
scene = drivingScenario;
groundTruthVehicle = vehicle(scene, 'PlotColor', [0 0.4470 0.7410]);
estVehicle = vehicle(scene, 'PlotColor', [0 0.4470 0.7410]);

% 生成基线轨迹。
sampleRate = 100;
wayPoints = [  0   0 0;
             200   0 0;
             200  50 0;
             200 230 0;
             215 245 0;
             260 245 0;
             290 240 0;
             310 258 0;
             290 275 0;
             260 260 0;
             -20 260 0];
t = [0 20 25 44 46 50 54 56 59 63 90].';
speed = 10;
velocities = [ speed     0 0;
               speed     0 0;
                   0 speed 0;
                   0 speed 0;
               speed     0 0;
               speed     0 0;
               speed     0 0;
                   0 speed 0;
              -speed     0 0;
              -speed     0 0;
              -speed     0 0];    

traj = waypointTrajectory(wayPoints, 'TimeOfArrival', t, ...
    'Velocities', velocities, 'SampleRate', sampleRate);

% 添加道路和建筑物到场景并进行可视化。
helperPopulateScene(scene, groundTruthVehicle);


%% 创建融合滤波器
% 创建滤波器以融合 IMU 和视觉里程计测量值。
% 本示例使用松耦合方法融合测量值。
% 虽然结果不如紧耦合方法精确，但所需的处理量明显较少，且结果令人满意。
% 融合滤波器使用误差状态卡尔曼滤波器来跟踪朝向（以四元数表示）、位置、速度和传感器偏差。
%
% 该 insfilterErrorState 对象具有以下函数来处理传感器数据：predict 和 fusemvo。
%
% 该 predict 函数将来自 IMU 的加速度计和陀螺仪测量值作为输入。
% 每次调用该 predict 函数对加速度计和陀螺仪进行采样时。
% 该函数根据加速度计和陀螺仪测量值，以一个时间步长预测状态，并更新滤波器的误差状态协方差。
%
% 该 fusemvo 函数以视觉里程计位姿估计值作为输入。
% 该函数通过计算卡尔曼增益来更新基于视觉里程计位姿估计值的误差状态，
% 卡尔曼增益会根据各种输入的不确定性对其进行加权。
% 与 predict 函数一样，该函数也会更新误差状态的协方差，这次会将卡尔曼增益考虑在内。
% 然后，使用新的误差状态更新状态，并重置误差状态。

filt = insfilterErrorState('IMUSampleRate', sampleRate, ...
    'ReferenceFrame', 'ENU');
% 设置初始状态和错误状态协方差。
helperInitialize(filt, traj);


%% 指定视觉里程计模型
% 定义视觉里程计模型参数。
% 这些参数使用单目相机建模基于特征匹配和跟踪的视觉里程计系统。
% 该 scale 参数考虑了单目相机后续视觉帧的未知尺度。
% 其他参数将视觉里程计读数的漂移建模为白噪声和一阶高斯-马尔可夫过程的组合。

% 标志 useVO 决定是否使用视觉里程计
% useVO = false; % 仅使用 IMU（轨迹会飘很远）
useVO = true; % 同时使用 IMU 和视觉里程计（仅使用视觉里程计的轨迹形状相似，但尺度变大）。

paramsVO.scale = 2;
paramsVO.sigmaN = 0.139;
paramsVO.tau = 232;
paramsVO.sigmaB = sqrt(1.34);
paramsVO.driftBias = [0 0 0];


%% 指定 IMU 传感器
% 使用 System 对象定义包含加速度计和陀螺仪的 IMU 传感器模型 imuSensor。
% 该传感器模型包含用于建模确定性和随机性噪声源的属性。
% 此处设置的属性值是低成本 MEMS 传感器的典型值。

% 将 RNG 种子设置为默认值，以便在后续运行中获得相同的结果。
rng('default')

imu = imuSensor('SampleRate', sampleRate, 'ReferenceFrame', 'ENU');

% 加速度计
imu.Accelerometer.MeasurementRange =  19.6; % m/s^2
imu.Accelerometer.Resolution = 0.0024; % m/s^2/LSB
imu.Accelerometer.NoiseDensity = 0.01; % (m/s^2)/sqrt(Hz)

% 陀螺仪
imu.Gyroscope.MeasurementRange = deg2rad(250); % rad/s
imu.Gyroscope.Resolution = deg2rad(0.0625); % rad/s/LSB
imu.Gyroscope.NoiseDensity = deg2rad(0.0573); % (rad/s)/sqrt(Hz)
imu.Gyroscope.ConstantBias = deg2rad(2); % rad/s


%% 设置模拟
% 指定运行模拟的时间量并初始化模拟循环期间记录的变量。

% 运行模拟 60 秒。
numSecondsToSimulate = 60;
numIMUSamples = numSecondsToSimulate * sampleRate;

% 定义视觉里程计采样率。
imuSamplesPerCamera = 4;
numCameraSamples = ceil(numIMUSamples / imuSamplesPerCamera);

% 预先分配数据数组以绘制结果。
[pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst] ...
    = helperPreallocateData(numIMUSamples, numCameraSamples);

% 为视觉里程计融合设置测量噪声参数
RposVO = 0.1;
RorientVO = 0.1;


%% 运行模拟循环
% 以 IMU 采样率运行模拟。每个 IMU 样本用于预测滤波器状态向前一个时间步长。
% 一旦获得新的视觉里程计读数，便会用它来校正当前的滤波器状态。
% 
% 滤波器估计值存在一些漂移，
% 可以通过附加传感器（例如 GPS）或附加约束（例如道路边界图）进一步校正。

cameraIdx = 1;
for i = 1:numIMUSamples
    % 生成地面真实轨迹值。
    [pos(i,:), orient(i,:), vel(i,:), acc(i,:), angvel(i,:)] = traj();
    
    % 根据地面真实轨迹值生成加速度计和陀螺仪测量值。
    [accelMeas, gyroMeas] = imu(acc(i,:), angvel(i,:), orient(i));
    
    % 根据加速度计和陀螺仪测量值预测滤波器向前一步的状态
    predict(filt, accelMeas, gyroMeas);
    
    if (1 == mod(i, imuSamplesPerCamera)) && useVO
        % 根据地面真实值和视觉里程计模型生成视觉里程计姿态估计。
        [posVO(cameraIdx,:), orientVO(cameraIdx,:), paramsVO] = ...
            helperVisualOdometryModel(pos(i,:), orient(i,:), paramsVO);
        
        % 根据视觉里程计数据纠正过滤器状态
        fusemvo(filt, posVO(cameraIdx,:), RposVO, ...
            orientVO(cameraIdx), RorientVO);
        
        cameraIdx = cameraIdx + 1;
    end
    
    [posEst(i,:), orientEst(i,:), velEst(i,:)] = pose(filt);

    % 更新估计的车辆姿态
    helperUpdatePose(estVehicle, posEst(i,:), velEst(i,:), orientEst(i));
    
    % 更新地面真实车辆姿态
    helperUpdatePose(groundTruthVehicle, pos(i,:), vel(i,:), orient(i));
    
    % 更新驾驶场景可视化
    updatePlots(scene);
    drawnow limitrate;
end


%% 绘制结果
% 绘制地面真实车辆轨迹、视觉里程计估计值和融合滤波器估计值。

figure
if useVO
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posVO(:,1), posVO(:,2), posVO(:,3), ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'Visual Odometry (VO)', ...
        'Visual-Inertial Odometry (VIO)', 'Location', 'northeast')
else
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'IMU Pose Estimate')
end
view(-90, 90)
title('Vehicle Position')
xlabel('X (m)')
ylabel('Y (m)')
grid on


%%
% 该图显示，视觉里程计估计在估计轨迹形状方面相对准确。
% IMU 和视觉里程计测量值的融合消除了视觉里程计测量值中的比例因子不确定性以及 IMU 测量值的漂移。


%% 支持函数

%%
% *|helperVisualOdometryModel|*
%
% 根据地面真实输入和参数结构计算视觉里程测量值。
% 为了模拟单目相机后续帧之间缩放的不确定性，
% 将一个恒定缩放因子与随机漂移相结合应用于地面真实位置。
function [posVO, orientVO, paramsVO] ...
    = helperVisualOdometryModel(pos, orient, paramsVO)

% 提取模型参数。
scaleVO = paramsVO.scale;
sigmaN = paramsVO.sigmaN;
tau = paramsVO.tau;
sigmaB = paramsVO.sigmaB;
sigmaA = sqrt((2/tau) + 1/(tau*tau))*sigmaB;
b = paramsVO.driftBias;

% 计算漂移。
b = (1 - 1/tau).*b + randn(1,3)*sigmaA;
drift = randn(1,3)*sigmaN + b;
paramsVO.driftBias = b;

% 计算视觉里程测量值。
posVO = scaleVO*pos + drift;
orientVO = orient;
end

%%
% *|helperInitialize|*
%
% 设置融合滤波器的初始状态和协方差值
function helperInitialize(filt, traj)

% 从轨迹对象中检索初始位置、方向和速度并重置内部状态
[pos, orient, vel] = traj();
reset(traj);

% 设置初始状态值。
filt.State(1:4) = compact(orient(1)).';
filt.State(5:7) = pos(1,:).';
filt.State(8:10) = vel(1,:).';

% 将陀螺仪方差和视觉里程计尺度因子协方差设置为对应于低置信度的大值。
filt.StateCovariance(10:12, 10:12) = 1e6;
filt.StateCovariance(end) = 2e2;
end

%%
% *|helperPreallocateData|*
%
% 预先分配数据以记录模拟结果。
function [pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst] ...
    = helperPreallocateData(numIMUSamples, numCameraSamples)

% 指定真值
pos = zeros(numIMUSamples, 3);
orient = quaternion.zeros(numIMUSamples, 1);
vel = zeros(numIMUSamples, 3);
acc = zeros(numIMUSamples, 3);
angvel = zeros(numIMUSamples, 3);

% 视觉里程计输出。
posVO = zeros(numCameraSamples, 3);
orientVO = quaternion.zeros(numCameraSamples, 1);

% 滤波器输出
posEst = zeros(numIMUSamples, 3);
orientEst = quaternion.zeros(numIMUSamples, 1);
velEst = zeros(numIMUSamples, 3);
end


%%
% *|helperUpdatePose|*
%
% 更新车辆的姿态。
function helperUpdatePose(veh, pos, vel, orient)

veh.Position = pos;
veh.Velocity = vel;
rpy = eulerd(orient, 'ZYX', 'frame');
veh.Yaw = rpy(1);
veh.Pitch = rpy(2);
veh.Roll = rpy(3);
end

%% 参考
%
% * https://ww2.mathworks.cn/help/driving/ug/visual-inertial-odometry-using-synthetic-data.html
% * Sola, J. "Quaternion Kinematics for the Error-State Kalman Filter." ArXiv e-prints, arXiv:1711.02508v1 [cs.RO] 3 Nov 2017.
% * R. Jiang, R., R. Klette, and S. Wang. "Modeling of Unbounded Long-Range Drift in Visual Odometry." 2010 Fourth Pacific-Rim Symposium on Image and Video Technology. Nov. 2010, pp. 121-126.
