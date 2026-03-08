% -------------------------------------------------------------------------
% KF-GINS-Matlab: Plot raw IMU increments as angular rate and acceleration
%
% 说明:
%   1) 读取 IMU 原始增量数据（时间, 三轴陀螺增量, 三轴加表增量）
%   2) 通过相邻历元时间差 dt，将增量转换为角速度(deg/s)和加速度(m/s^2)
%   3) 在两个 figure 中分别绘制三轴陀螺角速度和三轴加速度时间序列
%
% 数据格式假定:
%   每行 7 列: [t, dtheta_x, dtheta_y, dtheta_z, dv_x, dv_y, dv_z]
%   t 为 GPS 周内秒
%
% 文件类型:
%   - 文本:  使用 importdata 读取
%   - 二进制(.bin, double): 使用 ReadBinaryIMU 读取
% -------------------------------------------------------------------------

clear; clc;

% 根据需要修改 IMU 文件路径
imufile = 'dataset5/IMU.bin';

% 读取 IMU 数据
[~, ~, ext] = fileparts(imufile);
if strcmpi(ext, '.bin')
    % 二进制文件，依赖于工程根目录下的 ReadBinaryIMU.m
    imudata = ReadBinaryIMU(imufile);
else
    % 文本文件
    imudata = importdata(imufile);
end

if size(imudata, 2) < 7
    error('IMU 数据列数不足，期望至少 7 列: [t, dtheta(3), dv(3)]');
end

% 拆分数据
t      = imudata(:, 1);      % 时间 [s]
dtheta = imudata(:, 2:4);    % 三轴陀螺增量 [rad]
dv     = imudata(:, 5:7);    % 三轴加表增量 [m/s]

% 计算每个历元的 dt
dt = diff(t);
if any(dt <= 0)
    warning('IMU 时间戳存在非递增情况，请检查数据质量。');
end

% 为了和样本数对齐，将最后一个 dt 取为前一个 dt
dt_full = [dt; dt(end)];

% 将增量转换为角速度(deg/s)和加速度(m/s^2)
omega = dtheta ./ dt_full * 180/pi;   % 陀螺角速度 [deg/s]
acc   = dv     ./ dt_full;            % 线加速度 [m/s^2]

% 绘制陀螺角速度
figure();
plot(t, omega);
title('IMU Gyro Angular Rate');
xlabel('Time [s]');
ylabel('Angular Rate [deg/s]');
legend('Gyro X', 'Gyro Y', 'Gyro Z');
grid on;

% 绘制加速度
figure();
plot(t, acc);
title('IMU Accelerometer');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
legend('Acc X', 'Acc Y', 'Acc Z');
grid on;


