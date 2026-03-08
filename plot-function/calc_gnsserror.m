% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Weizhen Wang
% Contact :
%    Date : 2026.01.17
% -------------------------------------------------------------------------

% 计算 GNSS 数据（ReadGNSSData 格式）与参考结果（nav 文件）的
% 位置和速度误差，用于评估 GNSS 解算精度。
%
% ReadGNSSData 输出格式（13 列）：
%   1: 时间 t [s]
%   2-4: 位置（纬度、经度、高度，单位：deg, deg, m）
%   5-7: 位置标准差（sdn, sde, sdu），单位：m
%   8-10: 速度（vn, ve, vd），单位：m/s（NED 系）
%   11-13: 速度标准差

%% 文件路径设置（根据需要自行修改）MOV_PPP.pos
gnssfilepath = 'dataset5/GNSSsolution.pos';  % GNSS 解算结果（ReadGNSSData 支持的格式）
% gnssfilepath = 'dataset5/MOV_SPP.pos';  % GNSS 解算结果（ReadGNSSData 支持的格式）
% gnssfilepath = 'dataset5/MOV_PPP.pos';  % GNSS 解算结果（ReadGNSSData 支持的格式）
truthpath    = 'dataset5/REF_NAV.nav';       % 参考结果 nav 文件

%% 读取 GNSS 数据（只支持 ReadGNSSData）
gnssdata = ReadGNSSData(gnssfilepath);

param = Param();

% 将 GNSS 的纬度、经度由度转换为弧度，便于与参考结果统一
gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;

%% 读取参考结果（nav 文件）
temp = importdata(truthpath);
ref  = temp(:, 2:end);  % nav 文件第 1 列为 0，不用
% 参考数据格式：10 列
%   1: 时间 t [s]
%   2-4: 位置（纬度、经度、高度，单位：deg, deg, m）
%   5-7: 速度（vn, ve, vd），单位：m/s（NED 系）
%   8-10: 姿态（roll, pitch, yaw），单位：deg
% 将参考结果中的纬度、经度由度转换为弧度
ref(:, 2:3) = ref(:, 2:3) * param.D2R;

%% 找到 GNSS 与参考结果的时间重叠区间
gnss_start = gnssdata(1, 1);
gnss_end   = gnssdata(end, 1);
ref_start  = ref(1, 1);
ref_end    = ref(end, 1);

% 起始历元固定为 527000（只绘制从 527000 s 开始的误差）
starttime = 527000;

% 结束时间仍取 GNSS 与参考结果时间交集的末端
if gnss_end > ref_end
    endtime = ref_end;
else
    endtime = gnss_end;
end

%% 构造统一时间轴（以 GNSS 采样间隔为基准）
dt = mean(diff(gnssdata(:, 1)));
time = (starttime:dt:endtime)';

%% 插值到统一时间轴
% GNSS 数据插值（13 列）
gnss_interp = zeros(size(time, 1), 13);
gnss_interp(:, 1)    = time;
gnss_interp(:, 2:13) = interp1(gnssdata(:, 1), gnssdata(:, 2:13), time);

% 参考结果插值（10 列）
ref_interp = zeros(size(time, 1), 10);
ref_interp(:, 1)    = time;
ref_interp(:, 2:10) = interp1(ref(:, 1), ref(:, 2:10), time);

%% 计算位置误差（NED）
% 1) 在经纬高空间做差（纬度、经度为弧度，高度为米）
error_pos_blh = gnss_interp(:, 2:4) - ref_interp(:, 2:4);  % [dLat(rad), dLon(rad), dH(m)]

% 2) 使用参考轨迹第一个点构造 DR 矩阵，将经纬高误差转换为 NED
first_blh = ref_interp(1, 2:4);              % [lat(rad), lon(rad), h(m)]
[rm, rn]  = getRmRn(first_blh(1), param);    % 子午圈/卯酉圈曲率半径
h         = first_blh(3);
DR        = diag([rm + h, (rn + h) * cos(first_blh(1)), -1]);

error_pos_ned = zeros(size(error_pos_blh));  % [dN, dE, dD]
for i = 1:size(error_pos_blh, 1)
    delta_pos = DR * error_pos_blh(i, :)';
    error_pos_ned(i, :) = delta_pos';
end

%% 计算速度误差（NED）
% GNSS 速度（列 8-10）：[vn, ve, vd]
gnss_vel = gnss_interp(:, 8:10);
% 参考速度（列 5-7）：[vn, ve, vd]
ref_vel  = ref_interp(:, 5:7);
error_vel = gnss_vel - ref_vel;

%% 组合误差数据
% error 列定义：
%   1: 时间 [s]
%   2-4: 位置误差 [N, E, D]，单位 [m]
%   5-7: 速度误差 [vn, ve, vd]，单位 [m/s]
error = zeros(size(time, 1), 7);
error(:, 1)   = time;
error(:, 2:4) = error_pos_ned;
error(:, 5:7) = error_vel;

%% 输出误差 RMS 统计
disp("=== GNSS 位置/速度误差统计（ReadGNSSData 格式） ===");

pos_rms_ned = sqrt(mean(error_pos_ned .^ 2));
disp("位置误差 RMS (N, E, D) [m]: " + num2str(pos_rms_ned));
disp("位置总 RMS [m]: " + num2str(sqrt(mean(sum(error_pos_ned .^ 2, 2)))));

vel_rms_ned = sqrt(mean(error_vel .^ 2));
disp("速度误差 RMS (vn, ve, vd) [m/s]: " + num2str(vel_rms_ned));
disp("速度总 RMS [m/s]: " + num2str(sqrt(mean(sum(error_vel .^ 2, 2)))));

%% 计算并输出标准差统计
disp("=== GNSS 位置/速度标准差统计 ===");

% 位置标准差（列 5-7）：[sdn, sde, sdu]
pos_std_mean = mean(gnss_interp(:, 5:7), 1);
disp("位置标准差均值 (SD_N, SD_E, SD_D) [m]: " + num2str(pos_std_mean));
disp("位置标准差总均值 [m]: " + num2str(mean(pos_std_mean)));

% 速度标准差（列 11-13）
vel_std_mean = mean(gnss_interp(:, 11:13), 1);
disp("速度标准差均值 (SD_V_N, SD_V_E, SD_V_D) [m/s]: " + num2str(vel_std_mean));
disp("速度标准差总均值 [m/s]: " + num2str(mean(vel_std_mean)));

%% 绘制位置误差曲线
figure;
plot(error(:, 1), error(:, 2:4));
title('GNSS 位置误差 (NED)');
xlabel('时间 [s]');
ylabel('位置误差 [m]');
legend('North', 'East', 'Down');
grid("on");

%% 绘制速度误差曲线
figure;
plot(error(:, 1), error(:, 5:7));
title('GNSS 速度误差 (NED)');
xlabel('时间 [s]');
ylabel('速度误差 [m/s]');
legend('V_N', 'V_E', 'V_D');
grid("on");

%% 绘制位置标准差时间序列
figure;
plot(gnss_interp(:, 1), gnss_interp(:, 5:7));
title('GNSS 位置标准差 (NED)');
xlabel('时间 [s]');
ylabel('位置标准差 [m]');
legend('SD_N', 'SD_E', 'SD_D');
grid("on");

%% 绘制速度标准差时间序列
figure;
plot(gnss_interp(:, 1), gnss_interp(:, 11:13));
title('GNSS 速度标准差 (NED)');
xlabel('时间 [s]');
ylabel('速度标准差 [m/s]');
legend('SD_V_N', 'SD_V_E', 'SD_V_D');
grid("on");


