% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2022.11.30
%
%    MOD  : 在原有速度、姿态、位置绘图基础上，增加平面位置段落的细化显示，
%           先用灰色绘制全轨迹，再在四个GNSS中断时间段内用不同颜色高亮显示
%           （Weizhen Wang, 2026）
% -------------------------------------------------------------------------

% importdata navresult
navpath = "dataset5/ins_gnss/NavResult_GNSSVEL.nav";
navdata = importdata(navpath);

% velocity
figure()
plot(navdata(:, 2), navdata(:, 6:8));
title('Velocity');
legend('North', 'East', 'Down');
xlabel('Time[s]');
ylabel('Vel[m/s]');
grid("on");

% attitude
figure()
plot(navdata(:, 2), navdata(:, 9:11));
title('Attitude');
legend('Roll', 'Pitch', 'Yaw');
xlabel('Time[s]');
ylabel('Att[deg]');
grid("on");

% position
param = Param();
blh = navdata(:, 3:5);
blh(:, 1) = blh(:, 1) * param.D2R;
blh(:, 2) = blh(:, 2) * param.D2R;
first_blh = blh(1, 1:3);

[rm, rn] = getRmRn(first_blh(1), param);
h = first_blh(2);
DR = diag([rm + h, (rn + h)*cos(first_blh(1)), -1]);

% blh to ned
pos = zeros(size(blh));
for i = 1:size(pos, 1)
    delta_blh = blh(i, :) - first_blh;
    delta_pos = DR * delta_blh';
    pos(i, :) = delta_pos';
end

%% plane position（平面位置），启用数据光标显示对应时间
figure()
hold on;

time_vec = navdata(:, 2);

% 第一步：用灰色绘制全段轨迹
h_other = plot(pos(:, 2), pos(:, 1), 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.0);
set(h_other, 'UserData', time_vec);

% 第二步：在灰色基础上用不同颜色覆盖绘制四个GNSS中断时间段
% 定义四个GNSS中断时间段
gnss_block_intervals = [
    528250, 528310;
    528480, 528540;
    528660, 528720;
    528840, 528900
];
% 使用蓝色高亮GNSS中断时间段
block_color = [0.2, 0.6, 1.0];  % 浅蓝色

h_gnss_block = [];  % GNSS中断时间段句柄

for i = 1:size(gnss_block_intervals, 1)
    seg_start = gnss_block_intervals(i, 1);
    seg_end = gnss_block_intervals(i, 2);
    
    % 找到该时间段内的数据点
    mask = (time_vec >= seg_start) & (time_vec <= seg_end);
    
    if any(mask)
        % 用更粗的线条覆盖绘制，确保可见
        h_seg = plot(pos(mask, 2), pos(mask, 1), 'Color', block_color, 'LineWidth', 2.0);
        
        % 将时间信息存入曲线对象
        set(h_seg, 'UserData', time_vec(mask));
        
        % 存储句柄
        h_gnss_block = [h_gnss_block; h_seg];
    end
end

title('Position');
xlabel('East[m]');
ylabel('North[m]');
grid("on");

% 添加图例（使用实际绘制的线条句柄）
legend_handles = [];
legend_labels = {};
if ~isempty(h_gnss_block)
    legend_handles = [legend_handles; h_gnss_block(1)];  % 只取第一个作为代表
    legend_labels{end+1} = 'GNSS Block Interval';
end
if ~isempty(h_other)
    legend_handles = [legend_handles; h_other];
    legend_labels{end+1} = 'Other';
end

if ~isempty(legend_handles)
    legend(legend_handles, legend_labels, 'Location', 'best');
end

% 启用数据光标模式，并设置自定义显示函数：X/Y/Time
dcm = datacursormode(gcf);
set(dcm, 'Enable', 'on', 'UpdateFcn', @localUpdateDataCursor);


%% height
figure()
plot(navdata(:, 2), navdata(:, 5));
title('Height');
xlabel('Time[s]');
ylabel('Height[m]');
grid("on");

%% 数据光标回调函数：显示 East/North/Time
function txt = localUpdateDataCursor(~, event_obj)
    pos = get(event_obj, 'Position');          % [East, North]
    idx = get(event_obj, 'DataIndex');         % 点在数据中的索引
    h   = get(event_obj, 'Target');            % 曲线句柄
    t   = get(h, 'UserData');                  % 对应时间序列
    time = t(idx);
    
    txt = {sprintf('East: %.3f m',  pos(1)), ...
           sprintf('North: %.3f m', pos(2)), ...
           sprintf('Time: %.3f s',  time)};
end
