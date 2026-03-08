% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.2
%
%    MOD  : 在原有主程序基础上增加多种组合模式选择（松组合、松+NHC、
%           松+ODO、松+ODO+NHC、松+ZVT），并在零速区间内调用 ZVTUpdate.m，
%           支持 ODO 比例因子扩展状态及多种参数敏感性实验（Weizhen Wang, 2026）
% -------------------------------------------------------------------------

clear;
clc;
% add function to workspace
addpath("function\");

%% define parameters and importdata process config
param = Param();
% cfg = ProcessConfig1();
% cfg = ProcessConfig2();
% cfg = ProcessConfig3();

% 运行模式选择：1-常规松组合；2-松组合+NHC；3-松组合+ODO；4-松组合+ODO+NHC；5-松组合+ODO+NHC+ZVT
fprintf('\n========================================\n');
fprintf('请选择组合模式：\n');
fprintf('  1 - 常规GNSS/INS松组合\n');
fprintf('  2 - GNSS/INS松组合 + NHC\n');
fprintf('  3 - GNSS/INS松组合 + ODO\n');
fprintf('  4 - GNSS/INS松组合 + ODO + NHC\n');
fprintf('  5 - GNSS/INS松组合 + ZVT（零速修正+零航向角速率）\n');
fprintf('========================================\n');
mode_choice = input('请输入选择（1 / 2 / 3 / 4 / 5）：');
if mode_choice == 2
    cfg = ProcessConfig5NHC();
    fprintf('已选择：GNSS/INS松组合 + NHC\n');
elseif mode_choice == 3
    cfg = ProcessConfig5ODO();
    fprintf('已选择：GNSS/INS松组合 + ODO\n');
elseif mode_choice == 4
    cfg = ProcessConfig5ODONHC();
    fprintf('已选择：GNSS/INS松组合 + ODO + NHC\n');
elseif mode_choice == 5
    cfg = ProcessConfig5ZVT();
    fprintf('已选择：GNSS/INS松组合 + ZVT\n');
else
cfg = ProcessConfig5();
    fprintf('已选择：常规GNSS/INS松组合\n');
end

% 运行时间段选择
fprintf('\n========================================\n');
fprintf('请选择运行时间段：\n');
fprintf('  1 - 运行所有数据\n');
fprintf('  2 - 仅运行10分钟数据（用于快速调试）\n');
fprintf('  3 - 运行所有数据（在指定时间段中断GNSS更新）\n');
fprintf('========================================\n');
choice = input('请输入选择（1 / 2 / 3）：');

% GNSS 中断配置（用于展示 NHC 等对松组合的约束效果）
block_gnss = false;
% 定义四个GNSS中断时间段
block_intervals = [
    528250, 528310;
    528480, 528540;
    528660, 528720;
    528840, 528900
];

if choice == 2
    % 仅运行10分钟数据（600秒）
    original_endtime = cfg.endtime;
    cfg.endtime = cfg.starttime + 600;
    fprintf('已选择：仅运行10分钟数据（从 %.2f 到 %.2f 秒）\n', cfg.starttime, cfg.endtime);
    fprintf('原始结束时间 %.2f 已保存，可在需要时恢复\n', original_endtime);
elseif choice == 3
    % 运行全部数据，但在指定时间段中断 GNSS 更新
    block_gnss = true;
    fprintf('已选择：运行所有数据（在以下时间段内中断GNSS更新）：\n');
    for i = 1:size(block_intervals, 1)
        fprintf('  时间段 %d: %.0f - %.0f 秒\n', i, block_intervals(i, 1), block_intervals(i, 2));
    end
else
    fprintf('已选择：运行所有数据\n');
end

%% importdata data
% 二进制文件，使用专用函数读取
imudata = ReadBinaryIMU(cfg.imufilepath);
% 文本文件，使用importdata读取
% imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% gnss data
% 仅在使用GNSS更新时才读取GNSS数据
if cfg.usegnss
    % 使用新格式读取函数（支持"%"开头的注释行），适用于使用RTKLIB解算得到的GNSS结果文件的数据格式
    gnssdata = ReadGNSSData(cfg.gnssfilepath);
    % 使用旧格式读取，适用于原数据格式
    % gnssdata = importdata(cfg.gnssfilepath);
    
    % 将纬度和经度从度转换为弧度
    gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;
    if (size(gnssdata, 2) < 13)
        cfg.usegnssvel = false;
    end
    gnssstarttime = gnssdata(1, 1);
    gnssendtime = gnssdata(end, 1);
else
    % 纯惯导模式：不读取GNSS数据
    gnssdata = [];
    gnssstarttime = [];
    gnssendtime = [];
end

% odo data（仅在需要 ODO 时读取）
if cfg.useodo
    ododata = importdata(cfg.odofilepath);
end


%% save result
navpath = [cfg.outputfolder, '/NavResult'];
if cfg.usegnssvel
    navpath = [navpath, '_GNSSVEL'];
    disp("use GNSS velocity!");
end
% % if cfg.useodonhc
% %     navpath = [navpath, '_ODONHC'];
% %     disp("use ODO velocity!");
% % end
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
imuerrfp = fopen(imuerrpath, 'wt');

stdpath = [cfg.outputfolder, '/NavSTD.txt'];
stdfp = fopen(stdpath, 'wt');

% 里程计比例因子时间序列输出（无量纲）
odoscalepath = [cfg.outputfolder, '/OdoScale.txt'];
odoscalefp = fopen(odoscalepath, 'wt');

% ODO/NHC innov 时间序列输出（单位：m/s；未使用的分量用 NaN 填充）
odonhc_innov_path = [cfg.outputfolder, '/OdoNHCInnov.txt'];
odonhc_innov_fp = fopen(odonhc_innov_path, 'wt');

% ZVT innov 时间序列输出
if isfield(cfg, 'usezvt') && cfg.usezvt
    zupt_innov_path = [cfg.outputfolder, '/ZuptInnov.txt'];
    zupt_innov_fp = fopen(zupt_innov_path, 'wt');
    zyr_innov_path = [cfg.outputfolder, '/ZyrInnov.txt'];
    zyr_innov_fp = fopen(zyr_innov_path, 'wt');
end

% 输出 omega_nb_b（b 系相对 n 系角速度在 b 系表达），用于杆臂速度补偿等分析
omega_nb_path = [cfg.outputfolder, '/Omega_nb_b.txt'];
omega_nb_fp = fopen(omega_nb_path, 'wt');


%% get process time
if cfg.usegnss
    % GNSS/INS组合模式：使用IMU和GNSS的交集时间
    if imustarttime > gnssstarttime
        starttime = imustarttime;
    else
        starttime = gnssstarttime;
    end
    if imuendtime > gnssendtime
        endtime = gnssendtime;
    else
        endtime = imuendtime;
    end
else
    % 纯惯导模式：直接使用IMU数据的时间范围
    starttime = imustarttime;
    endtime = imuendtime;
end

% 使用配置中的时间范围限制（如果指定）
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
if cfg.endtime > endtime
    cfg.endtime = endtime;
end

% ODO/NHC 更新时间初始化
if cfg.useodo
    % odo update time（按照配置的更新频率）
    updateinterval = 1.0 / cfg.odoupdaterate;
    time_to_nextupdate = updateinterval - mod(cfg.starttime, updateinterval);
    odoupdatetime = cfg.starttime + time_to_nextupdate;
end
if cfg.usenhc
    % NHC 更新节拍
    nhc_interval = 1.0 / cfg.odoupdaterate;
    nhc_time_to_next = nhc_interval - mod(cfg.starttime, nhc_interval);
    nhc_updatetime = cfg.starttime + nhc_time_to_next;
end

% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
if cfg.usegnss
    gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
    gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);
end


%% for debug
disp("Start GNSS/INS Processing!");
lastprecent = 0;


%% initialization 
[kf, navstate] = Initialize(cfg);                                           %%%%对卡尔曼滤波初始化
laststate = navstate;

% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

% GNSS索引初始化（仅在使用GNSS更新时）
if cfg.usegnss
    gnssindex = 1;
    while gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < thisimu(1, 1)
        gnssindex = gnssindex + 1;
    end
end

% ODO 索引初始化（仅在需要 ODO 时）
if cfg.useodo
    odoindex = 1;
    while odoindex <= size(ododata, 1) && ododata(odoindex, 1) < thisimu(1, 1)
        odoindex = odoindex + 1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)-1

    %% set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    % 保存原始IMU数据（用于ZVT更新）
    thisimu_raw = thisimu;

    % 记录 omega_nb_b（b 系相对 n 系角速度在 b 系表达），供后续绘图/杆臂补偿使用
    % 计算 win_n = wie_n + wen_n，然后 omega_nb_b = omega_ib_b - Cbn' * win_n
    param = Param();
    wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
    wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
            -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
            -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
    win_n = wie_n + wen_n;
    wib_b = thisimu(2:4, 1) / imudt;
    omega_nb_b = wib_b - navstate.cbn' * win_n;
    if omega_nb_fp > 0
        fprintf(omega_nb_fp, '%12.6f %12.8f %12.8f %12.8f\n', thisimu(1,1), omega_nb_b(1), omega_nb_b(2), omega_nb_b(3));
    end

    %% compensate IMU error
    thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);


    %% 纯惯导模式：仅进行机械编排和误差传播
    if ~cfg.usegnss
        % INS机械编排
        navstate = InsMech(laststate, lastimu, thisimu);
        % 误差传播
        kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    else
        %% GNSS/INS组合模式
        % adjust GNSS index
        while (gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < lastimu(1, 1))
            gnssindex = gnssindex + 1;
        end
        % 若选择"中断GNSS更新"模式，则在指定时间段内跳过GNSS量测
        if block_gnss
            % 检查当前GNSS数据时间是否在任何一个中断区间内
            current_gnss_time = gnssdata(gnssindex, 1);
            is_in_block_interval = false;
            for i = 1:size(block_intervals, 1)
                if current_gnss_time >= block_intervals(i, 1) && current_gnss_time <= block_intervals(i, 2)
                    is_in_block_interval = true;
                    break;
                end
            end
            % 如果在中断区间内，跳过该GNSS数据
            while (gnssindex <= size(gnssdata, 1) && is_in_block_interval)
                gnssindex = gnssindex + 1;
                if gnssindex <= size(gnssdata, 1)
                    current_gnss_time = gnssdata(gnssindex, 1);
                    is_in_block_interval = false;
                    for i = 1:size(block_intervals, 1)
                        if current_gnss_time >= block_intervals(i, 1) && current_gnss_time <= block_intervals(i, 2)
                            is_in_block_interval = true;
                            break;
                        end
                    end
                else
                    break;
                end
            end
        end

        % check whether gnss data is valid
        if (gnssindex > size(gnssdata, 1))
            disp('GNSS file END!');
            break;
        end

        %% determine whether gnss update is required
        if lastimu(1, 1) == gnssdata(gnssindex, 1)
            % do gnss update for the current state
            thisgnss = gnssdata(gnssindex, :)';
            kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, lastimu, imudt);
            [kf, navstate] = ErrorFeedback(kf, navstate);
            gnssindex = gnssindex + 1;
            laststate = navstate;

            % do propagation for current imu data
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
        elseif (lastimu(1, 1) < gnssdata(gnssindex, 1) && thisimu(1, 1) > gnssdata(gnssindex, 1))
            % ineterpolate imu to gnss time
            [firstimu, secondimu] = interpolate(lastimu, thisimu, gnssdata(gnssindex, 1));

            % do propagation for first imu
            imudt = firstimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, firstimu);
            kf = InsPropagate(navstate, firstimu, imudt, kf, cfg.corrtime);

            % do gnss update
            thisgnss = gnssdata(gnssindex, :)';
            kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, firstimu, imudt);
            [kf, navstate] = ErrorFeedback(kf, navstate);
            gnssindex = gnssindex + 1;
            laststate = navstate;
            lastimu = firstimu;

            % do propagation for second imu
            imudt = secondimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, secondimu);
            kf = InsPropagate(navstate, secondimu, imudt, kf, cfg.corrtime);
        else
            %% only do propagation
            % INS mechanization机械编排
            navstate = InsMech(laststate, lastimu, thisimu);
            % error propagation
            kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
        end
    end

    %% ODO/NHC 更新逻辑（根据配置拆分为三个互斥分支）
    
    % 准备工作：如果使用 ODO，先更新 ODO 索引并获取当前里程计速度
    if cfg.useodo
        % 更新 ODO 索引（找到最接近当前时间的里程计数据点）
        while odoindex <= size(ododata, 1) && ododata(odoindex, 1) < thisimu(1, 1)
            odoindex = odoindex + 1;
        end

        % 检查是否到达 ODO 更新时间（按照配置的更新频率）
        if thisimu(1, 1) >= odoupdatetime
            % 直接使用当前历元对应的里程计速度（通过插值获取）
            % 为了满足 GetOdoVel 函数的最小数据要求（至少5个点），
            % 使用当前索引前后各2个历元的数据（共5个点）
            startindex = max(1, odoindex - 2);
            endindex = min(size(ododata, 1), odoindex + 2);
            
            % 如果数据点不足5个，扩展范围
            if (endindex - startindex + 1) < 5
                if startindex == 1
                    endindex = min(size(ododata, 1), startindex + 4);
                elseif endindex == size(ododata, 1)
                    startindex = max(1, endindex - 4);
                end
            end
            
            [current_odovel, is_odo_valid] = GetOdoVel(ododata(startindex:endindex, :), thisimu(1, 1));
            odoupdatetime = odoupdatetime + 1 / cfg.odoupdaterate;
        else
            current_odovel = [];
            is_odo_valid = false;
        end
    else
        current_odovel = [];
        is_odo_valid = false;
    end
    
    % 分支一：同时启用 ODO 和 NHC (cfg.useodo && cfg.usenhc)
    if cfg.useodo && cfg.usenhc
        if is_odo_valid
            % 构建 3 维观测向量：[前向速度(ODO), 侧向速度(NHC=0), 垂向速度(NHC=0)]
            odonhc_vel = [current_odovel; 0; 0];
                kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
            % 记录 innov（ODO+NHC: x/y/z）
            tmp = nan(1,3);
            tmp(kf.odonhc_innov_idx) = kf.odonhc_innov(:)';
            fprintf(odonhc_innov_fp, '%12.6f %12.8f %12.8f %12.8f\n', kf.odonhc_innov_time, tmp(1), tmp(2), tmp(3));
                [kf, navstate] = ErrorFeedback(kf, navstate);
            end
    
    % 分支二：仅 ODO (cfg.useodo && ~cfg.usenhc)
    elseif cfg.useodo && ~cfg.usenhc
        if is_odo_valid
            % 构建 1 维观测标量：仅前向速度（绝对不要补0）
            odonhc_vel = current_odovel;
            kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
            % 记录 innov（仅 ODO: x）
            tmp = nan(1,3);
            tmp(kf.odonhc_innov_idx) = kf.odonhc_innov(:)';
            fprintf(odonhc_innov_fp, '%12.6f %12.8f %12.8f %12.8f\n', kf.odonhc_innov_time, tmp(1), tmp(2), tmp(3));
            [kf, navstate] = ErrorFeedback(kf, navstate);
        end
    
    % 分支三：仅 NHC (~cfg.useodo && cfg.usenhc)
    elseif ~cfg.useodo && cfg.usenhc
        % 检查是否到达 NHC 更新时间
        if thisimu(1, 1) >= nhc_updatetime
            % 构建 2 维观测向量：[侧向速度(NHC=0), 垂向速度(NHC=0)]
            odonhc_vel = [0; 0];
            kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
            % 记录 innov（仅 NHC: y/z）
            tmp = nan(1,3);
            tmp(kf.odonhc_innov_idx) = kf.odonhc_innov(:)';
            fprintf(odonhc_innov_fp, '%12.6f %12.8f %12.8f %12.8f\n', kf.odonhc_innov_time, tmp(1), tmp(2), tmp(3));
            [kf, navstate] = ErrorFeedback(kf, navstate);
            nhc_updatetime = nhc_updatetime + nhc_interval;
        end
    end
    
    %% 零速修正和零航向角速率更新 (ZVT Update)
    if cfg.usezvt
        % 检查当前时间是否在零速区间内
        is_in_zerovel_interval = false;
        if isfield(cfg, 'zerovel_intervals') && ~isempty(cfg.zerovel_intervals)
            current_time = thisimu(1, 1);
            for i = 1:size(cfg.zerovel_intervals, 1)
                if current_time >= cfg.zerovel_intervals(i, 1) && current_time <= cfg.zerovel_intervals(i, 2)
                    is_in_zerovel_interval = true;
                    break;
                end
            end
        end
        
        % 如果在零速区间内，进行零速修正和零航向角速率更新
        if is_in_zerovel_interval
            % 同时进行零速修正和零航向角速率更新
            zvt_update_type = 'both';
            if isfield(cfg, 'zvt_update_type')
                zvt_update_type = cfg.zvt_update_type;
            end
            kf = ZVTUpdate(navstate, kf, cfg, thisimu, imudt, zvt_update_type, thisimu_raw);
            [kf, navstate] = ErrorFeedback(kf, navstate);
            
            % 输出零速修正新息（n系速度，单位：m/s）
            if isfield(kf, 'zupt_innov')
                fprintf(zupt_innov_fp, '%12.6f %12.8f %12.8f %12.8f\n', ...
                    kf.zupt_innov_time, kf.zupt_innov(1), kf.zupt_innov(2), kf.zupt_innov(3));
            end
            
            % 输出零航向角速率更新新息（b系z轴角速度，单位：rad/s）
            if isfield(kf, 'zyr_innov')
                fprintf(zyr_innov_fp, '%12.6f %12.8f\n', ...
                    kf.zyr_innov_time, kf.zyr_innov);
            end
        end
    end

    
    %%%%反馈修正（置零！）：导航参数；传感器误差参数。注意当传感器误差参数较小时（导航级）可以不同时修正，否则无法保证线性假设
    %%%%结果输出：导航结果；传感器误差
    %%%%用两个变量分别存储

    %% save data
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);

    % write imu error, convert to common unit
    imuerror = zeros(13, 1);
    imuerror(1, 1) = navstate.time;
    imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
    imuerror(5:7, 1) = navstate.accbias * 1e5;
    imuerror(8:10, 1) = navstate.gyrscale * 1e6;
    imuerror(11:13, 1) = navstate.accscale * 1e6;
    fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);

    % write state std, convert to common unit
    std = zeros(1, 23);
    std(1) = navstate.time;
    for idx=1:kf.RANK
        std(idx + 1) = sqrt(kf.P(idx, idx));
    end
    std(8:10) = std(8:10) * param.R2D;
    std(11:13) = std(11:13) * param.R2D *3600;
    std(14:16) = std(14:16) * 1e5;
    std(17:22) = std(17:22) * 1e6;
    % std(23) 里程计比例因子，无量纲，不再缩放
    fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);

    % write odometer scale factor (navigation state, already反馈修正)
    fprintf(odoscalefp, '%12.6f %12.8f\n', navstate.time, navstate.odoscale);


    %% print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.01) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end

% close file
fclose(imuerrfp);
fclose(navfp);
fclose(stdfp);
fclose(odoscalefp);
fclose(odonhc_innov_fp);
if isfield(cfg, 'usezvt') && cfg.usezvt
    fclose(zupt_innov_fp);
    fclose(zyr_innov_fp);
end
fclose(omega_nb_fp);

disp("GNSS/INS Integration Processing Finished!");

