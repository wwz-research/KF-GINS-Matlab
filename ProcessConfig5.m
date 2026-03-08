% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Weizhen Wang
% Contact : 
%    Date : 2026.1.16
%
%    NOTE : 基于原始 ProcessConfig5.m 为 dataset5 场景配置的松组合基准配置，
%           提供 GNSS/INS 松组合，并作为 NHC/ODO/ZVT 以及各类敏感性实验
%           （初始协方差、过程噪声）的基准配置文件
% -------------------------------------------------------------------------

function cfg = ProcessConfig5()

    param = Param();

    %% filepath
    cfg.imufilepath = 'dataset5/IMU.bin';
    cfg.gnssfilepath = 'dataset5/GNSSsolution.pos';
    % cfg.gnssfilepath = 'dataset5/MOV_SPP.pos';
    cfg.odofilepath = '';
    cfg.outputfolder = 'dataset5/ins_gnss';

    %% configure
    cfg.usegnss = true;      % 是否启用GNSS更新（位置/速度）
    cfg.usegnssvel = true;
    cfg.useodo = false;       % 启用ODO
    cfg.usenhc = false;       % 启用NHC
    cfg.useodonhc = false;    % 启用ODO和NHC
    cfg.odoupdaterate = 1;    % [Hz]
    cfg.usezvt = false;        % 不使用零速修正和零航向角速率更新
    cfg.initodoscale = 0.063;     % 里程计比例因子初值
    cfg.initodoscalestd = 0.01;   % 里程计比例因子标准差

    %% initial information
    cfg.starttime = 527000;
    cfg.endtime = inf;

    cfg.initpos = [28.1421701503; 112.9585396051; 38.591]; % [deg, deg, m]
    cfg.initvel = [-0.273; -3.942; -0.020]; % [m/s]
    cfg.initatt = [1.40822092; -0.95903515; 267.77618701]; % [deg]

    cfg.initposstd = [0.05; 0.05; 0.1]; %[m]
    cfg.initvelstd = [0.05; 0.05; 0.05]; %[m/s]
    cfg.initattstd = [0.1; 0.1; 0.5]; %[deg]

    cfg.initgyrbias = [100; -300; -100]; % [deg/h]
    cfg.initaccbias = [-1500; -600; 0]; % [mGal]
    cfg.initgyrscale = [1000; 6000; -4000]; % [ppm]
    cfg.initaccscale = [2600; 8400; 300]; % [ppm]
    % 里程计比例因子保持无量纲，不需要单位转换

    %%%% 初始IMU误差的协方差可以参考IMU参数设定，和processconfig2（高精度MEMS）和3（低精度MEMS）中的相同
    cfg.initgyrbiasstd = [48; 48; 48]; % [deg/h]
    cfg.initaccbiasstd = [50; 50; 50]; % [mGal]
    cfg.initgyrscalestd = [500; 500; 500]; % [ppm]
    cfg.initaccscalestd = [500; 500; 500]; % [ppm]

    %%%% 
    cfg.gyrarw = 0.2; % [deg/sqrt(h)]
    cfg.accvrw = 0.2; % [m/s/sqrt(h)]
    cfg.gyrbiasstd = 48; % [deg/h]
    cfg.accbiasstd = 50; % [mGal]
    cfg.gyrscalestd = 500; % [ppm]
    cfg.accscalestd = 500; % [ppm]
    cfg.corrtime = 1; % [h]

    %% install parameters 安装参数
    cfg.antlever = [0.505; -0.145; -1.105]; % [m]
    cfg.odolever = [0.605; -1.025; 0.705]; %[m]
    cfg.installangle = [0; -0.532; 1.38]; %[deg]

    %% ODO/NHC measurement noise 观测噪声
    cfg.odonhc_measnoise = [0.64; 0.1; 0.4]; % [m/s]


    %% convert unit to standard unit (单位转换)
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

    cfg.initattstd = cfg.initattstd * param.D2R;

    cfg.initgyrbias = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias = cfg.initaccbias * 1e-5;
    cfg.initgyrscale = cfg.initgyrscale * 1e-6;
    cfg.initaccscale = cfg.initaccscale * 1e-6;
    cfg.initgyrbiasstd = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw = cfg.accvrw / 60;
    cfg.gyrbiasstd = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;
    cfg.cbv = euler2dcm(cfg.installangle);

end

