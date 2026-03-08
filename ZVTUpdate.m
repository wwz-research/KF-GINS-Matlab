% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Weizhen Wang
% Contact : 
%    Date : 2026.01.20
%
%    ADD  : 实现零速修正（ZUPT，n系速度为零约束）和零航向角速率更新
% -------------------------------------------------------------------------

function kf = ZVTUpdate(navstate, kf, cfg, thisimu, dt, update_type, thisimu_raw)
% ZVTUpdate: 零速修正和零航向角速率更新
%
% 输入参数:
%   navstate: 导航状态
%   kf: 卡尔曼滤波器结构
%   cfg: 配置参数
%   thisimu: 当前IMU数据 [时间, 角增量, 速度增量]（已补偿）
%   dt: 时间间隔
%   update_type: 更新类型
%       'zupt' - 零速修正
%       'zyr'  - 零航向角速率更新
%       'both' - 同时进行零速修正和零航向角速率更新
%   thisimu_raw: 原始IMU数据 [时间, 角增量, 速度增量]（未补偿，可选）
%
% 输出参数:
%   kf: 更新后的卡尔曼滤波器结构

    param = Param();

    %% 计算几何参数和角速度（用于零航向角速率更新）
    wib_b_raw = thisimu_raw(2:4, 1) / dt;  % b系角速度（原始输出）
    wib_b = thisimu(2:4, 1) / dt;  % b系角速度（补偿后的真实角速度估计值）
    
    % 计算n系相对于i系的角速度
    wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
    wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
            -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
            -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
    win_n = wie_n + wen_n;  % n系相对于i系的角速度
    
    Cbn = navstate.cbn;

    %% 零航向角速率更新 (Zero Yaw Rate Update)
    if strcmp(update_type, 'zyr') || strcmp(update_type, 'both')
        %% 零角速度修正：约束b系相对于n系的角速度为0
        
        % 使用陀螺输出
        omega_ib_b = wib_b; 
        
        % n系相对于i系的角速度在b系中的表示
        omega_in_b = Cbn' * win_n;  
        
        % 惯导解算的b系相对于n系的角速度
        omega_nb_b_hat = omega_ib_b - omega_in_b; 
        
        %% 观测值（零角速度，b系相对于n系）
        omega_nb_b_obs = [0; 0; 0];  % b系相对于n系的角速度观测值（零）
        
        %% 观测残差（预测 - 观测）
        innov_omega = omega_nb_b_hat - omega_nb_b_obs;
        
        %% 构建完整的3x22量测矩阵 H_omega
        H_omega_full = zeros(3, kf.RANK);
        % 陀螺零偏 (10-12): I_3
        H_omega_full(:, 10:12) = eye(3);
        % 陀螺比例因子 (16-18): diag(omega_ib^b)
        % 使用惯导解算的b系角速度（已包含误差）来构建对角矩阵
        H_omega_full(:, 16:18) = diag(wib_b);
        
        % 里程计比例因子 (22): 根据配置确定是否包含
        if kf.RANK >= 22
            H_omega_full(:, 22) = zeros(3, 1);  % 里程计比例因子对角速度无影响
        end
        
        %% 取航向角对应的行（第3行，对应z轴/航向角）
        H_zyr = H_omega_full(3, :);  % 取第3行（航向角对应）
        innov_zyr = innov_omega(3);  % 取第3个分量（航向角对应）
        
        %% 构建噪声矩阵 R
        R_zyr = cfg.zeroyawrate_measnoise^2;
        
        %% 卡尔曼滤波更新
        K_zyr = kf.P * H_zyr' / (H_zyr * kf.P * H_zyr' + R_zyr);
        kf.x = kf.x + K_zyr * (innov_zyr - H_zyr * kf.x);
        kf.P = (eye(kf.RANK) - K_zyr * H_zyr) * kf.P * (eye(kf.RANK) - K_zyr * H_zyr)' + K_zyr * R_zyr * K_zyr';
        
        % 存储新息（用于调试/绘图）
        kf.zyr_innov = innov_zyr;
        kf.zyr_innov_time = navstate.time;
    end

    %% 零速修正 (ZUPT)
    if strcmp(update_type, 'zupt') || strcmp(update_type, 'both')
        %% 预测的n系速度（静止状态）
        vel_n_pred = navstate.vel;  % n系速度预测值
        
        %% 观测值（零速，n系下）
        vel_n_obs = [0; 0; 0];  % n系速度观测值（零速）
        
        %% 观测残差（预测 - 观测）
        innov_zupt = vel_n_pred - vel_n_obs;
        
        %% 构建H矩阵（3x22）
        H_zupt = zeros(3, kf.RANK);
        
        % 速度误差对n系速度的敏感度: dv_n = dv_n（直接对应）
        H_zupt(:, 4:6) = eye(3);
        
        %% 构建噪声矩阵 R
        R_zupt = diag(cfg.zerovel_measnoise.^2);
        
        %% 卡尔曼滤波更新
        K_zupt = kf.P * H_zupt' / (H_zupt * kf.P * H_zupt' + R_zupt);
        kf.x = kf.x + K_zupt * (innov_zupt - H_zupt * kf.x);
        kf.P = (eye(kf.RANK) - K_zupt * H_zupt) * kf.P * (eye(kf.RANK) - K_zupt * H_zupt)' + K_zupt * R_zupt * K_zupt';
        
        % 存储新息（用于调试/绘图）
        kf.zupt_innov = innov_zupt;
        kf.zupt_innov_time = navstate.time;
    end

end

