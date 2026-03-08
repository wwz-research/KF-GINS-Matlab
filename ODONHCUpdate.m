% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.9
%
%    MOD  : 重写为统一的 ODO/NHC 观测更新入口，根据观测维度自适应选择
%           1D ODO、2D NHC 或 3D ODO+NHC 模式，并构造相应的 H、R、残差（Weizhen Wang, 2026）
% -------------------------------------------------------------------------

function kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, dt)

    param = Param();

    %% 根据输入观测向量维度自适应判断模式
    m = size(odonhc_vel, 1);
    if m == 3
        % ODO + NHC 模式：使用前向、侧向、垂向
        idx = [1, 2, 3];
        has_odo = true;
        has_nhc = true;
    elseif m == 1
        % 仅 ODO 模式：仅使用前向
        idx = 1;
        has_odo = true;
        has_nhc = false;
    elseif m == 2
        % 仅 NHC 模式：使用侧向、垂向
        idx = [2, 3];
        has_odo = false;
        has_nhc = true;
    else
        error('ODONHCUpdate: 无效的观测向量维度 %d，应为 1、2 或 3', m);
    end

    %% 计算几何参数和角速度
    wib_b = thisimu(2:4, 1) / dt;
    wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
    wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
            -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
            -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
    win_n = wie_n + wen_n;
    wnb_b = wib_b - navstate.cbn' * win_n;

    %% 预测的车体坐标系(v系)速度，含安装角转换和杆臂角速度项
    Cbn = navstate.cbn;
    Cbv = cfg.cbv;
    vIMU_n = navstate.vel;
    omega_in_n = win_n;
    lb = cfg.odolever;

    vel_b = Cbn' * vIMU_n;                 % b系速度
    vel_lever = skew(wnb_b) * lb;          % 杆臂引起的附加速度（b系）
    vel_v_base = Cbv * (vel_b + vel_lever); % 转到v系（基础速度，未考虑里程计比例因子）

    % 重要：仅对前向速度应用里程计比例因子
    vel_v_obs = odonhc_vel;
    if has_odo
        % 仅前向速度受里程计比例因子影响
        vel_v_obs(1) = (1 + navstate.odoscale) * odonhc_vel(1);
    end

    %% 观测残差（预测 - 观测）
    % 根据观测维度构建完整预测向量
    if m == 1
        % 仅 ODO：只取前向
        vel_v_pre = vel_v_base(1);
    elseif m == 2
        % 仅 NHC：只取侧向和垂向
        vel_v_pre = vel_v_base(2:3);
    else
        % ODO + NHC：取全部三个分量
        vel_v_pre = vel_v_base;
    end
    
    innov = vel_v_pre - vel_v_obs;
    % 将 innov 暂存到 kf 结构体中，便于主程序统一写文件/绘图
    % 注意：这里不直接 fprintf，避免高频更新导致命令行输出过多
    kf.odonhc_innov = innov;
    kf.odonhc_innov_m = m;
    kf.odonhc_innov_idx = idx;
    kf.odonhc_innov_time = navstate.time;

    %% 构建完整的 3x22 测量矩阵 H_full
    H_full = zeros(3, kf.RANK);

    % dv: Cbv * Cbn'（对所有速度分量都适用）
    H_full(:, 4:6) = Cbv * Cbn';

    % dphi: H1 = -[ Cbv*Cbn'*(vIMU_n x) + Cbv*(lb x)*Cbn'*(omega_in_n x) ]
    H1 = -(Cbv * Cbn' * skew(vIMU_n) + Cbv * skew(lb) * Cbn' * skew(omega_in_n));
    H_full(:, 7:9) = H1;

    % dbg: H2 = -Cbv*(lb x)
    H2 = -(Cbv * skew(lb));
    H_full(:, 10:12) = H2;

    % dsg: H3 = -Cbv*(lb x)*diag(omega_ib^b)
    H3 = -(Cbv * skew(lb) * diag(wib_b));
    H_full(:, 16:18) = H3;

    % 里程计比例因子状态对前向速度观测的敏感度（仅当包含 ODO 时）
    if has_odo
        H_full(1, 22) = - odonhc_vel(1);
    end

    % 根据实际使用的观测分量切片 H 矩阵
    H = H_full(idx, :);

    %% 构建噪声矩阵 R
    % 从配置中提取对应分量的噪声
    if m == 1
        % 仅 ODO：使用前向噪声
        R = cfg.odonhc_measnoise(1)^2;
    elseif m == 2
        % 仅 NHC：使用侧向和垂向噪声
        R = diag(cfg.odonhc_measnoise(2:3).^2);
    else
        % ODO + NHC：使用全部三个分量噪声
        R = diag(cfg.odonhc_measnoise.^2);
    end

    %% 卡尔曼滤波更新
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (innov - H * kf.x);
    kf.P = (eye(kf.RANK) - K * H) * kf.P * (eye(kf.RANK) - K * H)' + K * R * K';

end
