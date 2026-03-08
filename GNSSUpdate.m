% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

% GNSS位置更新和速度更新
function kf = GNSSUpdate(navstate, gnssdata, kf, antlever, usegnssvel, thisimu, dt)

    param = Param();

    %% GNSS position update
    % abandon gnss vel outlier 
    gnssposstd = gnssdata(5:7, 1);
    if gnssposstd(1, 1) > 5 || gnssposstd(2, 1) > 5 || gnssposstd(3, 1) > 5
        disp(['WARNING: Abandon gnss position measurement at: ', num2str(gnssdata(1, 1))]);
    else
        % measurement innovation
        DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
        Z = DR*(navstate.pos - gnssdata(2:4, 1))+navstate.cbn*antlever;%N系下的NED
        
        % measurement matrix and noise matrix
        R = diag(power(gnssdata(5:7, 1), 2));%m m m 原始实现
        % % % 对位置标准差进行放大：水平方向乘以40，垂直方向乘以70
        % % % 因为GNSS位置标准差通常偏小，需要适当放大以反映实际精度
        % % gnssposstd_scaled = gnssdata(5:7, 1);
        % % gnssposstd_scaled(1:2) = gnssposstd_scaled(1:2) * 40;  % 水平方向（北向、东向）放大40倍
        % % gnssposstd_scaled(3) = gnssposstd_scaled(3) * 70;      % 垂直方向（天向）放大70倍
        % % R = diag(power(gnssposstd_scaled, 2));%m m m
        H = zeros(3, kf.RANK);
        H(1:3, 1:3) = eye(3);
        H(1:3, 7:9) = skew(navstate.cbn * antlever);
        
        % update covariance and state vector
        K = kf.P * H' / (H * kf.P * H' + R);
        kf.x = kf.x + K*(Z - H*kf.x);
        kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    end

    %% GNSS velocity update
    if usegnssvel
        
        
        % abandon gnss vel outlier 
        gnssvelstd = gnssdata(11:13, 1);
        if gnssvelstd(1, 1) > 0.5 || gnssvelstd(2, 1) > 0.5 || gnssvelstd(3, 1) > 0.5
            disp(['WARNING: Abandon gnss velocity measurement at: ', num2str(gnssdata(1, 1))]);
        else
            wie_n = [param.WGS84_WIE * cos(navstate.pos(1)); 0; -param.WGS84_WIE * sin(navstate.pos(1))];
            wen_n = [navstate.vel(2) / (navstate.Rn + navstate.pos(3)); 
                    -navstate.vel(1) / (navstate.Rm + navstate.pos(3)); 
                    -navstate.vel(2) * tan(navstate.pos(1)) / (navstate.Rn + navstate.pos(3))];
            win_n = wie_n + wen_n;
            wib_b = thisimu(2:4) / dt;
            vel_ant = navstate.vel - skew(win_n) * navstate.cbn * antlever - navstate.cbn * (skew(antlever) * wib_b);

            % measurement innovation, noise, matrix
            Z = vel_ant - gnssdata(8:10, 1);
            R = diag(power(gnssvelstd, 2));
            H = zeros(3, kf.RANK);
            H(1:3, 4:6) = eye(3);
            H(1:3, 7:9) = -skew(win_n) * skew(navstate.cbn * antlever) - skew(navstate.cbn * skew(antlever) * wib_b);
            H(1:3, 10:12) = -navstate.cbn * skew(antlever);
            H(1:3, 16:18) = -navstate.cbn * skew(antlever) * diag(wib_b);

            % update covariance and state vector
            K = kf.P * H' / (H * kf.P * H' + R);
            kf.x = kf.x + K*(Z - H*kf.x);
            kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
        end
        
        % % % ===== 重新实现：GNSS速度观测更新 =====
        % % % 提取速度观测值的标准差
        % % velocity_std = gnssdata(11:13, 1);
        % % 
        % % % 粗差检测：如果速度标准差过大，舍弃该观测值
        % % max_std_threshold = 0.5; % m/s
        % % if velocity_std(1) > max_std_threshold || velocity_std(2) > max_std_threshold || velocity_std(3) > max_std_threshold
        % %     disp(['WARNING: Abandon gnss velocity measurement at: ', num2str(gnssdata(1, 1))]);
        % % else
        % %     % 获取当前位置和速度
        % %     lat = navstate.pos(1);
        % %     height = navstate.pos(3);
        % %     vel_n = navstate.vel;
        % %     Rm = navstate.Rm;
        % %     Rn = navstate.Rn;
        % %     Cbn = navstate.cbn;
        % % 
        % %     % 计算地球自转角速度在导航坐标系中的投影
        % %     omega_ie_n = zeros(3, 1);
        % %     omega_ie_n(1) = param.WGS84_WIE * cos(lat);
        % %     omega_ie_n(2) = 0.0;
        % %     omega_ie_n(3) = -param.WGS84_WIE * sin(lat);
        % %     % 计算导航坐标系相对地球的角速度（由载体运动引起）
        % %     omega_en_n = zeros(3, 1);
        % %     omega_en_n(1) = vel_n(2) / (Rn + height);
        % %     omega_en_n(2) = -vel_n(1) / (Rm + height);
        % %     omega_en_n(3) = -vel_n(2) * tan(lat) / (Rn + height);
        % %     % 导航坐标系相对惯性坐标系的角速度
        % %     omega_in_n = omega_ie_n + omega_en_n;
        % % 
        % %     % IMU载体角速度（从增量计算）
        % %     gyro_increment = thisimu(2:4);
        % %     omega_ib_b = gyro_increment / dt;
        % % 
        % %     % 计算GNSS天线处的速度（考虑杠杆臂影响）
        % %     % 杠杆臂在导航系中的位置
        % %     lever_arm_n = Cbn * antlever;
        % %     % 由于载体旋转引起的附加速度项
        % %     vel_rotation = skew(omega_in_n) * lever_arm_n;
        % %     % 由于载体角速度在体坐标系中的影响
        % %     lever_arm_b = antlever;
        % %     vel_gyro = Cbn * (skew(lever_arm_b) * omega_ib_b);
        % %     % 天线处的速度 = IMU处速度 - 旋转引起的速度变化
        % %     velocity_at_antenna = vel_n - vel_rotation - vel_gyro;
        % % 
        % %     % 提取GNSS速度观测值（北东地）
        % %     gnss_velocity_obs = gnssdata(8:10, 1);
        % % 
        % %     % 观测残差（新息）
        % %     innovation = velocity_at_antenna - gnss_velocity_obs;
        % % 
        % %     % 观测噪声协方差矩阵
        % %     R_vel = diag(velocity_std .^ 2);
        % % 
        % %     % 观测矩阵初始化
        % %     H_vel = zeros(3, kf.RANK);
        % %     % 速度误差对观测的贡献
        % %     H_vel(1:3, 4:6) = eye(3);
        % %     % 姿态误差对观测的贡献（通过杠杆臂影响）
        % %     term1 = skew(omega_in_n) * skew(lever_arm_n);
        % %     term2 = skew(Cbn * (skew(lever_arm_b) * omega_ib_b));
        % %     H_vel(1:3, 7:9) = -term1 - term2;
        % %     % 陀螺零偏误差对观测的贡献
        % %     H_vel(1:3, 10:12) = -Cbn * skew(lever_arm_b);
        % %     % 陀螺比例因子误差对观测的贡献
        % %     H_vel(1:3, 16:18) = -Cbn * skew(lever_arm_b) * diag(omega_ib_b);
        % % 
        % %     % 卡尔曼滤波更新
        % %     % 计算卡尔曼增益
        % %     S = H_vel * kf.P * H_vel' + R_vel;
        % %     K = kf.P * H_vel' / S;
        % % 
        % %     % 更新状态向量
        % %     kf.x = kf.x + K * (innovation - H_vel * kf.x);
        % % 
        % %     % 更新协方差矩阵（采用Joseph形式保证数值稳定性）
        % %     I_KH = eye(kf.RANK) - K * H_vel;
        % %     kf.P = I_KH * kf.P * I_KH' + K * R_vel * K';
        % % end
    end
end