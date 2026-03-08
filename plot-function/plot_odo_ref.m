% -------------------------------------------------------------------------
% 绘制里程计原始前向速度与参考前向速度，并计算误差
% 数据路径已固定：
%   ODO  : dataset5/ODO.txt   （列1时间，列2前向速度）
%   参考 : dataset5/REF_NAV.nav（列2时间，列5-7为n系速度，列8-10为姿态角）
% 注意：需要将参考数据的 n 系速度转换到 v 系（车体坐标系）的前向速度，
%       并采用与 NHC 一致的安装角矩阵 Cbv，用于设计 ODO 噪声和零速阈值
% -------------------------------------------------------------------------
function plot_odo_ref()

    % 路径
    odo_path = fullfile('dataset5', 'ODO.txt');
    ref_path = fullfile('dataset5', 'NavResult_GNSSVEL.nav');

    % 读取数据
    odo_raw = importdata(odo_path);
    ref_raw = importdata(ref_path);

    % ODO: 时间与前向速度
    t_odo   = odo_raw(:, 1);
    v_odo   = odo_raw(:, 2);

    % 参考：去掉首列索引
    % 列1: 时间，列2-4: BLH，列5-7: n系速度，列8-10: 姿态角（度）
    ref     = ref_raw(:, 2:end);
    t_ref   = ref(:, 1);
    v_n_ref = ref(:, 5:7);  % n系速度 [北, 东, 天]
    att_ref = ref(:, 8:10); % 姿态角 [roll, pitch, yaw] (度)

    % 获取Cbv矩阵和ODO杆臂（从配置文件）
    param = Param();
    cfg_tmp = ProcessConfig5NHC();
    Cbv = cfg_tmp.cbv;
    lb = cfg_tmp.odolever;   % ODO 杆臂（b系下，从 IMU 指向 ODO 点）

    % 读取主程序输出的 omega_nb_b 日志（用于速度杆臂补偿）
    omega_nb_log = [];
    omega_path = fullfile('dataset5', 'ins_gnss', 'Omega_nb_b.txt');
    if exist(omega_path, 'file')
        omega_nb_log = importdata(omega_path);
        fprintf('已从 %s 读取 omega_nb_b 日志\n', omega_path);
    else
        warning('未找到 omega_nb_b 日志文件 %s，速度转换中将忽略杆臂引起的角速度项', omega_path);
    end

    % 将参考数据插值到ODO时间
    % 先插值n系速度和姿态角
    v_n_ref_interp = zeros(length(t_odo), 3);
    att_ref_interp = zeros(length(t_odo), 3);
    for i = 1:3
        v_n_ref_interp(:, i) = interp1(t_ref, v_n_ref(:, i), t_odo, 'linear', 'extrap');
        att_ref_interp(:, i) = interp1(t_ref, att_ref(:, i), t_odo, 'linear', 'extrap');
    end

    % 预先计算每个历元的 Cbn（b->n）
    N = length(t_odo);
    Cbn_all = zeros(3, 3, N);
    for i = 1:N
        att_rad = att_ref_interp(i, :) * param.D2R;
        Cbn_all(:,:,i) = euler2dcm(att_rad');
    end

    % 获取 omega_nb_b：优先插值日志值
    omega_nb_b = zeros(N, 3);
    has_log = ~isempty(omega_nb_log) && size(omega_nb_log, 2) >= 4;
    if has_log
        t_omega = omega_nb_log(:, 1);
        for k = 1:3
            omega_nb_b(:, k) = interp1(t_omega, omega_nb_log(:, 1 + k), t_odo, 'linear', 'extrap');
        end
    end

    % 将n系速度转换到v系前向速度（考虑杆臂影响）
    % 转换逻辑：v_b = Cbn' * v_n；v_b,lever = v_b + (omega_nb^b x lb)；v_v = Cbv * v_b,lever
    % 这与 ODONHCUpdate.m 和 plot_ref_velocity_v.m 的杆臂速度项一致
    v_v_forward_ref = zeros(length(t_odo), 1);
    for i = 1:length(t_odo)
        Cbn = Cbn_all(:,:,i);
        % n系速度转b系速度
        v_b = Cbn' * v_n_ref_interp(i, :)';
        % 杆臂角速度项（若有 omega_nb_b 日志则使用，否则视为 0）
        if has_log
            omega_b = omega_nb_b(i, :)';
            v_lever_b = skew(omega_b) * lb;   % (omega_nb^b x lb)
        else
            v_lever_b = zeros(3,1);
        end
        % b系速度+杆臂项转到v系
        v_v = Cbv * (v_b + v_lever_b);
        % 取前向速度（v系x方向）
        v_v_forward_ref(i) = v_v(1);
    end

    % 误差
    vel_err = v_odo - v_v_forward_ref;
    err_mean = mean(vel_err);
    err_std  = std(vel_err);

    fprintf('========================================\n');
    fprintf('ODO 前向速度误差统计（整段时间）：\n');
    fprintf('  mean = %.6f m/s\n', err_mean);
    fprintf('  std  = %.6f m/s\n', err_std);
    fprintf('========================================\n');

    % Figure 1: ODO 与参考前向速度
    figure;
    plot(t_odo, v_odo, 'b-', 'LineWidth', 1.2); hold on;
    plot(t_odo, v_v_forward_ref, 'r--', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Forward Velocity [m/s]');
    title('ODO Forward Velocity vs Reference (v-frame)');
    legend('ODO', 'Reference (v-frame, interp)', 'Location', 'best');

    % Figure 2: 误差
    figure;
    plot(t_odo, vel_err, 'k-', 'LineWidth', 1.2); hold on;
    yline(err_mean, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean=%.4f', err_mean));
    yline(err_mean + err_std, 'g--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean+STD=%.4f', err_mean+err_std));
    yline(err_mean - err_std, 'g--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean-STD=%.4f', err_mean-err_std));
    grid on; xlabel('Time [s]'); ylabel('Error [m/s]');
    title('ODO Forward Velocity Error (ODO - Reference)');
    legend('Error', 'Mean', 'Mean±STD', 'Location', 'best');

    %% Figure 3: (参考/ODO - 1) 时间序列，并统计均值与标准差
    % 注意：ODO 速度为 0 会导致除零，这里进行屏蔽
    eps_v = 1e-3;
    scale_series = v_v_forward_ref ./ v_odo - 1;
    % 对 |v_odo| 很小的点直接置零，避免除零和异常放大
    mask_small = abs(v_odo) <= eps_v;
    scale_series(mask_small) = 0;

    scale_mean = mean(scale_series);
    scale_std  = std(scale_series);
    fprintf('参考/ODO - 1 统计（|v_odo|<=%.1e 的点置零）：\n', eps_v);
    fprintf('  mean = %.6e\n', scale_mean);
    fprintf('  std  = %.6e\n', scale_std);
    fprintf('========================================\n');

    figure;
    plot(t_odo, scale_series, 'm-', 'LineWidth', 1.2); hold on;
    yline(scale_mean, 'k--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean=%.3e', scale_mean));
    yline(scale_mean + scale_std, 'c--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean+STD=%.3e', scale_mean+scale_std));
    yline(scale_mean - scale_std, 'c--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean-STD=%.3e', scale_mean-scale_std));
    grid on; xlabel('Time [s]'); ylabel('ref/odo - 1');
    title('Scale-like Series: (Reference / ODO) - 1');
    legend('Series', 'Mean', 'Mean±STD', 'Location', 'best');

end

%% 本文件内使用的简单 skew 运算（将 3x1 向量转为反对称矩阵）
function S = skew(w)
    S = [   0   -w(3)  w(2);
          w(3)    0   -w(1);
         -w(2)  w(1)   0  ];
end

