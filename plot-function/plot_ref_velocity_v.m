% -------------------------------------------------------------------------
% 绘制参考文件中的速度（n系 -> b系 -> v系），并对 v 系侧向、垂向速度与 0 作差
% 数据路径固定为 dataset5/NavResult_GNSSVEL.nav（改为使用不加约束松组合的导航结果）
% 结果：
%   - 输出 v 系前向/侧向/垂向速度的均值与方差
%   - 前向、侧向、垂向速度的时间序列，各绘制一个 figure
%   - 通过参考文件 v 系下前向速度判断零速区间并输出（已修改为从 527000 s
%     起计算，零速判定阈值 0.05 m/s，按 5 s 连续时间统计零速区间）
% -------------------------------------------------------------------------
function plot_ref_velocity_v()

    % 路径 NavResult_GNSSVEL
    % ref_path = fullfile('dataset5', 'REF_NAV.nav');
    ref_path = fullfile('dataset5', 'NavResult_GNSSVEL.nav');

    % 读取参考数据
    ref_raw = importdata(ref_path);
    ref     = ref_raw(:, 2:end);     % 去掉索引列

    % 设置初始历元
    start_time = 527000;  % [s]
    
    % 时间过滤：只保留时间 >= start_time 的数据
    time_mask = ref(:, 1) >= start_time;
    ref = ref(time_mask, :);
    
    if isempty(ref)
        error('错误：在时间 %.1f s 之后没有找到数据', start_time);
    end
    
    fprintf('数据时间范围：%.3f s - %.3f s（共 %d 个历元）\n', ...
        ref(1, 1), ref(end, 1), size(ref, 1));

    % 时间、姿态、n系速度
    t_ref   = ref(:, 1);
    v_n_ref = ref(:, 5:7);           % n 系速度 [北, 东, 天]
    att_ref = ref(:, 8:10);          % 姿态角 [roll, pitch, yaw] (deg)

    % 获取 Cbv
    cfg_tmp = ProcessConfig5NHC();   % 任取配置以获得安装角
    Cbv = cfg_tmp.cbv;
    param = Param();

    % n -> b -> v 坐标转换（考虑杆臂影响），优先使用主程序记录的 omega_nb_b 解算值
    % 转换逻辑：v_b = Cbn' * v_n；v_b,lever = v_b + (omega_nb^b x lb)；v_v = Cbv * v_b,lever
    % 这与 ODONHCUpdate.m 的杆臂速度项一致（skew(omega_nb_b)*lb）
    lb = cfg_tmp.odolever;           % 杆臂（b系）

    % 读取主程序输出的 omega_nb_b（时间, wx, wy, wz）
    omega_nb_log = [];
    omega_path = fullfile('dataset5', 'ins_gnss', 'Omega_nb_b.txt');
    if exist(omega_path, 'file')
        omega_nb_log = importdata(omega_path);
    end

    % 预先计算每个历元的 Cbn（b->n）
    N = size(ref, 1);
    Cbn_all = zeros(3, 3, N);
    for i = 1:N
        att_rad = att_ref(i, :) * param.D2R;
        Cbn_all(:,:,i) = euler2dcm(att_rad');
    end

    % 获取 omega_nb_b：优先插值日志值，否则退回姿态差分近似
    omega_nb_b = zeros(N, 3);
    has_log = ~isempty(omega_nb_log) && size(omega_nb_log, 2) >= 4;
    t_omega = omega_nb_log(:, 1);
    for k = 1:3
        omega_nb_b(:, k) = interp1(t_omega, omega_nb_log(:, 1 + k), t_ref, 'linear', 'extrap');
    end
    % if has_log
    %     t_omega = omega_nb_log(:, 1);
    %     for k = 1:3
    %         omega_nb_b(:, k) = interp1(t_omega, omega_nb_log(:, 1 + k), t_ref, 'linear', 'extrap');
    %     end
    % else
    %     % 姿态差分近似（兜底）
    %     for i = 1:N-1
    %         dt_i = t_ref(i+1) - t_ref(i);
    %         if dt_i <= 0
    %             continue;
    %         end
    %         Cbb = Cbn_all(:,:,i)' * Cbn_all(:,:,i+1);   % b_i -> b_{i+1}（在 b_i 中表达）
    %         S = 0.5 * (Cbb - Cbb');                     % 近似反对称部分
    %         rotvec = [S(3,2); S(1,3); S(2,1)];          % vex(S)
    %         omega_nb_b(i, :) = (rotvec / dt_i)';        % rad/s
    %     end
    %     if N >= 2
    %         omega_nb_b(N, :) = omega_nb_b(N-1, :);
    %     end
    %     fprintf('警告：未找到 Omega_nb_b.txt，使用姿态差分近似角速度。\n');
    % end

    % 计算 v 系速度（含杆臂项）
    v_v_ref = zeros(size(v_n_ref));
    for i = 1:N
        Cbn = Cbn_all(:,:,i);
        v_b = Cbn' * v_n_ref(i, :)';
        v_lever_b = skew(omega_nb_b(i, :)') * lb;   % (omega_nb^b x lb)
        v_v_ref(i, :) = (Cbv * (v_b + v_lever_b))';
    end

    % 前向、侧向、垂向速度序列
    forward_series = v_v_ref(:, 1);
    lat_series = v_v_ref(:, 2);
    down_series = v_v_ref(:, 3);

    % 统计
    forward_mean = mean(v_v_ref(:,1));
    forward_std  = std(v_v_ref(:,1));
    lat_mean     = mean(lat_series);
    lat_std      = std(lat_series);
    down_mean    = mean(down_series);
    down_std     = std(down_series);

    fprintf('v 系速度统计：\n');
    fprintf('  Forward: mean = %.6f, std = %.6f\n', forward_mean, forward_std);
    fprintf('  Lateral: mean = %.6f, std = %.6f\n', lat_mean,     lat_std);
    fprintf('  Down    : mean = %.6f, std = %.6f\n', down_mean,    down_std);

    % Figure 1: 侧向速度时间序列（与 0 对比）
    figure;
    plot(t_ref, lat_series, 'b-', 'LineWidth', 1.2); hold on;
    yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', '0');
    yline(lat_mean, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean=%.4f', lat_mean));
    grid on; xlabel('Time [s]'); ylabel('Lateral Vel (v) [m/s]');
    title('Reference Lateral Velocity in v-frame');
    legend('Lateral', '0', 'Mean', 'Location', 'best');

    % Figure 2: 垂向速度时间序列（与 0 对比）
    figure;
    plot(t_ref, down_series, 'b-', 'LineWidth', 1.2); hold on;
    yline(0, 'k--', 'LineWidth', 1.0, 'DisplayName', '0');
    yline(down_mean, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean=%.4f', down_mean));
    grid on; xlabel('Time [s]'); ylabel('Down Vel (v) [m/s]');
    title('Reference Down Velocity in v-frame');
    legend('Down', '0', 'Mean', 'Location', 'best');

    % Figure 3: 前向速度时间序列
    fig_forward = figure;
    plot(t_ref, forward_series, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Forward'); hold on;
    yline(forward_mean, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean=%.4f', forward_mean));
    grid on; xlabel('Time [s]'); ylabel('Forward Vel (v) [m/s]');
    title('Reference Forward Velocity in v-frame');
    legend('Forward', 'Mean', 'Location', 'best');

    %% 检测连续5秒的零速区间
    % 定义零速阈值（可根据实际情况调整）
    zero_vel_threshold = 0.05; % [m/s] - 前向速度绝对值小于此值视为零速0.05 1.0
    min_zero_duration = 5.0;   % [s] - 最小零速持续时间
    
    % 检测零速历元（前向速度绝对值小于阈值）
    is_zero_vel = abs(forward_series) < zero_vel_threshold;
    
    % 找到连续5秒以上的零速区间
    zero_vel_intervals = [];
    start_idx = [];
    start_time = [];
    
    for i = 1:length(is_zero_vel)
        if is_zero_vel(i)
            if isempty(start_idx)
                % 开始一个新的零速区间
                start_idx = i;
                start_time = t_ref(i);
            end
        else
            % 零速状态结束
            if ~isempty(start_idx)
                % 计算零速区间的持续时间
                duration = t_ref(i-1) - start_time;
                if duration >= min_zero_duration
                    % 持续时间达到5秒，记录为零速区间
                    zero_vel_intervals = [zero_vel_intervals; ...
                        start_time, t_ref(i-1), duration, i - start_idx];
                end
            end
            start_idx = [];
            start_time = [];
        end
    end
    
    % 处理最后一个区间（如果数据末尾也是零速）
    if ~isempty(start_idx)
        duration = t_ref(end) - start_time;
        if duration >= min_zero_duration
            zero_vel_intervals = [zero_vel_intervals; ...
                start_time, t_ref(end), duration, length(is_zero_vel) - start_idx + 1];
        end
    end
    
    % 输出零速区间
    fprintf('\n零速区间（连续%.1f秒以上，阈值=%.3f m/s）：\n', min_zero_duration, zero_vel_threshold);
    if isempty(zero_vel_intervals)
        fprintf('  未找到符合条件的零速区间\n');
    else
        fprintf('  区间编号 | 起始时间 [s] | 结束时间 [s] | 持续时间 [s] | 历元数\n');
        fprintf('  --------|--------------|--------------|--------------|--------\n');
        for i = 1:size(zero_vel_intervals, 1)
            fprintf('  %8d | %12.3f | %12.3f | %12.3f | %6d\n', ...
                i, zero_vel_intervals(i, 1), zero_vel_intervals(i, 2), ...
                zero_vel_intervals(i, 3), zero_vel_intervals(i, 4));
        end
        fprintf('  共找到 %d 个零速区间\n', size(zero_vel_intervals, 1));
        
        % 在前向速度图上用淡蓝色背景标记零速区间
        figure(fig_forward);  % 切换到前向速度图
        hold on;  % 确保在现有图形上添加
        
        % 获取当前坐标轴的y轴范围
        y_lim = ylim;
        
        % 为每个零速区间添加淡蓝色背景
        % 只在第一个区间添加图例项，其他区间不显示在图例中
        for i = 1:size(zero_vel_intervals, 1)
            x_start = zero_vel_intervals(i, 1);
            x_end = zero_vel_intervals(i, 2);
            if i == 1
                % 第一个区间添加图例
                fill([x_start, x_end, x_end, x_start], ...
                     [y_lim(1), y_lim(1), y_lim(2), y_lim(2)], ...
                     [0.7, 0.9, 1.0], ...  % 淡蓝色 RGB [0.7, 0.9, 1.0]
                     'EdgeColor', 'none', ...
                     'FaceAlpha', 0.3, ...  % 透明度
                     'DisplayName', 'Zero Vel Interval');
            else
                % 其他区间不添加图例
                fill([x_start, x_end, x_end, x_start], ...
                     [y_lim(1), y_lim(1), y_lim(2), y_lim(2)], ...
                     [0.7, 0.9, 1.0], ...  % 淡蓝色 RGB [0.7, 0.9, 1.0]
                     'EdgeColor', 'none', ...
                     'FaceAlpha', 0.3, ...  % 透明度
                     'HandleVisibility', 'off');
            end
        end
        
        % 将前向速度曲线移到最前面（确保在背景之上）
        h_forward = findobj(gca, 'Type', 'line', 'DisplayName', 'Forward');
        if ~isempty(h_forward)
            uistack(h_forward, 'top');
        end
        
        % 更新图例（按照绘制顺序：前向速度、均值、零速区间）
        legend('Mean', 'Zero Vel Interval', 'Forward', 'Location', 'best');
    end

end

