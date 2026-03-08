% -------------------------------------------------------------------------
% KF-GINS-Matlab: 不同组合模式误差对比绘图
%
% 使用说明：
%   - 直接运行本脚本，无需传入参数；
%   - 默认比较 dataset5 中不同组合模式的结果，
%     如需更换数据集，只需修改下方路径变量。
%
% 默认路径（可按需修改下方变量）：
%   松组合（baseline）：dataset5/ins_gnss/NavResult_GNSSVEL.nav
%   松+NHC          ：dataset5/ins_gnss_nhc/NavResult_GNSSVEL.nav
%   松+ODO          ：dataset5/ins_gnss_odo/NavResult_GNSSVEL.nav
%   松+ODO+NHC      ：dataset5/ins_gnss_odo_nhc/NavResult_GNSSVEL.nav
%   松+ZVT          ：dataset5/ins_gnss_odo_nhc_zvt/NavResult_GNSSVEL.nav
%   参考            ：dataset5/REF_NAV.nav
%
% 输出结果：
%   - 三个 figure：位置误差、速度误差、姿态误差；
%   - 每个 figure 内含三个方向子图，用于对比不同组合模式的误差；
%   - 支持三种时间段选择（全时段 / GNSS 中断段 / 指定零速段），并计算
%     在所选时间段内不同模式相对松组合的误差 RMS 及百分比改进；
%   - 当选择零速时间段时，额外给出 NE 平面位置误差散点图用于直观对比。
% -------------------------------------------------------------------------

%% 配置：需要其他数据集时直接修改这些路径
base_dir      = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset5');
path_loose    = fullfile(base_dir, 'ins_gnss',         'NavResult_GNSSVEL.nav');
path_nhc      = fullfile(base_dir, 'ins_gnss_nhc',     'NavResult_GNSSVEL.nav');
path_odo      = fullfile(base_dir, 'ins_gnss_odo',     'NavResult_GNSSVEL.nav');
path_odonhc   = fullfile(base_dir, 'ins_gnss_odo_nhc', 'NavResult_GNSSVEL.nav');
path_odonhc_zvt = fullfile(base_dir, 'ins_gnss_odo_nhc_zvt', 'NavResult_GNSSVEL.nav');
ref_path      = fullfile(base_dir, 'REF_NAV.nav');

fprintf('[plot_compare_error] 可选路径：\n');
fprintf('  松组合       : %s\n', path_loose);
fprintf('  松+NHC       : %s\n', path_nhc);
fprintf('  松+ODO       : %s\n', path_odo);
fprintf('  松+ODO+NHC   : %s\n', path_odonhc);
fprintf('  松+ZVT : %s\n', path_odonhc_zvt);
fprintf('  参考         : %s\n', ref_path);

plot_compare_error_main(path_loose, path_nhc, path_odo, path_odonhc, path_odonhc_zvt, ref_path);

%% 主流程：读取数据、插值、坐标转换并绘制误差对比
function plot_compare_error_main(path_loose, path_nhc, path_odo, path_odonhc, path_odonhc_zvt, ref_path)

    %% 选择对比模式
    fprintf('\n========================================\n');
    fprintf('请选择对比模式：\n');
    fprintf('  1 - 松组合 vs 松+NHC\n');
    fprintf('  2 - 松组合 vs 松+ODO\n');
    fprintf('  3 - 松组合 vs 松+ODO+NHC\n');
    fprintf('  4 - 四条轨迹一起对比（松、松+NHC、松+ODO、松+ODO+NHC）\n');
    fprintf('  5 - 松组合 vs 松+ZVT\n');
    fprintf('========================================\n');
    cmp_choice = input('请输入选择（1/2/3/4/5）：');

    series = {};
    series{end+1} = struct('name','Loose','path',path_loose,'color',[0 0.447 0.741],'style','-');
    if cmp_choice == 1
        series{end+1} = struct('name','Loose+NHC','path',path_nhc,'color',[0.850 0.325 0.098],'style','--');
    elseif cmp_choice == 2
        series{end+1} = struct('name','Loose+ODO','path',path_odo,'color',[0.494 0.184 0.556],'style','--');
    elseif cmp_choice == 3
        series{end+1} = struct('name','Loose+ODO+NHC','path',path_odonhc,'color',[0.466 0.674 0.188],'style','--');
    elseif cmp_choice == 5
        series{end+1} = struct('name','Loose+ZVT','path',path_odonhc_zvt,'color',[0.929 0.694 0.125],'style','--');
    else
        series{end+1} = struct('name','Loose+NHC','path',path_nhc,'color',[0.850 0.325 0.098],'style','--');
        series{end+1} = struct('name','Loose+ODO','path',path_odo,'color',[0.494 0.184 0.556],'style','-.');
        series{end+1} = struct('name','Loose+ODO+NHC','path',path_odonhc,'color',[0.466 0.674 0.188],'style',':');
    end

    %% 读取参考与各轨迹
    ref_raw = importdata(ref_path);
    ref = ref_raw(:, 2:end);

    traces = series;
    for i = 1:numel(series)
        raw = importdata(series{i}.path);
        traces{i}.data = raw(:, 2:end);
    end

    % 航向角预处理：先将所有轨迹航向角统一归一化到 [-180, 180]，再平滑
    ref = normalize_heading(ref);
    ref = smooth_heading(ref);
    for i = 1:numel(traces)
        traces{i}.data = normalize_heading(traces{i}.data);
        traces{i}.data = smooth_heading(traces{i}.data);
    end

    
    % 构建所有轨迹的时间重叠区间，并按统一时间步长进行插值
    t_first = [ref(1,1), traces{1}.data(1,1)];
    t_last  = [ref(end,1), traces{1}.data(end,1)];
    for i = 2:numel(traces)
        t_first(end+1) = traces{i}.data(1,1);
        t_last(end+1)  = traces{i}.data(end,1);
    end
    starttime = max(t_first);
    endtime   = min(t_last);
    dt        = mean(diff(ref(:, 1)));
    time      = (starttime:dt:endtime)';
    % 时间步长插值：保留时间列作为第 1 列
    ref_interp = zeros(length(time), 10);
    ref_interp(:,1) = time;
    ref_interp(:,2:10) = interp1(ref(:,1), ref(:,2:10), time);
    % 对参考插值后，再归一化航向
    ref_interp = normalize_heading(ref_interp);

    for i = 1:numel(traces)
        traces{i}.interp = zeros(length(time), 10);
        traces{i}.interp(:,1) = time;
        traces{i}.interp(:,2:10) = interp1(traces{i}.data(:,1), traces{i}.data(:,2:10), time);
        traces{i}.interp = normalize_heading(traces{i}.interp);
    end

    %% 统一局部原点与 Cbv、ODO 杆臂（用于将误差统一到 ODO 点）
    param = Param();
    lat0 = ref_interp(1,2) * param.D2R;
    lon0 = ref_interp(1,3) * param.D2R;
    h0   = ref_interp(1,4);
    [rm0, rn0] = getRmRn(lat0, param);
    DR0 = diag([rm0 + h0, (rn0 + h0) * cos(lat0), -1]);
    cfg_tmp = ProcessConfig5();   % 任取一份配置以获取安装矩阵和杆臂（各模式安装角和杆臂一致）
    Cbv = cfg_tmp.cbv;
    lb_odo = cfg_tmp.odolever;   % ODO 杆臂（b 系下，从 IMU 指向 ODO 点）

    % 为每条轨迹分别读取其所在输出文件夹中的 omega_nb_b 日志（用于速度杆臂补偿）
    for i = 1:numel(traces)
        [series_folder, ~, ~] = fileparts(traces{i}.path);
        omega_path_i = fullfile(series_folder, 'Omega_nb_b.txt');
        omega_nb_log_i = [];
        if exist(omega_path_i, 'file')
            omega_nb_log_i = importdata(omega_path_i);
        else
            warning('[plot_compare_error] 轨迹 "%s" 未找到 omega_nb_b 日志文件：%s，该轨迹速度误差将忽略杆臂角速度项', ...
                traces{i}.name, omega_path_i);
        end
        traces{i}.omega_nb_log = omega_nb_log_i;
    end

    %% 计算各轨迹相对于参考的 v 系误差（统一到 ODO 点）
    for i = 1:numel(traces)
        traces{i}.err_v = compute_err_v(traces{i}.interp, ref_interp, DR0, Cbv, param, lb_odo, traces{i}.omega_nb_log);
    end

    %% 绘图时间段选择（增加零速区间选项）
    % 定义四个GNSS中断时间段
    gnss_block_intervals = [
        528250, 528310;
        528480, 528540;
        528660, 528720;
        528840, 528900
    ];
    gnss_block_start = gnss_block_intervals(1, 1);  % 第一个时间段的开始
    gnss_block_end   = gnss_block_intervals(end, 2);  % 最后一个时间段的结束
    pad_sec = 30;  % 前后各30秒
    seg_start = gnss_block_start - pad_sec;  % 528220
    seg_end   = gnss_block_end + pad_sec;    % 528930
    % 零速区间（来自 ZVT 配置）：仅用于额外 NE 平面误差分析
    zupt_start = 530700.000;
    zupt_end   = 530934.990;

    fprintf('\n========================================\n');
    fprintf('请选择绘图时间段：\n');
    fprintf('  1 - 绘制全部时间段\n');
    fprintf('  2 - 仅绘制包含GNSS中断时间的时间段（%.0f-%.0f，前后各+%.0f秒）\n', gnss_block_start, gnss_block_end, pad_sec);
    fprintf('  3 - 仅绘制零速区间 [%.3f, %.3f]\n', zupt_start, zupt_end);
    fprintf('========================================\n');
    plot_choice = input('请输入选择（1 / 2 / 3）：');
    
    if plot_choice == 2
        mask = (time >= seg_start) & (time <= seg_end);
        time_plot = time(mask);
        for i = 1:numel(traces)
            traces{i}.plot_err = traces{i}.err_v(mask,:);
        end
        fprintf('已选择：局部时间段绘图 [%.3f, %.3f]\n', time_plot(1), time_plot(end));
        % 四个GNSS中断时间段，用蓝色背景显示
        shade_windows = cell(size(gnss_block_intervals, 1), 1);
        for i = 1:size(gnss_block_intervals, 1)
            shade_windows{i} = [gnss_block_intervals(i, 1), gnss_block_intervals(i, 2)];
        end
    elseif plot_choice == 3
        % 仅绘制零速区间
        mask = (time >= zupt_start) & (time <= zupt_end);
        time_plot = time(mask);
        for i = 1:numel(traces)
            traces{i}.plot_err = traces{i}.err_v(mask,:);
        end
        fprintf('已选择：零速区间绘图 [%.3f, %.3f]\n', time_plot(1), time_plot(end));
        shade_windows = {};
    else
        % 全时间段
        rms_end_time = 530500;
        mask = (time >= time(1)) & (time <= rms_end_time);
        time_plot = time(mask);
        for i = 1:numel(traces)
            traces{i}.plot_err = traces{i}.err_v(mask,:);
        end
        fprintf('已选择：全时间段绘图 [%.3f, %.3f]\n', time_plot(1), time_plot(end));
        shade_windows = {};
    end

    % 绘图参数设置（颜色、线宽等）
    lw1 = 1.5;

    % 1）位置误差对比（v 系：Forward / Lateral / Down）
    figure;
    titles = {'Forward Error', 'Lateral Error', 'Down Error'};
    ylabels = {'Error [m]', 'Error [m]', 'Error [m]'};
    for k = 1:3
        subplot(3,1,k);
        hold on;
        for i = 1:numel(traces)
            plot(time_plot, traces{i}.plot_err(:, k), 'Color', traces{i}.color, 'LineWidth', lw1, 'LineStyle', traces{i}.style);
        end
        if plot_choice == 2, add_time_shading(gca, shade_windows); end
        grid on; xlabel('Time [s]'); ylabel(ylabels{k}); title(titles{k});
        if k == 1
            leg = cellfun(@(s)s.name, traces, 'UniformOutput', false);
            legend(leg, 'Location', 'best');
        end
    end

    % 2）速度误差对比（v 系：Forward / Lateral / Down）
    figure;
    titles = {'Vel Forward Error', 'Vel Lateral Error', 'Vel Down Error'};
    ylabels = {'Error [m/s]', 'Error [m/s]', 'Error [m/s]'};
    for k = 1:3
        subplot(3,1,k);
        hold on;
        for i = 1:numel(traces)
            plot(time_plot, traces{i}.plot_err(:, 3 + k), 'Color', traces{i}.color, 'LineWidth', lw1, 'LineStyle', traces{i}.style);
        end
        if plot_choice == 2, add_time_shading(gca, shade_windows); end
        grid on; xlabel('Time [s]'); ylabel(ylabels{k}); title(titles{k});
        if k == 1
            leg = cellfun(@(s)s.name, traces, 'UniformOutput', false);
            legend(leg, 'Location', 'best');
        end
    end

    % 3）姿态误差对比（导航系欧拉角）
    figure;
    titles = {'Roll Error', 'Pitch Error', 'Yaw Error'};
    ylabels = {'Error [deg]', 'Error [deg]', 'Error [deg]'};
    for k = 1:3
        subplot(3,1,k);
        hold on;
        for i = 1:numel(traces)
            plot(time_plot, traces{i}.plot_err(:, 6 + k), 'Color', traces{i}.color, 'LineWidth', lw1, 'LineStyle', traces{i}.style);
        end
        if plot_choice == 2, add_time_shading(gca, shade_windows); end
        grid on; xlabel('Time [s]'); ylabel(ylabels{k}); title(titles{k});
        if k == 1
            leg = cellfun(@(s)s.name, traces, 'UniformOutput', false);
            legend(leg, 'Location', 'best');
        end
    end

    %% 4）计算并输出误差RMS统计
    if plot_choice == 1 || plot_choice == 2 || plot_choice == 3
        % 确保至少有两条轨迹（松组合 + 另一种模式）
        if numel(traces) >= 2
            % 第一条轨迹是松组合（baseline）
            trace_loose = traces{1};
            trace_other = traces{2};
            
            if plot_choice == 1
                % 全时间段
                rms_end_time = 530500;
                mask_rms = (time >= time(1)) & (time <= rms_end_time);
                err_loose_all = trace_loose.err_v(mask_rms, :);
                err_other_all = trace_other.err_v(mask_rms, :);
                time_range_str = sprintf('[%.3f - %.3f s]', time(1), min(rms_end_time, time(end)));
                fprintf('\n========================================\n');
                fprintf('全时间段误差RMS统计 %s\n', time_range_str);
                fprintf('========================================\n');
            elseif plot_choice == 2
                % GNSS中断时间段：仅使用四个中断时间段内的数据计算RMS
                % 注意：不包括未中断的区域，只统计中断时间段内的误差
                mask_gnss_block = false(size(time));
                for i = 1:size(gnss_block_intervals, 1)
                    mask_gnss_block = mask_gnss_block | ...
                        ((time >= gnss_block_intervals(i, 1)) & (time <= gnss_block_intervals(i, 2)));
                end
                % 只使用中断时间段内的数据进行RMS计算
                err_loose_all = trace_loose.err_v(mask_gnss_block, :);
                err_other_all = trace_other.err_v(mask_gnss_block, :);
                time_range_str = sprintf('四个中断时间段: [%.0f-%.0f, %.0f-%.0f, %.0f-%.0f, %.0f-%.0f] s', ...
                    gnss_block_intervals(1,1), gnss_block_intervals(1,2), ...
                    gnss_block_intervals(2,1), gnss_block_intervals(2,2), ...
                    gnss_block_intervals(3,1), gnss_block_intervals(3,2), ...
                    gnss_block_intervals(4,1), gnss_block_intervals(4,2));
                fprintf('\n========================================\n');
                fprintf('GNSS中断时间段误差RMS统计 %s\n', time_range_str);
                fprintf('========================================\n');
            else
                % 零速区间：仅使用530700-530934.990内的数据
                mask_zupt = (time >= zupt_start) & (time <= zupt_end);
                err_loose_all = trace_loose.err_v(mask_zupt, :);
                err_other_all = trace_other.err_v(mask_zupt, :);
                time_range_str = sprintf('[%.3f - %.3f s]', zupt_start, zupt_end);
                fprintf('\n========================================\n');
                fprintf('零速区间误差RMS统计 %s\n', time_range_str);
                fprintf('========================================\n');
            end
            
            % 位置误差RMS（m）
            fprintf('\n位置误差RMS [m]:\n');
            fprintf('%-20s | %12s | %12s | %12s\n', '模式', 'Forward', 'Lateral', 'Down');
            fprintf('%s\n', repmat('-', 1, 60));
            
            rms_loose_pos = sqrt(mean(err_loose_all(:, 1:3).^2, 1));
            rms_other_pos = sqrt(mean(err_other_all(:, 1:3).^2, 1));
            
            fprintf('%-20s | %12.4f | %12.4f | %12.4f\n', trace_loose.name, rms_loose_pos(1), rms_loose_pos(2), rms_loose_pos(3));
            fprintf('%-20s | %12.4f | %12.4f | %12.4f\n', trace_other.name, rms_other_pos(1), rms_other_pos(2), rms_other_pos(3));
            
            % 计算改善百分比
            improvement_pos = (rms_loose_pos - rms_other_pos) ./ rms_loose_pos * 100;
            fprintf('%-20s | %12.2f%% | %12.2f%% | %12.2f%%\n', '改善百分比', improvement_pos(1), improvement_pos(2), improvement_pos(3));
            
            % 速度误差RMS（m/s）
            fprintf('\n速度误差RMS [m/s]:\n');
            fprintf('%-20s | %12s | %12s | %12s\n', '模式', 'Forward', 'Lateral', 'Down');
            fprintf('%s\n', repmat('-', 1, 60));
            
            rms_loose_vel = sqrt(mean(err_loose_all(:, 4:6).^2, 1));
            rms_other_vel = sqrt(mean(err_other_all(:, 4:6).^2, 1));
            
            fprintf('%-20s | %12.6f | %12.6f | %12.6f\n', trace_loose.name, rms_loose_vel(1), rms_loose_vel(2), rms_loose_vel(3));
            fprintf('%-20s | %12.6f | %12.6f | %12.6f\n', trace_other.name, rms_other_vel(1), rms_other_vel(2), rms_other_vel(3));
            
            % 计算改善百分比
            improvement_vel = (rms_loose_vel - rms_other_vel) ./ rms_loose_vel * 100;
            fprintf('%-20s | %12.2f%% | %12.2f%% | %12.2f%%\n', '改善百分比', improvement_vel(1), improvement_vel(2), improvement_vel(3));
            
            % 姿态误差RMS（deg）
            fprintf('\n姿态误差RMS [deg]:\n');
            fprintf('%-20s | %12s | %12s | %12s\n', '模式', 'Roll', 'Pitch', 'Yaw');
            fprintf('%s\n', repmat('-', 1, 60));
            
            rms_loose_att = sqrt(mean(err_loose_all(:, 7:9).^2, 1));
            rms_other_att = sqrt(mean(err_other_all(:, 7:9).^2, 1));
            
            fprintf('%-20s | %12.4f | %12.4f | %12.4f\n', trace_loose.name, rms_loose_att(1), rms_loose_att(2), rms_loose_att(3));
            fprintf('%-20s | %12.4f | %12.4f | %12.4f\n', trace_other.name, rms_other_att(1), rms_other_att(2), rms_other_att(3));
            
            % 计算改善百分比
            improvement_att = (rms_loose_att - rms_other_att) ./ rms_loose_att * 100;
            fprintf('%-20s | %12.2f%% | %12.2f%% | %12.2f%%\n', '改善百分比', improvement_att(1), improvement_att(2), improvement_att(3));
            
            fprintf('\n========================================\n');
        else
            fprintf('\n警告：需要至少两条轨迹才能进行RMS对比分析\n');
        end
    end

    %% 5）若选择零速区间，则额外绘制 N-E 平面位置误差对比图（导航系）
    if plot_choice == 3
        figure;
        hold on; grid on;
        xlabel('North Error [m]');
        ylabel('East Error [m]');
        title('Position Error in N-E Plane (Zero-velocity Interval)');
        
        % 逐条轨迹计算 N/E 方向误差（导航系），与 compute_err_v 中位置误差计算保持一致
        for i = 1:numel(traces)
            % 找到当前轨迹对应的时间掩码（与 time_plot 一致）
            mask_ne = (time >= zupt_start) & (time <= zupt_end);
            idx_all = find(mask_ne);
            ne_err = zeros(numel(idx_all), 2);  % [dN, dE]
            for k = 1:numel(idx_all)
                idx = idx_all(k);
                % nav 轨迹与参考轨迹的 BLH 差值
                d_blh_nav = [traces{i}.interp(idx,2)*param.D2R - ref_interp(1,2)*param.D2R; ...
                             traces{i}.interp(idx,3)*param.D2R - ref_interp(1,3)*param.D2R; ...
                             traces{i}.interp(idx,4) - ref_interp(1,4)];
                d_blh_ref = [ref_interp(idx,2)*param.D2R - ref_interp(1,2)*param.D2R; ...
                             ref_interp(idx,3)*param.D2R - ref_interp(1,3)*param.D2R; ...
                             ref_interp(idx,4) - ref_interp(1,4)];
                r_n_nav = DR0 * d_blh_nav;
                r_n_ref = DR0 * d_blh_ref;
                dP_n = r_n_nav - r_n_ref;  % NED 位置误差
                ne_err(k,1) = dP_n(1);     % North
                ne_err(k,2) = dP_n(2);     % East
            end
            % 使用散点图绘制
            scatter(ne_err(:,1), ne_err(:,2), 20, traces{i}.color, 'filled', 'MarkerEdgeColor', traces{i}.color, 'LineWidth', 0.5);
        end
        axis equal;
        leg = cellfun(@(s)s.name, traces, 'UniformOutput', false);
        legend(leg, 'Location', 'best');
    end
end

%% 计算单条轨迹相对参考的 v 系误差（统一到 ODO 杆臂点）
% 位置：IMU 点 BLH -> n 系 -> 加上各自姿态下的 ODO 杆臂 -> v 系（参考姿态）
% 速度：IMU n 系速度 -> b 系 + (omega_nb^b x lb_odo) -> v 系，然后做差
function err_v = compute_err_v(nav_interp, ref_interp, DR0, Cbv, param, lb_odo, omega_nb_log)
    n = size(nav_interp, 1);
    err_v = zeros(n, 9);

    % 预计算每个历元的姿态 DCM
    Cbn_nav_all = zeros(3, 3, n);
    Cbn_ref_all = zeros(3, 3, n);
    Cnv_ref_all = zeros(3, 3, n);
    for i = 1:n
        att_nav = nav_interp(i,8:10) * param.D2R;
        att_ref = ref_interp(i,8:10) * param.D2R;
        Cbn_nav = euler2dcm(att_nav');
        Cbn_ref = euler2dcm(att_ref');
        Cbn_nav_all(:,:,i) = Cbn_nav;
        Cbn_ref_all(:,:,i) = Cbn_ref;
        Cnv_ref_all(:,:,i) = Cbv * Cbn_ref';
    end

    % 预插值 omega_nb_b 到当前轨迹时间轴（若日志存在）
    omega_nb_b = zeros(n, 3);
    has_omega = ~isempty(omega_nb_log) && size(omega_nb_log, 2) >= 4;
    if has_omega
        t_omega = omega_nb_log(:, 1);
        t_nav   = nav_interp(:, 1);
        for k = 1:3
            omega_nb_b(:, k) = interp1(t_omega, omega_nb_log(:, 1 + k), t_nav, 'linear', 'extrap');
        end
    end

    for i = 1:n
        %-----------------------------
        % 1）位置误差：统一到 ODO 点，再转到 v 系
        %-----------------------------
        % IMU 点 NED 位置
        d_blh_nav = [nav_interp(i,2)*param.D2R - ref_interp(1,2)*param.D2R; ...
                     nav_interp(i,3)*param.D2R - ref_interp(1,3)*param.D2R; ...
                     nav_interp(i,4) - ref_interp(1,4)];
        d_blh_ref = [ref_interp(i,2)*param.D2R - ref_interp(1,2)*param.D2R; ...
                     ref_interp(i,3)*param.D2R - ref_interp(1,3)*param.D2R; ...
                     ref_interp(i,4) - ref_interp(1,4)];
        r_n_nav = DR0 * d_blh_nav;
        r_n_ref = DR0 * d_blh_ref;

        % ODO 点 n 系位置：r_ODO^n = r_IMU^n + Cbn * lb_odo
        Cbn_nav = Cbn_nav_all(:,:,i);
        Cbn_ref = Cbn_ref_all(:,:,i);
        r_n_odo_nav = r_n_nav + Cbn_nav * lb_odo;
        r_n_odo_ref = r_n_ref + Cbn_ref * lb_odo;

        % 位置误差（n 系）
        dP_n = r_n_odo_nav - r_n_odo_ref;

        % 转到 v 系（使用参考姿态下的 Cnv）
        Cnv_ref = Cnv_ref_all(:,:,i);
        err_v(i,1:3) = (Cnv_ref * dP_n)';

        %-----------------------------
        % 2）速度误差：考虑 ODO 杆臂和角速度项
        %-----------------------------
        v_n_nav = nav_interp(i,5:7)';
        v_n_ref = ref_interp(i,5:7)';

        % IMU 点 b 系速度
        v_b_nav = Cbn_nav' * v_n_nav;
        v_b_ref = Cbn_ref' * v_n_ref;

        % 杆臂角速度项（若有 omega_nb_b 日志则使用，否则视为 0）
        if has_omega
            omega_b = omega_nb_b(i, :)';
            v_lever_nav_b = skew(omega_b) * lb_odo;
        else
            v_lever_nav_b = zeros(3,1);
            % 参考轨迹缺少对应的 omega_nb_b 日志，这里不对参考速度做杆臂角速度补偿
        end

        % ODO 点 v 系速度（导航轨迹使用自身 omega_nb_b，参考轨迹仅做刚体旋转）
        v_v_nav = Cbv * (v_b_nav + v_lever_nav_b);
        v_v_ref = Cbv * v_b_ref;

        dV_v = v_v_nav - v_v_ref;
        err_v(i,4:6) = dV_v';

        %-----------------------------
        % 3）姿态误差（deg），并将航向限制到 [-180,180]
        %-----------------------------
        att_err = (nav_interp(i,8:10) - ref_interp(i,8:10));
        while att_err(3) > 180, att_err(3) = att_err(3) - 360; end
        while att_err(3) < -180, att_err(3) = att_err(3) + 360; end
        err_v(i,7:9) = att_err;
    end
end

%% 本文件内使用的简单 skew 运算（将 3x1 向量转为反对称矩阵）
function S = skew(w)
    S = [   0   -w(3)  w(2);
          w(3)    0   -w(1);
         -w(2)  w(1)   0  ];
end

%% 工具函数：展开航向角，避免跨越 -180/180 时出现突变
function data_out = smooth_heading(data_in)
    data_out = data_in;
    for i = 2:size(data_in, 1)
        if (data_out(i, 10) - data_out(i-1, 10)) < -180
            data_out(i:end, 10) = data_out(i:end, 10) + 360;
        end
        if (data_out(i, 10) - data_out(i-1, 10)) > 180
            data_out(i:end, 10) = data_out(i:end, 10) - 360;
        end
    end
end

%% 工具函数：把航向角限制到 [-180, 180] 区间
function data_out = normalize_heading(data_in)
    data_out = data_in;
    for i = 1:size(data_in, 1)
        while data_out(i, 10) > 180
            data_out(i, 10) = data_out(i, 10) - 360;
        end
    end
end

%% 工具函数：在当前坐标轴上添加时间段背景色（淡色区分）
function add_time_shading(ax, windows)
    if nargin < 2 || isempty(windows)
        return;
    end
    yl = ylim(ax);
    hold(ax, 'on');
    % 使用蓝色背景显示GNSS中断时间段
    blue_color = [0.5 0.7 1.0];  % 淡蓝色
    for i = 1:numel(windows)
        t0 = windows{i}(1);
        t1 = windows{i}(2);
        patch(ax, [t0 t1 t1 t0], [yl(1) yl(1) yl(2) yl(2)], blue_color, ...
              'FaceAlpha', 0.25, 'EdgeColor', 'none');
    end
end

