% -------------------------------------------------------------------------
% 实验：陀螺零偏过程噪声敏感性分析
%
% 实验目的：分析不同陀螺零偏过程噪声对INS/GNSS松组合导航中
%           状态收敛性和状态误差的影响
%
% 实验变量：
%   Case B1（过小/僵化）：陀螺零偏过程噪声 * 0.01
%   Case B2（基准/适中）：陀螺零偏过程噪声（原始值）
%   Case B3（过大/过度敏感）：陀螺零偏过程噪声 * 100
%
% 观测指标：
%   - 全时段Z轴陀螺零偏估计值（deg/h）
%   - 全时段Z轴陀螺零偏标准差（deg/h）
%   - 全时段航向角误差（deg）
%
% 使用说明：直接运行本脚本即可完成实验和绘图
% -------------------------------------------------------------------------

clear;
clc;
addpath("function\");

%% 实验配置
param = Param();
base_dir = 'dataset5';
output_folders = {
    'dataset5/ins_gnss_gyrbias_B1',  % Case B1: 过程噪声 * 0.01
    'dataset5/ins_gnss_gyrbias_B2',  % Case B2: 基准
    'dataset5/ins_gnss_gyrbias_B3'   % Case B3: 过程噪声 * 100
};
case_names = {'Small Q', 'Baseline Q', 'Large Q'};
case_colors = {[0.850 0.325 0.098], [0 0.447 0.741], [0.929 0.694 0.125]};  % 橙、蓝、黄
case_styles = {'--', '-', '-.'};

% 参考数据路径
ref_path = fullfile(base_dir, 'REF_NAV.nav');

fprintf('========================================\n');
fprintf('陀螺零偏过程噪声敏感性分析实验\n');
fprintf('========================================\n');

%% 步骤1：检查结果文件
fprintf('\n========================================\n');
fprintf('步骤1：检查结果文件\n');
fprintf('========================================\n');

all_files_exist = true;
for i = 1:3
    imu_error_path = fullfile(output_folders{i}, 'ImuError.txt');
    std_path = fullfile(output_folders{i}, 'NavSTD.txt');
    nav_path = fullfile(output_folders{i}, 'NavResult_GNSSVEL.nav');
    if ~exist(imu_error_path, 'file') || ~exist(std_path, 'file') || ~exist(nav_path, 'file')
        all_files_exist = false;
        break;
    end
end

if ~all_files_exist
    fprintf('检测到部分结果文件不存在，需要运行kf_gins.m\n');
    fprintf('请按照以下步骤手动运行kf_gins.m：\n');
    fprintf('  1. 运行 Case B1：修改kf_gins.m第45行，将 ProcessConfig5() 改为 ProcessConfig5B1()\n');
    fprintf('  2. 运行kf_gins.m（选择模式1）\n');
    fprintf('  3. 运行 Case B2：修改kf_gins.m第45行，将 ProcessConfig5B1() 改为 ProcessConfig5B2()\n');
    fprintf('  4. 运行kf_gins.m（选择模式1）\n');
    fprintf('  5. 运行 Case B3：修改kf_gins.m第45行，将 ProcessConfig5B2() 改为 ProcessConfig5B3()\n');
    fprintf('  6. 运行kf_gins.m（选择模式1）\n');
    fprintf('\n完成后，请重新运行本脚本进行绘图分析。\n');
    return;
else
    fprintf('所有结果文件已存在，跳过kf_gins运行步骤，直接进行绘图分析。\n');
end

%% 步骤2：读取结果并绘制对比图
fprintf('\n========================================\n');
fprintf('步骤2：读取数据并绘制对比图\n');
fprintf('========================================\n');

plot_gyrbias_processnoise_comparison(output_folders, case_names, case_colors, case_styles, ref_path);

fprintf('\n实验完成！\n');

%% ========================================================================
% 子函数：绘制陀螺零偏过程噪声对比图
% ========================================================================
function plot_gyrbias_processnoise_comparison(output_folders, case_names, case_colors, case_styles, ref_path)
    
    start_time = 527000;  % 起始时间
    
    % 读取所有案例的数据
    data_cases = cell(3, 1);
    for i = 1:3
        % 读取ImuError.txt（陀螺零偏估计值）
        imu_error_path = fullfile(output_folders{i}, 'ImuError.txt');
        if ~exist(imu_error_path, 'file')
            warning('文件不存在: %s，请先运行kf_gins处理该案例', imu_error_path);
            continue;
        end
        
        imu_error_data = importdata(imu_error_path);
        time_imu = imu_error_data(:, 1) - start_time;  % 相对时间
        gyrbias_z_est = imu_error_data(:, 4);  % Z轴陀螺零偏估计值（deg/h）
        
        % 读取NavSTD.txt（陀螺零偏标准差）
        std_path = fullfile(output_folders{i}, 'NavSTD.txt');
        if ~exist(std_path, 'file')
            warning('文件不存在: %s', std_path);
            continue;
        end
        
        std_data = importdata(std_path);
        time_std = std_data(:, 1) - start_time;  % 相对时间
        gyrbias_z_std = std_data(:, 13);  % Z轴陀螺零偏标准差（deg/h，第13列）
        
        % 读取NavResult_GNSSVEL.nav（导航结果，用于计算航向角误差）
        nav_path = fullfile(output_folders{i}, 'NavResult_GNSSVEL.nav');
        if ~exist(nav_path, 'file')
            warning('文件不存在: %s', nav_path);
            continue;
        end
        
        nav_data = importdata(nav_path);
        time_nav = nav_data(:, 2) - start_time;  % 相对时间
        yaw_nav = nav_data(:, 11);  % 航向角（deg）
        
        data_cases{i}.time_est = time_imu;
        data_cases{i}.gyrbias_z_est = gyrbias_z_est;
        data_cases{i}.time_std = time_std;
        data_cases{i}.gyrbias_z_std = gyrbias_z_std;
        data_cases{i}.time_nav = time_nav;
        data_cases{i}.yaw_nav = yaw_nav;
    end
    
    % 读取参考数据（用于计算航向角误差）
    if ~exist(ref_path, 'file')
        warning('参考文件不存在: %s，无法计算航向角误差', ref_path);
        ref_data = [];
    else
        ref_data = importdata(ref_path);
    end
    
    % Figure 1: Z轴陀螺零偏估计值对比（全时段）
    figure('Name', 'Z轴陀螺零偏估计值对比（过程噪声敏感性）', 'Position', [100, 100, 1200, 600]);
    hold on; grid on;
    for i = 1:3
        if ~isempty(data_cases{i})
            plot(data_cases{i}.time_est, data_cases{i}.gyrbias_z_est, ...
                 'Color', case_colors{i}, 'LineStyle', case_styles{i}, ...
                 'LineWidth', 1.5, 'DisplayName', case_names{i});
        end
    end
    xlabel('时间 [s]', 'FontSize', 12);
    ylabel('Z轴陀螺零偏估计值 [deg/h]', 'FontSize', 12);
    title('不同过程噪声下的Z轴陀螺零偏稳态估计对比', 'FontSize', 14);
    legend('Location', 'best', 'FontSize', 11);
    
    % Figure 2: Z轴陀螺零偏标准差对比（全时段）
    figure('Name', 'Z轴陀螺零偏标准差对比（过程噪声敏感性）', 'Position', [100, 100, 1200, 600]);
    hold on; grid on;
    for i = 1:3
        if ~isempty(data_cases{i})
            plot(data_cases{i}.time_std, data_cases{i}.gyrbias_z_std, ...
                 'Color', case_colors{i}, 'LineStyle', case_styles{i}, ...
                 'LineWidth', 1.5, 'DisplayName', case_names{i});
        end
    end
    xlabel('时间 [s]', 'FontSize', 12);
    ylabel('Z轴陀螺零偏标准差 [deg/h]', 'FontSize', 12);
    title('不同过程噪声下的Z轴陀螺零偏标准差对比', 'FontSize', 14);
    legend('Location', 'best', 'FontSize', 11);
    set(gca, 'YScale', 'log');  % 使用对数坐标，便于观察差异
    
    % Figure 3: 航向角误差对比（全时段）
    if ~isempty(ref_data)
        % 计算航向角误差
        yaw_ref = ref_data(:, 11);  % 参考航向角（deg）
        time_ref = ref_data(:, 2) - start_time;  % 参考时间（相对时间）
        
        figure('Name', '航向角误差对比（过程噪声敏感性）', 'Position', [100, 100, 1200, 600]);
        hold on; grid on;
        for i = 1:3
            if ~isempty(data_cases{i})
                % 将导航结果和参考数据插值到共同时间网格
                % 找到时间重叠区间
                t_start = max([min(data_cases{i}.time_nav), min(time_ref)]);
                t_end = min([max(data_cases{i}.time_nav), max(time_ref)]);
                dt = 0.01;  % 插值时间步长（10ms）
                time_interp = (t_start:dt:t_end)';
                
                % 插值导航航向角
                yaw_nav_interp = interp1(data_cases{i}.time_nav, data_cases{i}.yaw_nav, time_interp, 'linear', 'extrap');
                
                % 插值参考航向角
                yaw_ref_interp = interp1(time_ref, yaw_ref, time_interp, 'linear', 'extrap');
                
                % 计算航向角误差（归一化到[-180, 180]）
                yaw_error = yaw_nav_interp - yaw_ref_interp;
                % 归一化到[-180, 180]
                yaw_error = normalize_angle(yaw_error);
                
                plot(time_interp, yaw_error, ...
                     'Color', case_colors{i}, 'LineStyle', case_styles{i}, ...
                     'LineWidth', 1.5, 'DisplayName', case_names{i});
            end
        end
        xlabel('时间 [s]', 'FontSize', 12);
        ylabel('航向角误差 [deg]', 'FontSize', 12);
        title('不同过程噪声对应的航向角误差对比', 'FontSize', 14);
        legend('Location', 'best', 'FontSize', 11);
    else
        warning('无法绘制航向角误差对比图：参考数据不存在');
    end
end

%% ========================================================================
% 工具函数：归一化角度到[-180, 180]
% ========================================================================
function angles = normalize_angle(angles)
    for i = 1:length(angles)
        while angles(i) > 180
            angles(i) = angles(i) - 360;
        end
        while angles(i) < -180
            angles(i) = angles(i) + 360;
        end
    end
end

