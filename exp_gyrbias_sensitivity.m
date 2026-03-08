% -------------------------------------------------------------------------
% 实验：初始陀螺零偏协方差敏感性分析
%
% 实验目的：分析不同初始陀螺零偏协方差对INS/GNSS松组合导航中
%           状态收敛性和状态误差的影响
%
% 实验变量：
%   Case A1（过小/盲目自信）：初始陀螺零偏协方差 * 0.01
%   Case A2（基准/适中）：初始陀螺零偏协方差（原始值）
%   Case A3（过大/过度恐慌）：初始陀螺零偏协方差 * 100
%
% 观测指标：
%   - 前600秒的Z轴陀螺零偏估计值（deg/h）
%   - 前600秒的Z轴陀螺零偏标准差（deg/h）
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
    'dataset5/ins_gnss_gyrbias_A1',  % Case A1: 初始协方差 * 0.01
    'dataset5/ins_gnss_gyrbias_A2',  % Case A2: 基准
    'dataset5/ins_gnss_gyrbias_A3'   % Case A3: 初始协方差 * 100
};
case_names = {'Small P0', 'Baseline P0', 'Large P0'};
case_colors = {[0.850 0.325 0.098], [0 0.447 0.741], [0.929 0.694 0.125]};  % 橙、蓝、黄
case_styles = {'--', '-', '-.'};

% 初始陀螺零偏协方差缩放因子
scale_factors = [0.01, 1.0, 100.0];

% 时间截取范围（前600秒）
time_limit = 600;  % 秒
start_time = 527000;  % 起始时间

fprintf('========================================\n');
fprintf('初始陀螺零偏协方差敏感性分析实验\n');
fprintf('========================================\n');

%% 步骤1：运行三次kf_gins，每次使用不同的初始协方差
fprintf('\n========================================\n');
fprintf('步骤1：运行三次kf_gins处理\n');
fprintf('========================================\n');

% 检查是否所有结果文件都已存在
all_files_exist = true;
for i = 1:3
    imu_error_path = fullfile(output_folders{i}, 'ImuError.txt');
    std_path = fullfile(output_folders{i}, 'NavSTD.txt');
    if ~exist(imu_error_path, 'file') || ~exist(std_path, 'file')
        all_files_exist = false;
        break;
    end
end

if ~all_files_exist
    fprintf('检测到部分结果文件不存在，需要运行kf_gins.m\n');
    fprintf('请按照以下步骤手动运行kf_gins.m：\n');
    fprintf('  1. 运行 Case A1：修改kf_gins.m第45行，将 ProcessConfig5() 改为 ProcessConfig5A1()\n');
    fprintf('  2. 运行kf_gins.m（选择模式1）\n');
    fprintf('  3. 运行 Case A2：修改kf_gins.m第45行，将 ProcessConfig5A1() 改为 ProcessConfig5A2()\n');
    fprintf('  4. 运行kf_gins.m（选择模式1）\n');
    fprintf('  5. 运行 Case A3：修改kf_gins.m第45行，将 ProcessConfig5A2() 改为 ProcessConfig5A3()\n');
    fprintf('  6. 运行kf_gins.m（选择模式1）\n');
    fprintf('\n完成后，请重新运行本脚本进行绘图分析。\n');
    return;
else
    fprintf('所有结果文件已存在，跳过kf_gins运行步骤，直接进行绘图分析。\n');
end

%% 步骤2：读取结果并绘制对比图
plot_gyrbias_comparison(output_folders, case_names, case_colors, case_styles, start_time, time_limit);

fprintf('\n实验完成！\n');

%% ========================================================================
% 子函数：创建带缩放初始陀螺零偏协方差的配置（已废弃，使用ProcessConfig5A1/A2/A3）
% ========================================================================
function cfg = create_config_with_gyrbias_std(case_idx, scale_factor, output_folder)
    % 基于ProcessConfig5创建配置，仅修改初始陀螺零偏协方差
    param = Param();
    
    %% filepath
    cfg.imufilepath = 'dataset5/IMU.bin';
    cfg.gnssfilepath = 'dataset5/GNSSsolution.pos';
    cfg.odofilepath = '';
    cfg.outputfolder = output_folder;
    
    %% configure
    cfg.usegnss = true;
    cfg.usegnssvel = true;
    cfg.useodo = false;
    cfg.usenhc = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1;
    cfg.usezvt = false;
    cfg.initodoscale = 0.063;
    cfg.initodoscalestd = 0.01;
    
    %% initial information
    cfg.starttime = 527000;
    cfg.endtime = inf;
    
    cfg.initpos = [28.1421701503; 112.9585396051; 38.591];
    cfg.initvel = [-0.273; -3.942; -0.020];
    cfg.initatt = [1.40822092; -0.95903515; 267.77618701];
    
    cfg.initposstd = [0.05; 0.05; 0.1];
    cfg.initvelstd = [0.05; 0.05; 0.05];
    cfg.initattstd = [0.1; 0.1; 0.5];
    
    cfg.initgyrbias = [100; -300; -100];
    cfg.initaccbias = [-1500; -600; 0];
    cfg.initgyrscale = [1000; 6000; -4000];
    cfg.initaccscale = [2600; 8400; 300];
    
    % 关键修改：缩放初始陀螺零偏协方差
    cfg.initgyrbiasstd = [48; 48; 48] * scale_factor;  % [deg/h]
    cfg.initaccbiasstd = [50; 50; 50];
    cfg.initgyrscalestd = [500; 500; 500];
    cfg.initaccscalestd = [500; 500; 500];
    
    cfg.gyrarw = 0.2;
    cfg.accvrw = 0.2;
    cfg.gyrbiasstd = 48;
    cfg.accbiasstd = 50;
    cfg.gyrscalestd = 500;
    cfg.accscalestd = 500;
    cfg.corrtime = 1;
    
    %% 安装参数
    cfg.antlever = [0.505; -0.145; -1.105];
    cfg.odolever = [0.605; -1.025; 0.705];
    cfg.installangle = [0; -0.532; 1.38];
    
    cfg.odonhc_measnoise = [0.64; 0.3; 0.4];
    
    %% 单位转换
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


%% ========================================================================
% 子函数：绘制陀螺零偏对比图
% ========================================================================
function plot_gyrbias_comparison(output_folders, case_names, case_colors, case_styles, start_time, time_limit)
    
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
        
        % 截取前time_limit秒的数据
        mask_imu = (time_imu >= 0) & (time_imu <= time_limit);
        mask_std = (time_std >= 0) & (time_std <= time_limit);
        
        data_cases{i}.time_est = time_imu(mask_imu);
        data_cases{i}.gyrbias_z_est = gyrbias_z_est(mask_imu);
        data_cases{i}.time_std = time_std(mask_std);
        data_cases{i}.gyrbias_z_std = gyrbias_z_std(mask_std);
    end
    
    % Figure 1: Z轴陀螺零偏估计值对比（横轴：0-140s）
    figure('Name', 'Z轴陀螺零偏估计值对比', 'Position', [100, 100, 1200, 600]);
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
    title('不同初始方差下的Z轴陀螺零偏收敛过程对比（估计值）', 'FontSize', 14);
    legend('Location', 'best', 'FontSize', 11);
    xlim([0, 140]);  % 横轴改到140s
    
    % Figure 2: Z轴陀螺零偏标准差对比（横轴：0-200s）
    figure('Name', 'Z轴陀螺零偏标准差对比', 'Position', [100, 100, 1200, 600]);
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
    title('不同初始方差下的Z轴陀螺零偏收敛过程对比（标准差）', 'FontSize', 14);
    legend('Location', 'best', 'FontSize', 11);
    xlim([0, 200]);  % 横轴改到200s
    set(gca, 'YScale', 'log');  % 使用对数坐标，便于观察差异
end

