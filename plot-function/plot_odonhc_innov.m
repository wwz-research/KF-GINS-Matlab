% -------------------------------------------------------------------------
% 绘制 ODO/NHC innov 时间序列
%
% 输入文件格式（由 kf_gins.m 输出）：
%   OdoNHCInnov.txt: [time, innov_x, innov_y, innov_z]
% 未使用的分量为 NaN。
% -------------------------------------------------------------------------
function plot_odonhc_innov()

    % 根据需要修改输出目录
    base_folder = fullfile('dataset5', 'ins_gnss_odo');
    innov_path = fullfile(base_folder, 'OdoNHCInnov.txt');

    data = importdata(innov_path);
    t = data(:, 1);
    innov = data(:, 2);

    % 统计（对 NaN 自动忽略）
    mu = mean(innov, 1, 'omitnan');
    sd = std(innov,  0, 'omitnan');

    fprintf('ODO/NHC innov 统计（%s）：\n', base_folder);
    fprintf('  x: mean=%.6f, std=%.6f\n', mu(1), sd(1));

    titles = {'innov x (Forward/ODO)', 'innov y (Lateral/NHC)', 'innov z (Down/NHC)'};
    figure;
    for k = 1:1
        subplot(1,1,k);
        plot(t, innov(:,k), 'b-', 'LineWidth', 1.2); hold on;
        yline(0, 'k--', 'LineWidth', 1.0);
        yline(mu(k), 'r--', 'LineWidth', 1.0);
        grid on;
        xlabel('Time [s]'); ylabel('innov [m/s]');
        title(titles{k});
    end

end


