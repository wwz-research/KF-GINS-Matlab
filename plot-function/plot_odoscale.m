% -------------------------------------------------------------------------
% 绘制里程计比例因子时间序列
% 说明：
%   - 读取主程序输出的 OdoScale.txt（time, odoscale）
%   - 可通过修改 base_folder 对不同模式结果进行查看
% -------------------------------------------------------------------------
function plot_odoscale()

    % 根据需要修改：例如 ins_gnss_odo / ins_gnss_odo_nhc / ins_gnss
    base_folder = fullfile('dataset5', 'ins_gnss_odo');
    odoscale_path = fullfile(base_folder, 'OdoScale.txt');

    data = importdata(odoscale_path);
    t = data(:, 1);
    s = data(:, 2);

    s_mean = mean(s);
    s_std  = std(s);

    fprintf('里程计比例因子统计（%s）：\n', base_folder);
    fprintf('  mean = %.6f\n', s_mean);
    fprintf('  std  = %.6f\n', s_std);

    figure;
    plot(t, s, 'b-', 'LineWidth', 1.2); hold on;
    yline(s_mean, 'r--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean=%.4f', s_mean));
    yline(s_mean + s_std, 'g--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean+STD=%.4f', s_mean + s_std));
    yline(s_mean - s_std, 'g--', 'LineWidth', 1.0, 'DisplayName', sprintf('Mean-STD=%.4f', s_mean - s_std));
    grid on; xlabel('Time [s]'); ylabel('Odo Scale Factor');
    title('Estimated Odometer Scale Factor');
    legend('Scale', 'Mean', 'Mean±STD', 'Location', 'best');

end


