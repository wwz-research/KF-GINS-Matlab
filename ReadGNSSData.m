% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Weizhen Wang
% Contact : 
%    Date : 2026.1.6
% -------------------------------------------------------------------------

% 读取新格式的GNSS数据文件
% 新格式：GPS周 GPS秒 latitude(deg) longitude(deg) height(m) Q ns sdn(m) sde(m) sdu(m) 
%         sdne(m) sdeu(m) sdun(m) age(s) ratio vn(m/s) ve(m/s) vu(m/s) 
%         sdvn sdve sdvu sdvne sdveu sdvun
% 数据头以"%"开头，需要跳过
% 输出格式：13列
%  列1: 时间戳(秒)
%  列2: 纬度(度，后续会转换为弧度)
%  列3: 经度(度，后续会转换为弧度)
%  列4: 高度(米)
%  列5: 纬度方向位置标准差(米) - 使用sdn近似
%  列6: 经度方向位置标准差(米) - 使用sde近似
%  列7: 高度方向位置标准差(米) - 使用sdu
%  列8: 北向速度(m/s)
%  列9: 东向速度(m/s)
%  列10: 地向速度(m/s) - 从vu(天向)取负得到
%  列11: 北向速度标准差(m/s)
%  列12: 东向速度标准差(m/s)
%  列13: 地向速度标准差(m/s) - 使用sdvu(天向速度标准差)，标准差为正值
function gnssdata = ReadGNSSData(filepath)

    % 打开文件
    fid = fopen(filepath, 'r');
    if fid == -1
        error('Cannot open GNSS file: %s', filepath);
    end
    
    % 读取所有行
    data_lines = {};
    line_count = 0;
    
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line)
            % 跳过以"%"开头的注释行
            if ~isempty(line) && line(1) ~= '%'
                line_count = line_count + 1;
                data_lines{line_count} = line;
            end
        end
    end
    
    fclose(fid);
    
    % 解析数据
    if line_count == 0
        error('No valid data found in GNSS file: %s', filepath);
    end
    
    % 预分配数据矩阵
    gnssdata = zeros(line_count, 13);
    
    % 解析每一行数据
    for i = 1:line_count
        line = data_lines{i};
        % 去除首尾空格，然后使用textscan分割多个空格
        line_trimmed = strtrim(line);
        % 使用textscan按空格分割
        tokens_cell = textscan(line_trimmed, '%s', 'Delimiter', ' ', 'MultipleDelimsAsOne', true);
        tokens = tokens_cell{1};
        
        % 检查是否有足够的列
        if length(tokens) < 23
            warning('Line %d does not have enough columns (%d < 23), skipping...', i, length(tokens));
            continue;
        end
        
        try
            % 提取所需的数据列
            % 列1: GPS周秒
            % GPS时间戳 = GPS秒 (604800是一周的秒数)
            gps_sec = str2double(tokens{2});       % GPS秒
            gnssdata(i, 1) = gps_sec;  % 转换为统一时间戳(秒)
            
            % 列2: latitude (纬度，度)
            gnssdata(i, 2) = str2double(tokens{3});
            
            % 列3: longitude (经度，度)
            gnssdata(i, 3) = str2double(tokens{4});
            
            % 列4: height (高度，米)
            gnssdata(i, 4) = str2double(tokens{5});
            
            % 列5-7: 位置标准差 (sdn, sde, sdu)
            gnssdata(i, 5) = str2double(tokens{8});  % sdn (北向标准差，近似为纬度方向)
            gnssdata(i, 6) = str2double(tokens{9});  % sde (东向标准差，近似为经度方向)
            gnssdata(i, 7) = str2double(tokens{10});  % sdu (天向标准差，用于高度)
            
            % 列8-10: 速度 (vn, ve, vu)
            gnssdata(i, 8) = str2double(tokens{16});  % vn (北向速度)
            gnssdata(i, 9) = str2double(tokens{17});  % ve (东向速度)
            gnssdata(i, 10) = -str2double(tokens{18});  % vu (天向速度，取负得到地向速度)
            
            % 列11-13: 速度标准差 (sdvn, sdve, sdvu)
            gnssdata(i, 11) = str2double(tokens{19});  % sdvn (北向速度标准差)
            gnssdata(i, 12) = str2double(tokens{20});  % sdve (东向速度标准差)
            gnssdata(i, 13) = str2double(tokens{21});  % sdvu (天向速度标准差，直接用于地向，标准差为正值)
            
        catch ME
            warning('Error parsing line %d: %s. Skipping...', i, ME.message);
            % 如果解析失败，使用NaN标记
            gnssdata(i, :) = NaN;
        end
    end
    
    % 移除包含NaN的行
    gnssdata = gnssdata(~any(isnan(gnssdata), 2), :);
    
    if isempty(gnssdata)
        error('No valid data parsed from GNSS file: %s', filepath);
    end
    
end

