% -------------------------------------------------------------------------
% 读取二进制IMU数据文件
%
%  Author : Weizhen Wang
% Contact : 
%    Date : 2025.12.18
% -------------------------------------------------------------------------

% 读取二进制IMU数据文件
% 输入: filepath - 二进制文件路径
% 输出: imudata - n×7 矩阵 [时间, 陀螺增量(3), 加表增量(3)]
function imudata = ReadBinaryIMU(filepath)
    
    % 打开二进制文件
    fid = fopen(filepath, 'rb');
    if fid == -1
        error(['Cannot open binary IMU file: ', filepath]);
    end
    
    % 读取所有double数据（7列：时间 + 陀螺3轴 + 加表3轴）
    data = fread(fid, [7, inf], 'double');
    fclose(fid);
    
    % 转置为 n×7 矩阵（每行一个历元）
    imudata = data';
    
    fprintf('Successfully read %d epochs from binary IMU file: %s\n', ...
            size(imudata, 1), filepath);
    
end

