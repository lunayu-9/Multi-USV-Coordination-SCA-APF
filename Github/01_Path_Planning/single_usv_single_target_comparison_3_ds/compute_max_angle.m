% function max_angle = compute_max_angle(path)
%     max_angle = 0;
%     if size(path,1) < 3
%         return;
%     end
%     for i = 2:size(path,1)-1
%         p1 = path(i-1,:);
%         p2 = path(i,:);
%         p3 = path(i+1,:);
%         v1 = p2 - p1;
%         v2 = p3 - p2;
%         angle = atan2d(v2(2), v2(1)) - atan2d(v1(2), v1(1));
%         angle = abs(mod(angle + 180, 360) - 180); % 限制在0-180度
%         if angle > max_angle
%             max_angle = angle;
%         end
%     end
% end



% function max_angle = compute_max_angle(path)
%     max_angle = 0;
%     numPoints = size(path, 1);
%     if numPoints < 3
%         return;
%     end
%     for i = 2:numPoints-1
%         p1 = path(i-1, :);
%         p2 = path(i, :);
%         p3 = path(i+1, :);
% 
%         % 计算连续两段的向量
%         v1 = p2 - p1;
%         v2 = p3 - p2;
% 
%         norm_v1 = norm(v1);
%         norm_v2 = norm(v2);
% 
%         % 如果任意一个向量长度太小，则跳过本次计算（避免除零错误）
%         if norm_v1 < 1e-6 || norm_v2 < 1e-6
%             continue;
%         end
% 
%         % 使用点乘方法计算两向量间夹角（结果范围为 [0, 180] 度）
%         dot_val = dot(v1, v2) / (norm_v1 * norm_v2);
%         % 防止因浮点数误差导致值略超 [-1, 1] 范围
%         dot_val = max(min(dot_val, 1), -1);
%         angle = acosd(dot_val);
% 
%         if angle > max_angle
%             max_angle = angle;
%         end
%     end
% end




function max_angle = compute_max_angle(path)
    % 对路径数据进行平滑处理，减少噪声影响
    if size(path, 1) >= 5
        smooth_path = movmean(path, 3, 1); % 窗口大小3的移动平均
    else
        smooth_path = path;
    end
    
    max_angle = 0;
    numPoints = size(smooth_path, 1);
    if numPoints < 3
        return;
    end
    
    % 设置步长阈值（可根据实际情况调整）
    step_threshold = 0.1;  % 当两点间距离小于该阈值时忽略该段
    for i = 2:numPoints-1
        p1 = smooth_path(i-1, :);
        p2 = smooth_path(i, :);
        p3 = smooth_path(i+1, :);
        
        % 计算连续两段向量
        v1 = p2 - p1;
        v2 = p3 - p2;
        
        norm_v1 = norm(v1);
        norm_v2 = norm(v2);
        
        % 如果任一段距离太短，则跳过此段（避免噪声带来的异常转角）
        if norm_v1 < step_threshold || norm_v2 < step_threshold
            continue;
        end
        
        % 使用点乘方法计算转角
        dot_val = dot(v1, v2) / (norm_v1 * norm_v2);
        dot_val = max(min(dot_val, 1), -1);  % 限制在 [-1,1] 范围内
        angle = acosd(dot_val);
        
        % 如果步长非常短而计算出的转角异常大（如超过150度），则可能是由噪声引起，忽略该次转角
        if angle > 150 && (norm_v1 < 0.5 || norm_v2 < 0.5)
            continue;
        end
        
        if angle > max_angle
            max_angle = angle;
        end
    end
end
