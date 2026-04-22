function experiment_100_runs_smooth_turn_v2()
    %% 实验参数设置
    total_experiments = 100;
    % 随机抽取两次实验（索引）用于最终展示
    selected_exp_indices = sort(randperm(total_experiments, 2));
    
    % 用于记录每次实验的统计数据
    path_lengths = zeros(total_experiments, 1);
    success_flags = zeros(total_experiments, 1);  % 1: 避障成功，0: 失败
    max_turn_angles = zeros(total_experiments, 1);
    experiment_times = zeros(total_experiments, 1);  % 记录每次实验的运行时间（秒）
    
    % 存储每次实验的所有数据，便于后续可视化和分析
    results = struct();
    % 全局记录所有实验中每一步的转向角
    global_turning_angles = [];
    
    %% 地图与目标设置
    MAP_size = [40, 40];
    m = MAP_size(1); n = MAP_size(2);
    start = [5, 5];
    goal  = [35, 35];
    
    % 静态障碍物构造（0 表示障碍，采用 [row, col] 存储）
    MAP = ones(m, n);
    for i = 20:24
        MAP(i, 20) = 0;
        MAP(i, 24) = 0;
    end
    MAP(30:32, 28:30) = 0;
    MAP(18:20, 32:34) = 0;
    [obs_i, obs_j] = find(MAP == 0);
    obs = [obs_i, obs_j];
    
    %% 人工势场参数设置
    Eta_att = 10;       % 吸引力增益
    Eta_rep_ob = 100;   % 障碍物斥力增益
    sensing_range = 15; % 扩大预感知范围（原 d0 = 10）
    d0 = sensing_range; % 斥力影响范围
    len_step = 0.3;     % 每步移动距离
    
    % 碰撞判断阈值
    collision_threshold = 1;
    % 防止无限循环
    max_iter = 1000;
    
    %% 开始 100 次实验（全部在“不可见”模式下运行）
    for exp = 1:total_experiments
        % 开始计时
        t_exp = tic;
        
        % 初始化无人船状态与路径记录
        current_pos = start;
        path = start;
        
        % 初始化动态障碍物（总数 5：原 3 + 新增 2，此处按代码设定为 1）
        num_dyn = 1;
        % 随机布置在起点与目标之间的区域
        dyn_obs = [10 + rand(num_dyn,1)*(goal(1)-10), 10 + rand(num_dyn,1)*(goal(2)-10)];
        dyn_speed = 0.2;
        dyn_vel = zeros(num_dyn, 2);
        for i = 1:num_dyn
            theta = rand() * 2*pi;
            dyn_vel(i,:) = dyn_speed * [cos(theta), sin(theta)];
        end
        % 每个动态障碍物的历史轨迹
        dyn_history = cell(num_dyn,1);
        for i = 1:num_dyn
            dyn_history{i} = dyn_obs(i,:);
        end
        
        % 初始化用于记录碰撞与轨迹交叉的点
        collision_points = [];
        intersection_points = [];
        
        collision_flag = false;
        iter = 0;
        
        % 初始化上一时刻的航向角，取从起点到目标的方向
        prev_heading = atan2(goal(2)-current_pos(2), goal(1)-current_pos(1));
        
        % 模拟路径规划过程（不绘图，仅更新状态）
        while norm(current_pos - goal) >= 0.3 && iter < max_iter
            iter = iter + 1;
            % 更新动态障碍物的位置及其历史轨迹
            for i = 1:num_dyn
                dyn_obs(i,:) = dyn_obs(i,:) + dyn_vel(i,:);
                dyn_history{i} = [dyn_history{i}; dyn_obs(i,:)];
                % 边界检测：若超出地图边界则反向
                if dyn_obs(i,1) < 1 || dyn_obs(i,1) > n
                    dyn_vel(i,1) = -dyn_vel(i,1);
                end
                if dyn_obs(i,2) < 1 || dyn_obs(i,2) > m
                    dyn_vel(i,2) = -dyn_vel(i,2);
                end
            end
            
            % 计算吸引力（朝向目标）
            delta_goal = goal - current_pos;
            dist_goal = norm(delta_goal);
            unit_vec_goal = delta_goal / dist_goal;
            F_att = Eta_att * unit_vec_goal;
            
            % 计算静态障碍物斥力
            F_rep = [0, 0];
            for j = 1:size(obs,1)
                % 将障碍物坐标从 [row, col] 转为 [x,y]
                obs_pos = [obs(j,2), obs(j,1)];
                delta_obs = current_pos - obs_pos;
                dist_obs = norm(delta_obs);
                if dist_obs < d0 && dist_obs > 0
                    unit_vec_obs = delta_obs / dist_obs;
                    F_rep = F_rep + Eta_rep_ob*(1/dist_obs - 1/d0)*(1/dist_obs^2)*unit_vec_obs;
                end
            end
            
            % 计算动态障碍物斥力（当前障碍物位置）
            for j = 1:num_dyn
                delta_dyn = current_pos - dyn_obs(j,:);
                dist_dyn = norm(delta_dyn);
                if dist_dyn < d0 && dist_dyn > 0
                    unit_vec_dyn = delta_dyn / dist_dyn;
                    F_rep = F_rep + Eta_rep_ob*(1/dist_dyn - 1/d0)*(1/dist_dyn^2)*unit_vec_dyn;
                end
            end
            
            % 合力计算
            F_sum = F_att + F_rep;
            F_norm = norm(F_sum);
            if F_norm ~= 0
                UnitVec_Fsum = F_sum / F_norm;
            else
                UnitVec_Fsum = [cos(prev_heading), sin(prev_heading)]; % 无力时保持原方向
            end
            
            % 原始期望航向
            desired_heading = atan2(UnitVec_Fsum(2), UnitVec_Fsum(1));
            % 计算航向变化量，并归一化到 [-pi, pi]
            raw_delta_heading = desired_heading - prev_heading;
            raw_delta_heading = mod(raw_delta_heading + pi, 2*pi) - pi;
            
            % 引入阻尼因子，避免骤变（例如 0.5 表示只沿预期方向走一半）
            damping_factor = 0.5;
            delta_heading = damping_factor * raw_delta_heading;
            
            % 限制转向角不超过30度（30°=pi/6 弧度）
            max_turn = pi/6;
            if abs(delta_heading) > max_turn
                delta_heading = sign(delta_heading) * max_turn;
            end
            
            % 新的航向角
            new_heading = prev_heading + delta_heading;
            
            % 更新无人船位置（采用新的航向角）
            new_pos = current_pos + len_step * [cos(new_heading), sin(new_heading)];
            
            % 检测与动态障碍物当前位置的碰撞
            if any(vecnorm(dyn_obs - new_pos, 2, 2) < collision_threshold)
                collision_points = [collision_points; new_pos];
                collision_flag = true;
                path = [path; new_pos];
                break;  % 碰撞则退出循环
            end
            
            % 检查与动态障碍物历史轨迹（除最新位置）的交叉
            for j = 1:num_dyn
                past_positions = dyn_history{j}(1:end-1, :);
                if ~isempty(past_positions)
                    if any(vecnorm(past_positions - new_pos, 2, 2) < collision_threshold)
                        intersection_points = [intersection_points; new_pos];
                    end
                end
            end
            
            % 更新路径、当前位置以及上一时刻的航向角
            path = [path; new_pos];
            current_pos = new_pos;
            prev_heading = new_heading;
        end  % end while
        
        % 保存本次实验的结果数据
        results(exp).path = path;
        results(exp).dyn_history = dyn_history;
        results(exp).dyn_obs = dyn_obs;
        results(exp).collision_flag = collision_flag;
        results(exp).collision_points = collision_points;
        results(exp).intersection_points = intersection_points;
        
        % 计算规划路径长度
        diff_path = diff(path);
        segment_lengths = sqrt(sum(diff_path.^2, 2));
        results(exp).path_length = sum(segment_lengths);
        path_lengths(exp) = results(exp).path_length;
        
        % 计算每次实验中的转向角（连续步之间）
        if size(path, 1) < 3
            results(exp).turning_angles = [];
            results(exp).max_turn_angle = 0;
            max_turn_angles(exp) = 0;
        else
            headings = atan2(diff(path(:,2)), diff(path(:,1)));
            turning_angles = abs(diff(headings));
            turning_angles = mod(turning_angles, pi);  % 保证在 [0, pi]
            results(exp).turning_angles = rad2deg(turning_angles);
            global_turning_angles = [global_turning_angles; results(exp).turning_angles];
            results(exp).max_turn_angle = max(results(exp).turning_angles);
            max_turn_angles(exp) = results(exp).max_turn_angle;
        end
        
        % 判断实验是否成功（到达目标且未发生碰撞）
        if norm(current_pos - goal) < 0.3 && ~collision_flag
            results(exp).success = true;
            success_flags(exp) = 1;
        else
            results(exp).success = false;
            success_flags(exp) = 0;
        end
        
        % 记录本次实验运行时间
        experiment_times(exp) = toc(t_exp);
        fprintf('Experiment %d/%d completed in %.2f seconds.\n', exp, total_experiments, experiment_times(exp));
    end  % end experiments loop
    
   %% 统计并输出 100 次实验结果（均值 ± 标准差）
valid_idx = (success_flags == 1);

success_rate = (sum(success_flags) / total_experiments) * 100;
success_rate_sd = std(success_flags) * 100;

if any(valid_idx)
    avg_path_length = mean(path_lengths(valid_idx));
    std_path_length = std(path_lengths(valid_idx));

    avg_max_turn_angle = mean(max_turn_angles(valid_idx));
    std_max_turn_angle = std(max_turn_angles(valid_idx));
else
    avg_path_length = NaN;
    std_path_length = NaN;
    avg_max_turn_angle = NaN;
    std_max_turn_angle = NaN;
end

avg_runtime = mean(experiment_times);
std_runtime = std(experiment_times);

fprintf('\n100次实验结果：\n');
fprintf('平均规划路径长度：%.2f ± %.2f m\n', avg_path_length, std_path_length);
fprintf('避障成功率：%.2f ± %.2f %%\n', success_rate, success_rate_sd);
fprintf('平均运行时间：%.4f ± %.4f s\n', avg_runtime, std_runtime);
fprintf('平均最大转向角：%.2f ± %.2f °\n', avg_max_turn_angle, std_max_turn_angle);

    
    % 输出所有实验中前10大的转向角
    if ~isempty(global_turning_angles)
        global_turning_angles = sort(global_turning_angles, 'descend');
        top10_turning_angles = global_turning_angles(1:min(10, length(global_turning_angles)));
        fprintf('前10大转向角（°）：\n');
        disp(top10_turning_angles');
    else
        fprintf('无转向角数据。\n');
    end
    
    %% 随机抽取两次实验结果进行最终可视化展示
    fprintf('显示随机抽取的两次实验结果图...\n');
    for idx = 1:length(selected_exp_indices)
        exp_index = selected_exp_indices(idx);
        figure;
        hold on; axis equal;
        xlim([0, n]); ylim([0, m]);
        title(sprintf('随机实验结果（实验 #%d）', exp_index));
        xlabel('X 轴'); ylabel('Y 轴');
        
        % 绘制静态障碍物
        for k = 1:size(obs,1)
            temp = obs(k,:);
            fill([temp(2)-1, temp(2), temp(2), temp(2)-1],...
                 [temp(1)-1, temp(1)-1, temp(1), temp(1)], 'k');
        end
        % 绘制目标点（红色方块）
        fill([goal(1)-0.5, goal(1)+0.5, goal(1)+0.5, goal(1)-0.5],...
             [goal(2)-0.5, goal(2)-0.5, goal(2)+0.5, goal(2)+0.5], 'r');
         
        % 绘制无人船规划路径（蓝色线条）
        path = results(exp_index).path;
        plot(path(:,1), path(:,2), 'b', 'LineWidth', 0.5);
        plot(path(end,1), path(end,2), 'bo', 'MarkerFaceColor', 'b');
        
        % 绘制动态障碍物的历史轨迹与最终位置（品红色虚线和实心圆）
        dyn_history = results(exp_index).dyn_history;
        for i = 1:length(dyn_history)
            h = dyn_history{i};
            plot(h(:,1), h(:,2), 'm--');
            plot(h(end,1), h(end,2), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
        end
        
        % 标记碰撞点（红色实心圆）
        collision_points = results(exp_index).collision_points;
        if ~isempty(collision_points)
            plot(collision_points(:,1), collision_points(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
        end
        
        % 标记历史轨迹交叉点（黑色空心圆）
        intersection_points = results(exp_index).intersection_points;
        if ~isempty(intersection_points)
            plot(intersection_points(:,1), intersection_points(:,2), 'ko', 'MarkerSize', 8);
        end
    end
    
    %% 显示最大转向角的实验结果图
    % 查找所有实验中最大转向角等于整体最大值的实验（可能有多个）
    max_value = max(max_turn_angles);
    indices_max = find(max_turn_angles == max_value);
    fprintf('显示最大转向角为 %.2f° 的实验结果图，共 %d 个实验。\n', max_value, length(indices_max));
    for idx = 1:length(indices_max)
        exp_index = indices_max(idx);
        figure;
        hold on; axis equal;
        xlim([0, n]); ylim([0, m]);
        title(sprintf('最大转向角实验结果（实验 #%d, 最大转向角: %.2f°）', exp_index, max_turn_angles(exp_index)));
        xlabel('X 轴'); ylabel('Y 轴');
        
        % 绘制静态障碍物
        for k = 1:size(obs,1)
            temp = obs(k,:);
            fill([temp(2)-1, temp(2), temp(2), temp(2)-1], ...
                 [temp(1)-1, temp(1)-1, temp(1), temp(1)], 'k');
        end
        % 绘制目标点（红色方块）
        fill([goal(1)-0.5, goal(1)+0.5, goal(1)+0.5, goal(1)-0.5], ...
             [goal(2)-0.5, goal(2)-0.5, goal(2)+0.5, goal(2)+0.5], 'r');
         
        % 绘制无人船规划路径（蓝色线条）
        path = results(exp_index).path;
        plot(path(:,1), path(:,2), 'b', 'LineWidth', 0.5);
        plot(path(end,1), path(end,2), 'bo', 'MarkerFaceColor', 'b');
        
        % 绘制动态障碍物的历史轨迹与最终位置（品红色虚线和实心圆）
        dyn_history = results(exp_index).dyn_history;
        for i = 1:length(dyn_history)
            h = dyn_history{i};
            plot(h(:,1), h(:,2), 'm--');
            plot(h(end,1), h(end,2), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
        end
        % 绘制碰撞点（红色实心圆）
        collision_points = results(exp_index).collision_points;
        if ~isempty(collision_points)
            plot(collision_points(:,1), collision_points(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
        end
        % 绘制历史轨迹交叉点（黑色空心圆）
        intersection_points = results(exp_index).intersection_points;
        if ~isempty(intersection_points)
            plot(intersection_points(:,1), intersection_points(:,2), 'ko', 'MarkerSize', 8);
        end
    end
end
