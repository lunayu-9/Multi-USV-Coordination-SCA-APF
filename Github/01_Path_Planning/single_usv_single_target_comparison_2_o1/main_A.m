function experiment_100_runs_astar_improved()
    %% 实验参数设置
    total_experiments = 100;
    
    % 用于记录每次实验的统计数据
    path_lengths = zeros(total_experiments, 1);
    success_flags = zeros(total_experiments, 1);  % 1: 避障成功，0: 失败
    max_turn_angles = zeros(total_experiments, 1);
    experiment_times = zeros(total_experiments, 1);  % 每次实验运行时间（秒）
    
    results = struct();
    global_turning_angles = [];
    
    %% 地图与目标设置
    MAP_size = [40, 40];  % 行数×列数
    m = MAP_size(1); n = MAP_size(2);
    start = [5, 5];       % 起点（行,列）
    goal  = [35, 35];     % 目标（行,列）
    
    % 构造初始静态障碍物地图：1 表示可通行，0 表示障碍
    MAP = ones(m, n);
    for i = 20:24
        MAP(i, 20) = 0;
        MAP(i, 24) = 0;
    end
    MAP(30:32, 28:30) = 0;
    MAP(18:20, 32:34) = 0;
    
    % 保存原始静态障碍物位置用于可视化
    [obs_i, obs_j] = find(MAP == 0);
    obs = [obs_i, obs_j];
    
    %% 安全距离设置：对静态障碍物进行膨胀，避免规划路径紧贴障碍物
    safety_margin = 1;  % 安全距离，单位：网格（可根据需要调整）
    % 使用 imdilate 膨胀障碍物（需要 Image Processing Toolbox）
    se = strel('disk', safety_margin, 0);
    obstacles_binary = 1 - MAP;       % 原障碍物标记为 1
    obstacles_inflated = imdilate(obstacles_binary, se);
    MAP_inflated = 1 - obstacles_inflated;  % 膨胀后：1 表示可通行，0 表示障碍
    
    %% 实验参数
    collision_threshold = 1;  % 动态障碍物碰撞判断阈值（单位：网格）
    max_iter = 1000;          % 最大迭代次数
    
    % 动态障碍物参数（设置 1 个动态障碍物）
    num_dyn = 3;            
    dyn_speed = 0.2;        % 动态障碍物移动速度（单位：每步移动距离）
    
    %% 100 次实验循环
    for exp = 1:total_experiments
        t_exp = tic;
        
        % 初始化机器人状态（以网格坐标表示，初值为整数）
        current_cell = start;
        path_record = start;
        
        % 初始化动态障碍物（连续坐标），在起点与目标之间随机布置
        dyn_obs = [10 + rand(num_dyn,1)*(goal(1)-10), 10 + rand(num_dyn,1)*(goal(2)-10)];
        dyn_vel = zeros(num_dyn, 2);
        for i = 1:num_dyn
            theta = rand()*2*pi;
            dyn_vel(i,:) = dyn_speed * [cos(theta), sin(theta)];
        end
        % 记录动态障碍物历史轨迹
        dyn_history = cell(num_dyn,1);
        for i = 1:num_dyn
            dyn_history{i} = dyn_obs(i,:);
        end
        
        collision_flag = false;
        iter = 0;
        % 初始朝向为起点到目标的方向（弧度）
        prev_heading = atan2(goal(2)-current_cell(2), goal(1)-current_cell(1));
        turning_angles_experiment = [];
        
        % 模拟机器人运动：直到到达目标（距离 < 0.5 个网格）或超出最大迭代次数
        while norm(current_cell - goal) > 0.5 && iter < max_iter
            iter = iter + 1;
            
            %% 更新动态障碍物位置
            for i = 1:num_dyn
                dyn_obs(i,:) = dyn_obs(i,:) + dyn_vel(i,:);
                dyn_history{i} = [dyn_history{i}; dyn_obs(i,:)];
                % 边界检测：超出地图边界则反向
                if dyn_obs(i,1) < 1 || dyn_obs(i,1) > n
                    dyn_vel(i,1) = -dyn_vel(i,1);
                end
                if dyn_obs(i,2) < 1 || dyn_obs(i,2) > m
                    dyn_vel(i,2) = -dyn_vel(i,2);
                end
            end
            
            %% 构建 A* 使用的地图：基于膨胀后的静态障碍物地图，并添加动态障碍物
            grid_astar = MAP_inflated;
            for i = 1:num_dyn
                cell_dyn = round(dyn_obs(i,:));
                if cell_dyn(1) >= 1 && cell_dyn(1) <= m && cell_dyn(2) >= 1 && cell_dyn(2) <= n
                    grid_astar(cell_dyn(1), cell_dyn(2)) = 0;
                end
            end
            
            %% 使用 A* 算法规划从当前网格到目标的路径
            [planned_path, found] = astar(grid_astar, current_cell, goal);
            if ~found
                % 未找到可行路径，认为实验失败
                collision_flag = true;
                break;
            end
            
            if size(planned_path,1) < 2
                break;
            end
            
            % 前进一步：取规划路径的第二个网格
            next_cell = planned_path(2,:);
            
            %% 计算转向角（用于统计）
            new_heading = atan2(next_cell(2)-current_cell(2), next_cell(1)-current_cell(1));
            delta_heading = new_heading - prev_heading;
            delta_heading = mod(delta_heading+pi, 2*pi) - pi;
            turning_angles_experiment = [turning_angles_experiment; abs(rad2deg(delta_heading))];
            
            %% 更新机器人位置
            current_cell = next_cell;
            path_record = [path_record; current_cell];
            prev_heading = new_heading;
            
            %% 检测与动态障碍物碰撞（连续坐标比较，阈值 collision_threshold）
            for i = 1:num_dyn
                if norm(dyn_obs(i,:) - current_cell) < collision_threshold
                    collision_flag = true;
                    break;
                end
            end
            if collision_flag
                break;
            end
        end  % end while
        
        %% 保存本次实验数据
        results(exp).path = path_record;
        results(exp).dyn_history = dyn_history;
        results(exp).dyn_obs = dyn_obs;
        results(exp).collision_flag = collision_flag;
        results(exp).turning_angles = turning_angles_experiment;
        if isempty(turning_angles_experiment)
            results(exp).max_turn_angle = 0;
            max_turn_angles(exp) = 0;
        else
            results(exp).max_turn_angle = max(turning_angles_experiment);
            max_turn_angles(exp) = results(exp).max_turn_angle;
            global_turning_angles = [global_turning_angles; turning_angles_experiment];
        end
        
        diff_path = diff(path_record);
        segment_lengths = sqrt(sum(diff_path.^2, 2));
        results(exp).path_length = sum(segment_lengths);
        path_lengths(exp) = results(exp).path_length;
        
        if norm(current_cell - goal) <= 0.5 && ~collision_flag
            results(exp).success = true;
            success_flags(exp) = 1;
        else
            results(exp).success = false;
            success_flags(exp) = 0;
        end
        
        experiment_times(exp) = toc(t_exp);
        fprintf('Experiment %d/%d completed in %.2f seconds.\n', exp, total_experiments, experiment_times(exp));
    end  % end for experiments
    
%% 输出统计结果
successful_path_lengths = path_lengths(success_flags == 1); % 仅取成功实验的路径长度
if ~isempty(successful_path_lengths)
    avg_path_length = mean(successful_path_lengths);
else
    avg_path_length = 0;
end
success_rate = (sum(success_flags)/total_experiments)*100;
overall_max_turn_angle = max(max_turn_angles);
avg_runtime = mean(experiment_times);
fprintf('\n100次实验结果（仅统计成功到达目标的实验）：\n');
fprintf('平均规划路径长度：%.2f m\n', avg_path_length);
fprintf('避障成功率：%.2f %%\n', success_rate);
fprintf('最大转向角：%.2f °\n', overall_max_turn_angle);
fprintf('平均运行时间：%.2f s\n', avg_runtime);

    
    %% 随机选取2个避障成功的实验结果进行可视化
    successful_indices = find(success_flags == 1);
    if length(successful_indices) >= 2
        rand_success = successful_indices(randperm(length(successful_indices), 2));
    elseif ~isempty(successful_indices)
        rand_success = successful_indices;
    else
        rand_success = [];
    end
    
    for idx = 1:length(rand_success)
        exp_index = rand_success(idx);
        figure;
        hold on; axis equal;
        xlim([0, n+1]); ylim([0, m+1]);
        title(sprintf('实验 #%d - 避障成功', exp_index));
        xlabel('X 轴'); ylabel('Y 轴');
        
        % 绘制膨胀前的原始静态障碍物（用于展示障碍位置）
        for k = 1:size(obs,1)
            temp = obs(k,:);
            fill([temp(2)-1, temp(2), temp(2), temp(2)-1],...
                 [temp(1)-1, temp(1)-1, temp(1), temp(1)], 'k');
        end
        % 绘制目标点（红色方块）
        fill([goal(2)-0.5, goal(2)+0.5, goal(2)+0.5, goal(2)-0.5],...
             [goal(1)-0.5, goal(1)-0.5, goal(1)+0.5, goal(1)+0.5], 'r');
         
        % 绘制机器人规划路径（蓝色线条）
        path_plot = results(exp_index).path;
        x_path = path_plot(:,2);
        y_path = path_plot(:,1);
        plot(x_path, y_path, 'b-', 'LineWidth', 1);
        plot(x_path(end), y_path(end), 'bo', 'MarkerFaceColor', 'b');
        
        % 绘制动态障碍物轨迹与最终位置（品红色虚线和圆点）
        dyn_history = results(exp_index).dyn_history;
        for i = 1:num_dyn
            h = dyn_history{i};
            plot(h(:,2), h(:,1), 'm--');
            plot(h(end,2), h(end,1), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
        end
    end
    
    %% 随机选取1个避障失败（collision）的实验结果进行可视化
    failure_indices = find(success_flags == 0);
    if ~isempty(failure_indices)
        rand_failure = failure_indices(randi(length(failure_indices)));
        figure;
        hold on; axis equal;
        xlim([0, n+1]); ylim([0, m+1]);
        title(sprintf('实验 #%d - 避障失败', rand_failure));
        xlabel('X 轴'); ylabel('Y 轴');
        
        for k = 1:size(obs,1)
            temp = obs(k,:);
            fill([temp(2)-1, temp(2), temp(2), temp(2)-1],...
                 [temp(1)-1, temp(1)-1, temp(1), temp(1)], 'k');
        end
        fill([goal(2)-0.5, goal(2)+0.5, goal(2)+0.5, goal(2)-0.5],...
             [goal(1)-0.5, goal(1)-0.5, goal(1)+0.5, goal(1)+0.5], 'r');
         
        path_plot = results(rand_failure).path;
        x_path = path_plot(:,2);
        y_path = path_plot(:,1);
        plot(x_path, y_path, 'b-', 'LineWidth', 1);
        plot(x_path(end), y_path(end), 'bo', 'MarkerFaceColor', 'b');
        
        dyn_history = results(rand_failure).dyn_history;
        for i = 1:num_dyn
            h = dyn_history{i};
            plot(h(:,2), h(:,1), 'm--');
            plot(h(end,2), h(end,1), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
        end
        % 标记碰撞点（红色圆点）
        plot(x_path(end), y_path(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    else
        fprintf('无避障失败的实验结果。\n');
    end
end

%% A* 算法子函数
function [path, found] = astar(grid, start, goal)
    [m, n] = size(grid);
    start_node = sub2ind([m, n], start(1), start(2));
    goal_node = sub2ind([m, n], goal(1), goal(2));
    
    g = inf(m*n, 1);
    f = inf(m*n, 1);
    g(start_node) = 0;
    [start_r, start_c] = ind2sub([m, n], start_node);
    [goal_r, goal_c] = ind2sub([m, n], goal_node);
    h = sqrt((start_r - goal_r)^2 + (start_c - goal_c)^2);
    f(start_node) = h;
    
    parent = zeros(m*n, 1);
    
    open_set = false(m*n, 1);
    open_set(start_node) = true;
    closed_set = false(m*n, 1);
    
    while any(open_set)
        open_indices = find(open_set);
        [~, min_idx] = min(f(open_indices));
        current = open_indices(min_idx);
        
        if current == goal_node
            found = true;
            path_ind = current;
            path = [];
            while path_ind ~= 0
                [r, c] = ind2sub([m, n], path_ind);
                path = [[r, c]; path];
                path_ind = parent(path_ind);
            end
            return;
        end
        
        open_set(current) = false;
        closed_set(current) = true;
        
        [cur_r, cur_c] = ind2sub([m, n], current);
        for dr = -1:1
            for dc = -1:1
                if dr==0 && dc==0
                    continue;
                end
                nb_r = cur_r + dr;
                nb_c = cur_c + dc;
                if nb_r < 1 || nb_r > m || nb_c < 1 || nb_c > n
                    continue;
                end
                neighbor = sub2ind([m, n], nb_r, nb_c);
                if grid(nb_r, nb_c) == 0
                    continue;
                end
                if closed_set(neighbor)
                    continue;
                end
                if abs(dr)==1 && abs(dc)==1
                    move_cost = sqrt(2);
                else
                    move_cost = 1;
                end
                tentative_g = g(current) + move_cost;
                if ~open_set(neighbor)
                    open_set(neighbor) = true;
                elseif tentative_g >= g(neighbor)
                    continue;
                end
                parent(neighbor) = current;
                g(neighbor) = tentative_g;
                heuristic = sqrt((nb_r - goal_r)^2 + (nb_c - goal_c)^2);
                f(neighbor) = g(neighbor) + heuristic;
            end
        end
    end
    found = false;
    path = [];
end
