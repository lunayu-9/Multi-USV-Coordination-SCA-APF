function experimental_main()
    %% 实验参数设置
    num_runs = 100;
    num_dynamic_obs = 5;
    params = struct(...
        'Eta_att', 10,...
        'Eta_rep_ob', 100,...
        'd0', 10,...
        'len_step', 0.3,...
        'safe_distance', 1.5);
    
    %% 结果存储
    results = struct(...
        'path_length', zeros(num_runs,1),...
        'collision_flag', false(num_runs,1),...
        'max_angle', zeros(num_runs,1),...
        'path_data', cell(num_runs,1),...
        'obs_data', cell(num_runs,1));
    
    %% 静态障碍物初始化
    MAP_size = [40, 40];
    start_pos = [5, 5];
    goal = [35, 35];
    [MAP, static_obs] = create_static_map(MAP_size);
    
    %% 主实验循环
    for exp_id = 1:num_runs
        % 动态障碍物初始化（修复版本）
        dynamic_obs = initialize_dynamic_obstacles(num_dynamic_obs, MAP_size, start_pos, goal, static_obs);
        
        % 路径规划核心函数（保持原样）
        [path, max_angle, collision_points, dynamic_obs_history] = ...
            single_ship_planning(start_pos, goal, static_obs, dynamic_obs, MAP_size, params);
        
        % 结果记录
        results.path_length(exp_id) = calculate_path_length(path);
        results.collision_flag(exp_id) = ~isempty(collision_points);
        results.max_angle(exp_id) = max_angle;
        
        % 存储最后两次数据
        if exp_id > num_runs - 2
            results.path_data{exp_id} = path;
            results.obs_data{exp_id} = dynamic_obs_history;
        end
    end
    
    %% 结果分析
    success_rate = mean(~results.collision_flag)*100;
    avg_path_length = mean(results.path_length);
    overall_max_angle = max(results.max_angle);
    
    fprintf('===== 实验结果统计 =====\n');
    fprintf('平均路径长度: %.2f m\n', avg_path_length);
    fprintf('避障成功率: %.1f%%\n', success_rate);
    fprintf('最大转向角: %.2f°\n', overall_max_angle);
    
    %% 随机展示两次实验结果
    rng shuffle;
    sample_ids = randperm(num_runs, 2);
    for i = 1:2
        plot_experiment(results.path_data{sample_ids(i)},...
                       results.obs_data{sample_ids(i)},...
                       MAP_size, static_obs, start_pos, goal, params.safe_distance);
    end
end

%% 静态地图生成函数
function [MAP, static_obs] = create_static_map(MAP_size)
    m = MAP_size(1); n = MAP_size(2);
    MAP = ones(m, n);
    
    % 垂直墙体
    for i = 20:24
        MAP(i, 20) = 0;
        MAP(i, 24) = 0;
    end
    
    % 方块障碍物
    MAP(30:32, 28:30) = 0;
    MAP(18:20, 32:34) = 0;
    
    [obs_i, obs_j] = find(MAP == 0);
    static_obs = [obs_i, obs_j];
end

%% 动态障碍物初始化（修复关键错误）
function dynamic_obs = initialize_dynamic_obstacles(num_obs, MAP_size, start, goal, static_obs)
    % 预分配结构体数组
    dynamic_obs = repmat(struct('pos',[],'speed',[],'dir',[],'traj',[]), 1, num_obs);
    main_dir = (goal - start)/norm(goal - start);
    
    for i = 1:num_obs
        valid = false;
        attempts = 0;
        max_attempts = 100;
        
        while ~valid && attempts < max_attempts
            % 生成候选位置（确保为行向量）
            angle = rand*2*pi;
            pos = start + (8 + rand*12)*[cos(angle), sin(angle)];
            pos = pos(:)'; % 确保为1x2行向量
            
            % 有效性检查
            valid = all(pos > 5 & pos < (MAP_size-5)) && ...
                   min(vecnorm(static_obs - pos, 2, 2)) > 3 && ...
                   check_dynamic_obs_distance(pos, dynamic_obs(1:i-1), 3);
            
            attempts = attempts + 1;
        end
        
        if ~valid
            error('无法找到有效的障碍物初始位置');
        end
        
        % 设置运动参数（确保方向为行向量）
        dynamic_obs(i).pos = pos;
        dynamic_obs(i).speed = 0.4 + rand*0.8;
        dir_vec = main_dir + randn(1,2)*0.6;
        dynamic_obs(i).dir = dir_vec/norm(dir_vec);
        dynamic_obs(i).traj = pos;
    end
end

%% 距离检查函数（修复版本）
function valid = check_dynamic_obs_distance(pos, existing_obs, min_dist)
    valid = true;
    for j = 1:length(existing_obs)
        if isempty(existing_obs(j).pos)
            continue; % 跳过未初始化项
        end
        if norm(pos - existing_obs(j).pos) < min_dist
            valid = false;
            return;
        end
    end
end