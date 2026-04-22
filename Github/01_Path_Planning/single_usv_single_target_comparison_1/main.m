function main()
    %% 参数与地图设置
    MAP_size = [40, 40]; % 地图尺寸
    m = MAP_size(1); n = MAP_size(2); 
    
    % 目标点位置
    goal = [35, 35];
    
    % 静态障碍物设置（参考给定文件程序）
    MAP = ones(m, n);
    MAP(20:24, 20) = 0; 
    MAP(20:24, 24) = 0;
    MAP(30:32, 28:30) = 0;
    MAP(18:20, 32:34) = 0;
    [obs_i, obs_j] = find(MAP == 0);
    obs = [obs_i, obs_j];  % 静态障碍物位置
    
    %% 初始动态障碍物设置
    dynamicObs1 = [10, 5]; % 初始位置
    dynamicObs2 = [15, 8]; % 初始位置
    dynamicSpeed = 0.5;    % 运动速度
    
    %% 无人船初始位置（左下角）
    pos = [1, 1];  
    
    %% 实验次数
    num_experiments = 100;
    path_lengths = zeros(num_experiments, 1);
    success_flags = zeros(num_experiments, 1);
    max_turn_angles = zeros(num_experiments, 1);
    comp_times = zeros(num_experiments, 1);
    
    % 为了展示动态避障过程，保存最后一次规划结果
    final_boatPath = [];
    final_dynTraj1 = [];
    final_dynTraj2 = [];
    
    %% 开始实验
    for exp_idx = 1:num_experiments
        t_start = tic;
        % 调用改进的人工势场法（注意传入初始动态障碍物位置和速度）
        [boatPath, success, dynTraj1, dynTraj2] = improvedArtificialPotentialField(pos, goal, obs, dynamicObs1, dynamicObs2, MAP_size, dynamicSpeed);
        
        comp_times(exp_idx) = toc(t_start);          % 计算时间
        path_lengths(exp_idx) = sum(sqrt(sum(diff(boatPath).^2, 2)));  % 路径长度
        max_turn_angles(exp_idx) = compute_max_angle(boatPath);         % 最大转向角
        
       if success
    success_flags(exp_idx) = 1;
else
    fprintf('第 %d 次失败\n', exp_idx);
end
        
        % 保存最后一次成功的避障过程
        if success && exp_idx == num_experiments
            final_boatPath = boatPath;
            final_dynTraj1 = dynTraj1;
            final_dynTraj2 = dynTraj2;
        end
    end

   %% 计算并输出评价指标（均值 ± 标准差）
success_rate = (sum(success_flags) / num_experiments) * 100;
success_rate_sd = std(success_flags) * 100;

valid_idx = (success_flags == 1);

avg_path_length = mean(path_lengths(valid_idx));
std_path_length = std(path_lengths(valid_idx));

avg_max_turn_angle = mean(max_turn_angles(valid_idx));
std_max_turn_angle = std(max_turn_angles(valid_idx));

avg_comp_time = mean(comp_times);
std_comp_time = std(comp_times);

fprintf('成功率: %.2f ± %.2f %%\n', success_rate, success_rate_sd);
fprintf('平均路径长度: %.2f ± %.2f 米\n', avg_path_length, std_path_length);
fprintf('平均最大转向角度: %.2f ± %.2f 度\n', avg_max_turn_angle, std_max_turn_angle);
fprintf('平均计算时间: %.4f ± %.4f 秒\n', avg_comp_time, std_comp_time);
    
    %% 绘制最后一次成功的动态避障过程
    if ~isempty(final_boatPath)
        plot_dynamic_obstacle_avoidance(final_boatPath, final_dynTraj1, final_dynTraj2, MAP, goal);
    end
end
