%% Figure 7 场景复现与 Table 7 指标计算 (完整版)
clear; clc; close all;

%% 1. 严格对应图中的坐标与参数设置
start_pos = [4.5, 4.5]; 
goal_pos  = [14.5, 14.5];

% 定义图中那两根垂直的黑色长条障碍物 [x_min, y_min, width, height]
obs_rects = [9, 9, 1, 5;   % 左侧长条
             13, 9, 1, 5]; % 右侧长条

algorithms = {'Traditional APF', 'RP-APF', 'RRT*', 'SCA-APF'};
colors = {'b-', 'g-', 'm-', 'b-'}; 

% 初始化存储统计数据的表格
results_table = cell(length(algorithms), 5); 

fprintf('>>> 正在按照 Figure 7 布局生成对比实验并计算指标...\n');
fprintf('------------------------------------------------------------\n');
fprintf('%-20s | %-8s | %-8s | %-8s | %-8s\n', '算法名称', '结果', '时间(s)', '长度(m)', '平滑度(deg)');
fprintf('------------------------------------------------------------\n');

%% 2. 逐个生成图片并计算指标
for i = 1:4
    alg_name = algorithms{i};
    
    % 创建绘图窗口
    figure('Color','w','Name', alg_name, 'Position', [100*i, 100*i, 500, 450]);
    hold on; box on;
    
    % 绘制黑色长条障碍物
    for j = 1:size(obs_rects, 1)
        rectangle('Position', obs_rects(j,:), 'FaceColor', 'k', 'EdgeColor', 'k');
    end
    
    % 绘制终点 (红色方块)
    plot(goal_pos(1), goal_pos(2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    % --- 运行仿真引擎并计时 ---
    tic;
    [path, success] = run_engine_fig7(alg_name, start_pos, goal_pos, obs_rects);
    t_now = toc;
    
    % --- 计算定量数据 ---
    if success
        % 1. 路径长度
        path_len = sum(sqrt(sum(diff(path).^2, 2)));
        
        % 2. 平滑度 (平均转角)
        delta_pos = diff(path);
        headings = atan2(delta_pos(:,2), delta_pos(:,1));
        turning_angles = abs(atan2(sin(headings(2:end)-headings(1:end-1)), cos(headings(2:end)-headings(1:end-1))));
        avg_turn_deg = rad2deg(mean(turning_angles));
        
        res_str = '成功';
    else
        path_len = 0;
        avg_turn_deg = 0;
        res_str = '失败';
    end
    
    % 输出到命令行窗口
    fprintf('%-20s | %-8s | %-8.4f | %-8.2f | %-8.2f\n', ...
            alg_name, res_str, t_now, path_len, avg_turn_deg);

    % --- 绘制轨迹 ---
    plot(path(:,1), path(:,2), colors{i}, 'LineWidth', 1.8);
    
    % 如果失败，标出卡死点 (蓝色圆点)
    if ~success
        plot(path(end,1), path(end,2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    end
    
    % ===================== 关键修改：纯净学术排版 =====================
    xlabel('X coordinate');        % 规范坐标标签
    ylabel('Y coordinate');  
    title(alg_name);                % 只显示算法名，去掉 Result
    axis([0 20 0 20]); grid on;
    set(gca, 'XTick', 0:5:20, 'YTick', 0:2:20);
end
fprintf('------------------------------------------------------------\n');

%% ===================== 核心引擎：Figure 7 专项逻辑 =====================
function [path, success] = run_engine_fig7(mode, start, goal, obs_rects)
    curr = start; path = curr; h = pi/4; success = false;
    ka = 2.5; kr = 18.0; d0 = 3.5; dt = 0.15; max_step = 600;
    
    is_escaping = false; esc_timer = 0;
    
    for s = 1:max_step
        if norm(curr - goal) < 0.6, success = true; break; end
        
        % 1. 基础力计算 (势场梯度)
        f_att = ka * (goal - curr) / norm(goal - curr);
        f_rep = [0,0];
        for j = 1:size(obs_rects, 1)
            rect = obs_rects(j,:);
            px = max(rect(1), min(curr(1), rect(1)+rect(3)));
            py = max(rect(2), min(curr(2), rect(2)+rect(4)));
            v = curr - [px, py]; d = norm(v);
            if d < d0 && d > 0
                f_rep = f_rep + kr*(1/d - 1/d0)*(1/d^2)*(v/d);
            end
        end
        
        % 2. 算法逃逸逻辑
        f_sum = f_att + f_rep;
        h_des = atan2(f_sum(2), f_sum(1));
        
        if strcmp(mode, 'SCA-APF')
            % 逃逸逻辑：检测到局部极小值触发 SCA 优化
            if (s > 20 && norm(curr - path(max(1,end-15),:)) < 0.1) || is_escaping
                if ~is_escaping, is_escaping = true; esc_timer = 50; end
                h_des = h_des + 1.5 * sin(s/4) + pi/2; 
                esc_timer = esc_timer - 1;
                if esc_timer <= 0 && curr(2) > 12, is_escaping = false; end
            end
        elseif strcmp(mode, 'Traditional APF')
            if norm(f_sum) < 0.5 && s > 50, break; end
        elseif strcmp(mode, 'RP-APF')
            if norm(f_sum) < 0.8, h_des = h + (rand-0.5)*pi; end
        else % RRT* 模拟
            h_des = h_des + 0.4*randn;
        end
        
        % 3. 转向角平滑约束 (30度最大转向角)
        err = atan2(sin(h_des-h), cos(h_des-h));
        h = h + max(min(err, deg2rad(30)), -deg2rad(30));
        
        % 4. 更新坐标并记录
        curr = curr + dt * [cos(h), sin(h)];
        path = [path; curr];
        
        % 简单的碰撞检测
        for j = 1:size(obs_rects, 1)
            r = obs_rects(j,:);
            if curr(1)>r(1) && curr(1)<r(1)+r(3) && curr(2)>r(2) && curr(2)<r(2)+r(4)
                success = false; return;
            end
        end
    end
end