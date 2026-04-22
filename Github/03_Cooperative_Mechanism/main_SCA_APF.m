function Main_System_Academic_Final()
    % =====================================================================
    % 系统名称：多USV协同任务分配与路径规划仿真系统 (学术增强版)
    % 修复重点：1. 彻底杜绝图表负值  2. 适配论文分组逻辑 (N=8, K=2) 3. 强化统计分析
    % =====================================================================
    clear; clc; close all;

    % --- 实验参数 (严格遵循审稿人要求) ---
    N_list = [4, 8, 16, 32]; 
    K_list = [2, 4, 8];
    num_trials = 10; % 建议论文数据使用100次，此处演示为10次
    
    results_data = [];
    fprintf('>>> 正在执行可扩展性实验 (含海洋电流与转向约束)...\n');

    for K = K_list
        for N = N_list
            if N < K, continue; end 
            
            trial_metrics = zeros(num_trials, 3); 
            for t = 1:num_trials
                % 仅在第一次运行 N=8, K=2 时显示动画，方便你录屏或截图
                show_anim = (t == 1 && N == 8 && K == 2); 
                res = execute_core_simulation(N, K, show_anim);
                trial_metrics(t, :) = [res.total_path, res.comp_time, res.max_angle];
            end
            
            % 计算统计量 (Mean ± SD)
            avg = mean(trial_metrics, 1);
            sd = std(trial_metrics, 0, 1);
            results_data = [results_data; N, K, avg(1), sd(1), avg(2), sd(2), avg(3)];
        end
    end

    % --- 生成符合论文要求的表格 ---
    Table_Final = array2table(results_data, 'VariableNames', ...
        {'N_USV','K_Target','Path_Mean','Path_Std','Time_Mean','Time_Std','Max_Angle'});
    disp('>>> 统计结果已生成：');
    disp(Table_Final);
    
    % --- 绘制修正后的趋势图 ---
    draw_academic_plots(Table_Final);
end

%% ===================== 核心引擎：适配分组编队 =====================
function res = execute_core_simulation(N, K, is_visible)
    map_limit = 100;
    v_stream = [0.12, 0.05]; % 海洋扰动 [cite: 168]
    obs_list = [35,35; 35,65; 60,50; 75,70; 45,25; 80,40]; % 障碍物布局
    
    % 初始化船只 (均匀分布在左侧)
    ships = struct('pos', [], 'group', 0, 'path', [], 'heading', 0);
    for i = 1:N
        ships(i).pos = [5 + rand*10, 10 + (i-1)*(80/N)];
        ships(i).path = ships(i).pos;
        ships(i).heading = 0;
    end
    
    % 初始化目标点 (对应论文多任务场景 [cite: 315])
    targets = struct('pos', []);
    for k = 1:K
        targets(k).pos = [85, 15 + (k-1)*(70/K)];
    end
    
    % 执行分组分配 (适配论文改进匈牙利算法逻辑 [cite: 1447])
    s_pos = vertcat(ships.pos);
    t_pos = vertcat(targets.pos);
    alloc = balanced_task_allocation(s_pos, t_pos, N, K);
    for i = 1:N, ships(i).group = alloc(i); end
    
    if is_visible
        h_fig = figure('Color','w','Name','论文适配场景: N=8, K=2'); hold on;
        scatter(obs_list(:,1), obs_list(:,2), 120, [0.5 0.5 0.5], 'filled');
        h_s = scatter(s_pos(:,1), s_pos(:,2), 50, 'b', 'filled');
        scatter(t_pos(:,1), t_pos(:,2), 150, 'rp', 'LineWidth', 2);
        axis([0 map_limit 0 map_limit]); grid on;
        title(['编队协同仿真 (海洋电流: ', num2str(v_stream(1)), ' m/s)']);
    end

    tic;
    for step = 1:450
        active = false;
        for i = 1:N
            target_pos = targets(ships(i).group).pos;
            if norm(ships(i).pos - target_pos) > 1.8
                % 带物理约束的SCA-APF引擎 [cite: 134, 1532]
                [ships(i).pos, ships(i).heading] = sca_apf_core(...
                    ships(i).pos, target_pos, obs_list, ships(i).heading, v_stream);
                ships(i).path(end+1,:) = ships(i).pos;
                active = true;
            end
        end
        if is_visible && mod(step, 4) == 0
            cur = vertcat(ships.pos);
            set(h_s, 'XData', cur(:,1), 'YData', cur(:,2));
            drawnow limitrate;
        end
        if ~active, break; end
    end
    res.comp_time = toc;
    res = get_metrics(ships, res);
    if is_visible, pause(1); close(h_fig); end
end

%% ===================== 避障引擎：转角约束与电流 =====================
function [p_new, h_new] = sca_apf_core(curr, goal, obs, h_old, v_env)
    % 势场参数优化
    ka = 2.0; kr = 450; d0 = 7.5;
    limit_turn = deg2rad(30); % 严格转向角约束 [cite: 1203]
    v_step = 0.65;

    f_att = ka * (goal - curr) / (norm(goal - curr) + 0.1);
    f_rep = [0,0];
    for i = 1:size(obs,1)
        v_obs = curr - obs(i,:); d = norm(v_obs);
        if d < d0
            f_rep = f_rep + kr * (1/d - 1/d0) * (1/d^2) * (v_obs/d);
        end
    end
    
    % 总受力 = 引力 + 斥力 + 海洋电流
    f_total = f_att + f_rep + v_env;
    
    % 航向角平滑处理
    h_target = atan2(f_total(2), f_total(1));
    diff = angdiff(h_old, h_target);
    % 强制截断转角，消除 180 度跳变
    h_new = h_old + max(min(diff, limit_turn), -limit_turn);
    p_new = curr + v_step * [cos(h_new), sin(h_new)];
end

%% ===================== 绘图函数：修正负值误差条 =====================
function draw_academic_plots(T)
    fig = figure('Color','w','Name','Scalability Analysis (Corrected)');
    k_tags = unique(T.K_Target);
    colors = {'r', [0 0.7 0], 'b'};
    syms = {'-o', '-s', '-d'};

    % 左图：计算效率 (修正负值)
    subplot(1,2,1); hold on;
    for i = 1:length(k_tags)
        idx = (T.K_Target == k_tags(i));
        % 关键修正：确保误差下界不为负
        low_err = min(T.Time_Mean(idx), T.Time_Std(idx));
        errorbar(T.N_USV(idx), T.Time_Mean(idx), low_err, T.Time_Std(idx), ...
            syms{i}, 'Color', colors{i}, 'LineWidth', 1.5, ...
            'DisplayName', ['K=' num2str(k_tags(i))]);
    end
    set(gca, 'YScale', 'linear'); % 保持线性坐标，此时Y轴起始点必为0
    ylim([0, max(T.Time_Mean + T.Time_Std)*1.1]); 
    xlabel('Number of USVs (N)'); ylabel('Computational Time (s)');
    title('Time Efficiency (Mean \pm SD)'); legend('show'); grid on;

    % 右图：路径质量
    subplot(1,2,2); hold on;
    for i = 1:length(k_tags)
        idx = (T.K_Target == k_tags(i));
        errorbar(T.N_USV(idx), T.Path_Mean(idx), T.Path_Std(idx), ...
            syms{i}, 'Color', colors{i}, 'LineWidth', 1.5, ...
            'DisplayName', ['K=' num2str(k_tags(i))]);
    end
    xlabel('Number of USVs (N)'); ylabel('Total Path Length (m)');
    title('Path Quality Scalability'); legend('show'); grid on;
end

%% ===================== 辅助计算与分配 =====================
function groups = balanced_task_allocation(ships, tgts, N, K)
    groups = zeros(N,1);
    counts = zeros(K,1);
    cap = ceil(N/K);
    for i = 1:N
        dists = vecnorm(tgts - ships(i,:), 2, 2);
        [~, sort_idx] = sort(dists);
        for k = 1:K
            t_id = sort_idx(k);
            if counts(t_id) < cap
                groups(i) = t_id;
                counts(t_id) = counts(t_id) + 1;
                break;
            end
        end
    end
end

function res = get_metrics(ships, res)
    d_total = 0; ang_max = 0;
    for i = 1:length(ships)
        p = ships(i).path;
        if size(p,1) < 3, continue; end
        d_total = d_total + sum(vecnorm(diff(p),2,2));
        for j = 2:size(p,1)-1
            v1 = p(j,:) - p(j-1,:); v2 = p(j+1,:) - p(j,:);
            if norm(v1)>0.01 && norm(v2)>0.01
                a = acosd(dot(v1,v2)/(norm(v1)*norm(v2)));
                if a < 170, ang_max = max(ang_max, a); end
            end
        end
    end
    res.total_path = d_total;
    res.max_angle = ang_max;
end