function [assignments, total_cost, cluster_cost] = improved_hungarian(ships, targets, ships_per_target)
       % 获取无人船数量
    N = size(ships, 1); % 新增
% 参数设置
    max_iter = 200;    % 最大迭代次数
    temp_init = 100;   % 初始温度（模拟退火）
    cooling_rate = 0.95;

    % 步骤1: 计算倾向性并预分组
    [~, sorted_idx] = sort(pdist2(ships, targets), 2);
    preferences = sorted_idx(:,1)./sorted_idx(:,2); % 倾向性指标
    [~, pre_groups] = kmeans(ships, K, 'Start', targets); % 基于位置聚类
    
    % 步骤2: 初始分配确保容量约束
    assignments = zeros(N, 1); % 使用本地N
    assignments = zeros(N,1);
    for k = 1:K
        candidates = find(pre_groups == k);
        if length(candidates) >= ships_per_target
            selected = randperm(length(candidates), ships_per_target);
            assignments(candidates(selected)) = k;
        end
    end
    
    % 步骤3: 局部交换优化
    temp = temp_init;
    best_cost = calculate_total_cost(assignments, ships, targets);
    for iter = 1:max_iter
        % 随机选择两个不同组的无人船
        valid_pairs = find(assignments ~= 0);
        ship1 = valid_pairs(randi(length(valid_pairs)));
        ship2 = valid_pairs(randi(length(valid_pairs)));
        while assignments(ship1) == assignments(ship2)
            ship2 = valid_pairs(randi(length(valid_pairs)));
        end
        
        % 尝试交换分组
        new_assignments = assignments;
        new_assignments([ship1, ship2]) = new_assignments([ship2, ship1]);
        
        % 计算代价变化
        new_cost = calculate_total_cost(new_assignments, ships, targets);
        delta = new_cost - best_cost;
        
        % 模拟退火接受准则
        if delta < 0 || exp(-delta/temp) > rand()
            assignments = new_assignments;
            best_cost = new_cost;
        end
        
        % 降温
        temp = temp * cooling_rate;
    end
    
    % 计算最终代价
    [total_cost, cluster_cost] = calculate_total_cost(assignments, ships, targets);
end

function [total_cost, cluster_cost] = calculate_total_cost(assignments, ships, targets)
    dist_cost = 0;
    cluster_cost = 0;
    for k = 1:size(targets,1)
        members = ships(assignments == k, :);
        if ~isempty(members)
            dist_cost = dist_cost + sum(vecnorm(members - targets(k,:), 2, 2));
            cluster_cost = cluster_cost + sum(pdist(members));
        end
    end
    total_cost = dist_cost + 0.5 * cluster_cost; % 权重系数α=0.5
end
