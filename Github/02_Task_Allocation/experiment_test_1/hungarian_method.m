function [assignments, total_cost] = hungarian_method(ships, targets, ships_per_target)
    % 获取问题规模参数
    N = size(ships, 1);  % 无人船数量
    K = size(targets, 1);% 目标点数量
    
    % 验证参数合理性
    if K * ships_per_target ~= N
        error('总船只数N必须等于K × ships_per_target');
    end
    
    % 构建距离矩阵（N×K）
    cost_matrix = pdist2(ships, targets);
    
    % 扩展距离矩阵并添加噪声
    expanded_matrix = repelem(cost_matrix, 1, ships_per_target);
    noise = (rand(size(expanded_matrix)) - 0.5) * 1e-5; % [-0.5e-5, 0.5e-5]
    expanded_matrix = expanded_matrix + noise;
    
    % 执行匈牙利算法
    [assignment, ~] = matchpairs(expanded_matrix, 0, 'min');
    
    % 处理分配结果
    if size(assignment,1) < N
        warning('部分匹配（%d/%d），启动修复...', size(assignment,1), N);
        assignments = repair_assignment(assignment, N, K, ships_per_target);
    else
        assignments = zeros(N,1);
        for i = 1:N
            expanded_col = assignment(i,2);
            target_id = ceil(expanded_col / ships_per_target);
            assignments(i) = target_id;
        end
    end
    
    % 验证目标ID有效性
    invalid = assignments < 1 | assignments > K;
    if any(invalid)
        error('非法目标点ID: %s', mat2str(find(invalid)));
    end
    
    % 计算总代价
    linear_indices = sub2ind(size(cost_matrix), (1:N)', assignments);
    total_cost = sum(cost_matrix(linear_indices));
end

function assignments = repair_assignment(raw_assignment, N, K, ships_per_target)
    % 初始化分配结果
    assignments = zeros(N,1);
    
    % 记录已分配的列
    assigned_cols = raw_assignment(:,2);
    
    % 未分配的船只列表
    unassigned = setdiff(1:N, raw_assignment(:,1));
    
    % 生成可用目标列表（未达到容量限制）
    target_counts = zeros(K,1);
    for t = 1:K
        target_counts(t) = sum(ceil(assigned_cols / ships_per_target) == t);
    end
    available_targets = find(target_counts < ships_per_target);
    
    % 为未分配船只随机分配
    for ship = unassigned'
        if isempty(available_targets)
            error('无可用目标点，请检查容量限制');
        end
        target = available_targets(randi(length(available_targets)));
        assignments(ship) = target;
        target_counts(target) = target_counts(target) + 1;
        if target_counts(target) >= ships_per_target
            available_targets = available_targets(available_targets ~= target);
        end
    end
end
