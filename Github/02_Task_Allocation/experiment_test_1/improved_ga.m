%遗传算法
function [best_assign, best_cost, cluster_cost] = improved_ga(ships, targets, ships_per_target)
    % 参数设置
    pop_size = 50;      % 种群大小
    max_gen = 200;      % 最大迭代次数
    crossover_rate = 0.8;
    mutation_rate = 0.1;
    elite_ratio = 0.1;  % 精英保留比例
    
    % 初始化种群
    [n_ships, ~] = size(ships);
    n_targets = size(targets,1);
    population = init_population(pop_size, n_ships, n_targets, ships_per_target);
    
    % 进化循环
    best_cost = inf;
    for gen = 1:max_gen
        % 计算适应度
        fitness = zeros(pop_size,1);
        for i = 1:pop_size
            [fitness(i), ~] = evaluate_individual(population(i,:), ships, targets);
        end
        
        % 精英保留
        [~, elite_idx] = mink(fitness, round(elite_ratio*pop_size));
        new_pop = population(elite_idx,:);
        
        % 选择-交叉-变异
        while size(new_pop,1) < pop_size
            % 锦标赛选择
            parent1 = tournament_select(fitness, 3);
            parent2 = tournament_select(fitness, 3);
            
            % 交叉
            if rand() < crossover_rate
                [child1, child2] = crossover(population(parent1,:), population(parent2,:), ships_per_target);
            else
                child1 = population(parent1,:);
                child2 = population(parent2,:);
            end
            
            % 变异
            child1 = mutate(child1, n_targets, mutation_rate, ships_per_target);
            child2 = mutate(child2, n_targets, mutation_rate, ships_per_target);
            
            new_pop = [new_pop; child1; child2];
        end
        
        % 截断至种群大小
        population = new_pop(1:pop_size,:);
        
        % 更新最优解
        [min_fit, idx] = min(fitness);
        if min_fit < best_cost
            best_cost = min_fit;
            best_assign = population(idx,:);
        end
    end
    
    % 计算最终代价
    [~, cluster_cost] = evaluate_individual(best_assign, ships, targets);
end

%% 初始化种群（满足容量约束）
function pop = init_population(pop_size, n_ships, n_targets, ships_per_target)
    pop = zeros(pop_size, n_ships);
    for i = 1:pop_size
        % 确保每个目标分配到指定数量
        assign = zeros(1, n_ships);
        for t = 1:n_targets
            candidates = randperm(n_ships);
            selected = candidates(1:ships_per_target);
            assign(selected) = t;
        end
        pop(i,:) = assign;
    end
end

%% 适应度评估函数
function [total_cost, cluster_cost] = evaluate_individual(assign, ships, targets)
    dist_cost = 0;
    cluster_cost = 0;
    for t = 1:max(assign)
        members = ships(assign == t, :);
        if ~isempty(members)
            dist_cost = dist_cost + sum(vecnorm(members - targets(t,:), 2, 2));
            if size(members,1) > 1
                cluster_cost = cluster_cost + sum(pdist(members));
            end
        end
    end
    total_cost = dist_cost + 0.5 * cluster_cost; % 权重系数α=0.5
end

%% 锦标赛选择
function idx = tournament_select(fitness, k)
    candidates = randperm(length(fitness), k);
    [~, idx] = min(fitness(candidates));
    idx = candidates(idx);
end

%% 交叉操作（保留有效分配）
function [child1, child2] = crossover(parent1, parent2, ships_per_target)
    n_ships = length(parent1);
    crossover_point = randi(n_ships-1);
    
    child1 = [parent1(1:crossover_point), parent2(crossover_point+1:end)];
    child2 = [parent2(1:crossover_point), parent1(crossover_point+1:end)];
    
    % 修复容量约束
    child1 = repair_assignment(child1, ships_per_target);
    child2 = repair_assignment(child2, ships_per_target);
end

%% 变异操作
function child = mutate(parent, n_targets, mutation_rate, ships_per_target)
    child = parent;
    for i = 1:length(parent)
        if rand() < mutation_rate
            child(i) = randi(n_targets);
        end
    end
    child = repair_assignment(child, ships_per_target);
end

%% 修复分配方案满足容量约束
function assign = repair_assignment(assign, ships_per_target)
    [counts, targets] = groupcounts(assign');
    over_targets = targets(counts > ships_per_target);
    under_targets = targets(counts < ships_per_target);
    
    % 移除超额分配的船只
    for t = over_targets'
        members = find(assign == t);
        to_remove = members(ships_per_target+1:end);
        assign(to_remove) = 0;
    end
    
    % 将未分配的船只分配到不足的目标
    unassigned = find(assign == 0);
    for t = under_targets'
        need = ships_per_target - sum(assign == t);
        if need > 0 && ~isempty(unassigned)
            selected = unassigned(1:min(need, length(unassigned)));
            assign(selected) = t;
            unassigned = setdiff(unassigned, selected);
        end
    end
end
