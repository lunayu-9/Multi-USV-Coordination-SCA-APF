function [groups, cost, time] = genetic_algorithm(ships, targets, group_size)
% 输入: 同上
% 输出: 分组结果、总代价、时间

tic;
N = size(ships, 1);
K = size(targets, 1);

% 初始化种群
population_size = 50;
population = cell(population_size, 1);
for i = 1:population_size
    population{i} = random_grouping(N, K, group_size);
end

% 遗传迭代
max_generations = 100;
best_cost = Inf;
for gen = 1:max_generations
    % 计算适应度
    costs = zeros(population_size, 1);
    for i = 1:population_size
        costs(i) = calculate_cost(population{i}, ships, targets);
    end
    [min_cost, idx] = min(costs);
    if min_cost < best_cost
        best_cost = min_cost;
        best_group = population{idx};
    end
    
    % 选择、交叉、变异
    new_population = cell(population_size, 1);
    for i = 1:population_size
        parent1 = tournament_selection(population, costs);
        parent2 = tournament_selection(population, costs);
        child = crossover(parent1, parent2);
        child = mutate(child, group_size);
        new_population{i} = child;
    end
    population = new_population;
end

time = toc;
groups = best_group;
cost = best_cost;
end

function group = random_grouping(N, K, group_size)
% 随机生成合法分组
group = cell(K, 1);
all_idx = randperm(N);
ptr = 1;
for k = 1:K
    group{k} = all_idx(ptr:ptr+group_size(k)-1);
    ptr = ptr + group_size(k);
end
end

function selected = tournament_selection(population, costs)
% 锦标赛选择
tournament_size = 5;
candidates = randi(length(population), tournament_size, 1);
[~, idx] = min(costs(candidates));
selected = population{candidates(idx)};
end

function child = crossover(parent1, parent2)
% 单点交叉
K = length(parent1);
crossover_point = randi(K-1);
child = [parent1(1:crossover_point); parent2(crossover_point+1:end)];
end

function group = mutate(group, group_size)
% 随机交换两个成员
k1 = randi(length(group));
k2 = randi(length(group));
if k1 == k2, return; end
member1 = randi(length(group{k1}));
member2 = randi(length(group{k2}));
temp = group{k1}(member1);
group{k1}(member1) = group{k2}(member2);
group{k2}(member2) = temp;
end