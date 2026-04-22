function [groups, total_cost, time] = improved_hungarian(ships, targets, group_size)
% 输入: ships-无人船位置(N×2), targets-目标点位置(K×2), group_size-每组容量
% 输出: groups-分组结果, total_cost-总代价, time-运算时间

tic;
N = size(ships, 1);
K = size(targets, 1);

% 1. 计算扩展距离矩阵（含编队紧密度预判）
D_ext = zeros(N, K);
for i = 1:N
    for k = 1:K
        dist = sqrt(sum((ships(i,:) - targets(k,:)).^2)); % 欧氏距离
        group_dist = sum(custom_pdist2(ships, ships(i,:))); % 编队紧密度
        D_ext(i,k) = dist + 0.5 * group_dist; % 权重α=0.5
    end
end

% 2. 启发式预分配（按倾向性ρ）
rho = zeros(N, K);
for i = 1:N
    inv_dist = 1 ./ sqrt(sum((ships(i,:) - targets).^2, 2)); % 1/距离
    rho(i,:) = inv_dist / sum(inv_dist); % 归一化倾向性
end

% 预分配：每个目标点选择前n_k个倾向性最高的无人船
pre_groups = cell(K, 1);
for k = 1:K
    [~, idx] = sort(rho(:,k), 'descend');
    pre_groups{k} = idx(1:group_size(k));
end

% 3. 局部搜索优化（模拟退火）
current_groups = pre_groups;
current_cost = calculate_cost(current_groups, ships, targets);
T0 = 100; T_min = 1; max_iter = 50;

for iter = 1:max_iter
    T = T0 * (1 - iter/max_iter);
    % 随机选择两组交换成员
    k1 = randi(K); k2 = randi(K);
    if k1 == k2, continue; end
    
    % 交换成员
    member1 = randi(length(current_groups{k1}));
    member2 = randi(length(current_groups{k2}));
    temp = current_groups{k1}(member1);
    current_groups{k1}(member1) = current_groups{k2}(member2);
    current_groups{k2}(member2) = temp;
    
    % 计算新代价
    new_cost = calculate_cost(current_groups, ships, targets);
    delta = new_cost - current_cost;
    
    % 模拟退火接受准则
    if delta < 0 || exp(-delta/T) > rand()
        current_cost = new_cost;
    else
        % 回退交换
        current_groups{k2}(member2) = current_groups{k1}(member1);
        current_groups{k1}(member1) = temp;
    end
end

time = toc;
groups = current_groups;
total_cost = current_cost;
end

function cost = calculate_cost(groups, ships, targets)
cost = 0;
for k = 1:length(groups)
    group_ships = ships(groups{k}, :);
    target_pos = targets(k,:);
    dist_cost = sum(sqrt(sum((group_ships - target_pos).^2, 2))); % 距离代价
    cohesion_cost = custom_pdist(group_ships); % 调用自定义函数
    cost = cost + dist_cost + 0.5 * cohesion_cost;
end
end