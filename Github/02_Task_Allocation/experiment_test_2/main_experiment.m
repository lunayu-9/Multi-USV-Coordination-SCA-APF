% 实验参数
map_size = [50, 50];  % 地图尺寸
N = 8; K = 2;         % 8无人船-2目标点
group_size = [4, 4];   % 每组4艘
seed = 42;             % 随机种子

% 生成环境
[ships, targets, static_obs] = generate_environment(N, K, map_size, seed);

% 运行改进匈牙利算法
[groups_ih, cost_ih, time_ih] = improved_hungarian(ships, targets, group_size);

% 运行传统匈牙利算法（需构造代价矩阵）
cost_matrix = custom_pdist2(ships, targets); % 使用自定义距离计算
[assignment, cost_ha] = traditional_hungarian(cost_matrix);

% 运行改进遗传算法
[groups_ga, cost_ga, time_ga] = genetic_algorithm(ships, targets, group_size);

% 输出结果
fprintf('算法对比结果:\n');
fprintf('改进匈牙利算法 - 代价: %.2f, 时间: %.4f秒\n', cost_ih, time_ih);
fprintf('传统匈牙利算法 - 代价: %.2f, 时间: %.4f秒\n', sum(cost_matrix(sub2ind(size(cost_matrix), assignment(:,1), assignment(:,2)))), 0.01);
fprintf('改进遗传算法   - 代价: %.2f, 时间: %.4f秒\n', cost_ga, time_ga);

% 可视化分组结果
figure;
plot_environment(ships, targets, static_obs);
hold on;
colors = ['r', 'b'];
for k = 1:K
    plot(ships(groups_ih{k},1), ships(groups_ih{k},2), 'o', 'Color', colors(k), 'MarkerSize', 8);
end
title('改进匈牙利算法分组结果');