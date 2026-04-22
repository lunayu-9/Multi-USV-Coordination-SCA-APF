%% 实验参数设置
rng(42); 
N = 12;  % 明确定义N
K = 4;   
ships_per_target = 3; 

%% 生成位置数据（不再依赖外部文件）
ships_pos = rand(N,2)*50;       
targets_pos = rand(K,2)*30 + 10;

%% 运行算法（参数传递验证）
[assign_ta1, cost_ta1] = hungarian_method(ships_pos, targets_pos, ships_per_target);
[assign_ta3, cost_ta3, cluster_ta3] = improved_hungarian(ships_pos, targets_pos, ships_per_target);
[assign_ga, cost_ga, cluster_ga] = improved_ga(ships_pos, targets_pos, ships_per_target);

%% 输出结果（示例）
fprintf('传统匈牙利算法: N=%d, 代价=%.1f\n', N, cost_ta1);
fprintf('改进匈牙利算法: N=%d, 代价=%.1f\n', N, cost_ta3); 
fprintf('改进遗传算法: N=%d, 代价=%.1f\n', N, cost_ga);
