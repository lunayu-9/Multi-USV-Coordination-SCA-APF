%% 实验参数设置
rng(42); % 固定随机种子保证可重复性
N = 12;  % 无人船数量
K = 4;   % 目标点数量
ships_per_target = 3; % 每个目标需分配的无人船数

%% 生成随机初始位置 (50x50海域)
ships_pos = rand(N,2)*50;       % 无人船初始位置
targets_pos = rand(K,2)*30 + 10;% 目标点集中在中心区域

%% 可视化初始场景
figure;
scatter(ships_pos(:,1), ships_pos(:,2), 'b^', 'filled'); hold on;
scatter(targets_pos(:,1), targets_pos(:,2), 100, 'r*');
legend('无人船', '目标点'); title('初始场景分布');
xlabel('X坐标 (m)'); ylabel('Y坐标 (m)'); grid on;
