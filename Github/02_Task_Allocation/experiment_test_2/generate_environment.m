function [ships, targets, static_obs] = generate_environment(N, K, map_size, seed)
% 输入: N-无人船数量, K-目标点数量, map_size-地图尺寸 [w,h], seed-随机种子
% 输出: ships-无人船位置(N×2), targets-目标点位置(K×2), static_obs-静态障碍物

rng(seed); % 固定随机种子

% 生成无人船初始位置（左下区域）
ships = [rand(N,1)*map_size(1)*0.2, rand(N,1)*map_size(2)*0.2];

% 生成目标点（上半区域）
targets = [rand(K,1)*map_size(1), map_size(2)*0.5 + rand(K,1)*map_size(2)*0.5];

% 生成静态障碍物（围绕目标点随机分布）
static_obs = [];
for k = 1:K
    theta = rand(ceil(N*1.5),1)*2*pi; % 障碍物数量为1.5倍N
    r = 0.5 + rand(ceil(N*1.5),1)*1.5; % 半径U(0.5,2.0)
    obs_pos = targets(k,:) + [r.*cos(theta), r.*sin(theta)];
    static_obs = [static_obs; obs_pos];
end
end