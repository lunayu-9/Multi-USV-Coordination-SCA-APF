function [groups, totalCost, nearestShips] = improvedOptimizeGroupsThree(Fleet, goal1, goal2, goal3, obs, MAP_size) % improvedOptimizeGroupsThree - 将无人船分配到三个目标 % % 输入: % Fleet - 无人船当前位置矩阵，每一行对应一艘船的位置 [numRobots x 2] % goal1 - 目标1的位置 (1x2) % goal2 - 目标2的位置 (1x2) % goal3 - 目标3的位置 (1x2) % obs - 障碍物信息（此版本中未使用，但保留接口一致性） % MAP_size - 地图尺寸（此版本中未使用，但保留接口一致性） % % 输出: % groups - 分配结果，每个元素表示对应无人船的目标组（1、2或3） % totalCost - 分配总代价，定义为各无人船到其目标的欧氏距离之和 % nearestShips - 各组中离目标最近的船的索引（领导船）

% 计算无人船数量
numRobots = size(Fleet, 1);
groups = zeros(numRobots, 1);

% 计算每艘船到各目标的欧氏距离
d1 = vecnorm(Fleet - goal1, 2, 2);
d2 = vecnorm(Fleet - goal2, 2, 2);
d3 = vecnorm(Fleet - goal3, 2, 2);

% 固定分配2艘船到第三目标：选取离 goal3 最近的两个 
[~, sortedIdx] = sort(d3);
if numRobots >= 2
    group3 = sortedIdx(1:2);
else
    group3 = sortedIdx;
end
groups(group3) = 3;

% 剩余无人船分配给目标1和目标2：初步按距离比较
remaining = setdiff(1:numRobots, group3);
for i = remaining
    if d1(i) < d2(i)
        groups(i) = 1;
    else
        groups(i) = 2;
    end
end

% 若结果导致组1或组2为空，则将剩余均分
rem = remaining;
if all(groups(rem) == 2)
    half = floor(length(rem)/2);
    groups(rem(1:half)) = 1;
elseif all(groups(rem) == 1)
    half = floor(length(rem)/2);
    groups(rem(1:half)) = 2;
end

% 计算总代价：各船到其目标的距离之和
totalCost = sum(d1(groups==1)) + sum(d2(groups==2)) + sum(d3(groups==3));

% 计算各组中离目标最近的船作为领导船
nearestShips = zeros(3,1);
for g = 1:3
    indices = find(groups == g);
    if isempty(indices)
        nearestShips(g) = NaN;
    else
        if g == 1
            [~, idx] = min(d1(indices));
            nearestShips(g) = indices(idx);
        elseif g == 2
            [~, idx] = min(d2(indices));
            nearestShips(g) = indices(idx);
        else
            [~, idx] = min(d3(indices));
            nearestShips(g) = indices(idx);
        end
    end
end