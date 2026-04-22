function [path, success, dynTraj1, dynTraj2] = improvedArtificialPotentialField(startPos, goal, obs, dynamicObs1, dynamicObs2, MAP_size, dynamicSpeed)
    % 单无人船版改进人工势场法（动态障碍物参数暂不使用）
    % 设置参数（可根据需要调整）
    params.Eta_att    = 1;    % 引力增益系数
    params.Eta_rep_ob = 100;  % 障碍物斥力增益系数
    params.d0         = 3;    % 障碍物斥力影响范围
    params.len_step   = 0.5;  % 移动步长

    % 初始化无人船位置与路径记录
    pos = startPos;
    path = pos;
    success = false;

    max_iter = 1000;
    for iter = 1:max_iter
        % 计算目标引力
        delta_goal = goal - pos;
        dist_goal = norm(delta_goal);
        if dist_goal == 0
            break;
        end
        unit_vec_goal = delta_goal / dist_goal;
        F_att = params.Eta_att * unit_vec_goal;
        
        % 计算障碍物斥力
        F_rep = [0, 0];
        for j = 1:size(obs, 1)
            % 注意：假设 obs 坐标顺序为 [y, x]，因此交换顺序
            delta_obs = pos - obs(j, [2, 1]);
            dist_obs = norm(delta_obs);
            if dist_obs < params.d0 && dist_obs > 0
                unit_vec_obs = delta_obs / dist_obs;
                F_rep = F_rep + params.Eta_rep_ob * (1/dist_obs - 1/params.d0) * (1/(dist_obs^2)) * unit_vec_obs;
            end
        end
        
        % 合成总力并归一化
        F_sum = F_att + F_rep;
        F_norm = norm(F_sum);
        if F_norm == 0
            break;
        end
        UnitVec_Fsum = F_sum / F_norm;
        
        % 更新无人船位置
        pos = pos + params.len_step * UnitVec_Fsum;
        path = [path; pos];
        
        % 判断是否到达目标
        if norm(pos - goal) < 1
            success = true;
            break;
        end
    end

    % 单无人船场景，不涉及动态障碍物轨迹
    dynTraj1 = [];
    dynTraj2 = [];
end
