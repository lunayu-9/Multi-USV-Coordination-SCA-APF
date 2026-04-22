function [path, max_angle, collision_points, dynamic_obs_history] = ...
    single_ship_planning(start_pos, goal, static_obs, dynamic_obs, MAP_size, params)
    
    path = start_pos;
    current_pos = start_pos;
    collision_points = [];
    max_iter = 1000;
    
    % 记录障碍物历史轨迹
    dynamic_obs_history = repmat(struct('trajectory',[]),1,length(dynamic_obs));
    
    for iter = 1:max_iter
        % 更新动态障碍物
        dynamic_obs = update_dynamic_obstacles(dynamic_obs, MAP_size, params);
        
        % 计算控制力
        [F_total, is_collision] = calculate_total_force(current_pos, goal, static_obs, dynamic_obs, params);
        
        % 记录碰撞事件
        if is_collision
            collision_points = [collision_points; current_pos];
        end
        
        % 更新船体位置
        current_pos = current_pos + params.len_step * F_total/norm(F_total);
        path = [path; current_pos];
        
        % 终止条件
        if norm(current_pos - goal) < params.len_step
            break;
        end
    end
    
    % 保存障碍物轨迹历史
    for i = 1:length(dynamic_obs)
        dynamic_obs_history(i).trajectory = dynamic_obs(i).trajectory;
    end
    
    max_angle = compute_max_angle(path);
end

function dynamic_obs = update_dynamic_obstacles(dynamic_obs, MAP_size, params)
    for i = 1:length(dynamic_obs)
        % 记录轨迹
        dynamic_obs(i).trajectory(end+1,:) = dynamic_obs(i).position;
        
        % 运动更新
        new_pos = dynamic_obs(i).position + params.len_step * dynamic_obs(i).speed * dynamic_obs(i).direction;
        
        % 边界处理
        if new_pos(1)<1 || new_pos(1)>MAP_size(1) || new_pos(2)<1 || new_pos(2)>MAP_size(2)
            dynamic_obs(i).direction = -dynamic_obs(i).direction;
            new_pos = dynamic_obs(i).position;
        end
        
        dynamic_obs(i).position = new_pos;
    end
end
