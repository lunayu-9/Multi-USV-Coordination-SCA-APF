function [paths_leaders1, paths_followers1, paths_leaders2, paths_followers2] = ...
    plan_and_control_formation_combined(Fleet1, goal1, Fleet2, goal2, obs, MAP_size, params)
    
    %% 初始化路径数据
    paths_leaders1 = Fleet1(1, 1:2);
    paths_followers1 = cell(size(Fleet1,1), 1);
    for j = 1:size(Fleet1,1)
        paths_followers1{j} = Fleet1(j, 1:2);
    end
    
    paths_leaders2 = Fleet2(1, 1:2);
    paths_followers2 = cell(size(Fleet2,1), 1);
    for j = 1:size(Fleet2,1)
        paths_followers2{j} = Fleet2(j, 1:2);
    end
    
    %% 运动与编队控制
    figure;
    hold on;
    axis equal; 
    xlim([0, MAP_size(2)]); 
    ylim([0, MAP_size(1)]);
    
    % 预先绘制静态障碍物（使用 fill 绘制，不会受到后续清除影响）
    for i = 1:size(obs,1)
        temp = obs(i,:);
        fill([temp(2)-1, temp(2), temp(2), temp(2)-1], ...
             [temp(1)-1, temp(1)-1, temp(1), temp(1)], 'k');
    end
    
    % 预先绘制静态目标点（同样使用 fill 绘制）
    fill([goal1(1)-0.5, goal1(1)+0.5, goal1(1)+0.5, goal1(1)-0.5], ...
         [goal1(2)-0.5, goal1(2)-0.5, goal1(2)+0.5, goal1(2)+0.5], 'r');
    fill([goal2(1)-0.5, goal2(1)+0.5, goal2(1)+0.5, goal2(1)-0.5], ...
         [goal2(2)-0.5, goal2(2)-0.5, goal2(2)+0.5, goal2(2)+0.5], 'b');
    
    % 设置迭代计数器及绘图更新频率（例如每5次更新一次）
    iter = 0;
    plot_update_interval = 5;
    
    goalflags = [0, 0];
    while ~all(goalflags)
        iter = iter + 1;
        
        % 调用各自的路径规划与编队控制模块，更新舰队位置与路径
        [Fleet1, paths_leaders1, paths_followers1] = plan_and_control_formation_1(...
            Fleet1, goal1, obs, MAP_size, params, paths_leaders1, paths_followers1);
        [Fleet2, paths_leaders2, paths_followers2] = plan_and_control_formation_2(...
            Fleet2, goal2, obs, MAP_size, params, paths_leaders2, paths_followers2);
        
        % 每隔一定次数更新绘图
        if mod(iter, plot_update_interval) == 1
            % 仅删除之前更新时绘制的动态对象，保留静态障碍物和目标
            delete(findobj(gca, 'Tag', 'Dynamic'));
            
            % 绘制第一组路径（直接在绘图时设置 Tag）
            h1 = plot(paths_leaders1(:,1), paths_leaders1(:,2), 'r', 'LineWidth', 1, 'Tag', 'Dynamic');
            for j = 2:size(Fleet1,1)
                h2 = plot(paths_followers1{j}(:,1), paths_followers1{j}(:,2), 'm', 'LineWidth', 1, 'Tag', 'Dynamic');
            end
            
            % 绘制第二组路径
            h3 = plot(paths_leaders2(:,1), paths_leaders2(:,2), 'b', 'LineWidth', 1, 'Tag', 'Dynamic');
            for j = 2:size(Fleet2,1)
                h4 = plot(paths_followers2{j}(:,1), paths_followers2{j}(:,2), 'c', 'LineWidth', 1, 'Tag', 'Dynamic');
            end
            
            % 绘制各组无人船的位置与菱形编队
            draw_fleet(Fleet1, 'r', 'm');
            draw_fleet(Fleet2, 'b', 'c');
            draw_formation(Fleet1);
            draw_formation(Fleet2);
            % 由于 draw_fleet 与 draw_formation 没有返回句柄，这里统一对当前坐标区内的新绘制对象（rectangle 和 line）加上 Tag
            hRect = findobj(gca, 'Type', 'rectangle');
            set(hRect, 'Tag', 'Dynamic');
            hLine = findobj(gca, 'Type', 'line');
            set(hLine, 'Tag', 'Dynamic');
            
            drawnow;
        end
        
        % 判断是否到达目标
        if norm(Fleet1(1,1:2) - goal1) < 0.3
            goalflags(1) = 1;
        end
        if norm(Fleet2(1,1:2) - goal2) < 0.3
            goalflags(2) = 1;
        end
    end
    
    % 绘制最终路径（加粗显示），同时将这些动态对象标记
    hFinal1 = plot(paths_leaders1(:,1), paths_leaders1(:,2), 'r', 'LineWidth', 2, 'Tag', 'Dynamic');
    for j = 2:size(Fleet1,1)
        hFinal2 = plot(paths_followers1{j}(:,1), paths_followers1{j}(:,2), 'm', 'LineWidth', 2, 'Tag', 'Dynamic');
    end
    hFinal3 = plot(paths_leaders2(:,1), paths_leaders2(:,2), 'b', 'LineWidth', 2, 'Tag', 'Dynamic');
    for j = 2:size(Fleet2,1)
        hFinal4 = plot(paths_followers2{j}(:,1), paths_followers2{j}(:,2), 'c', 'LineWidth', 2, 'Tag', 'Dynamic');
    end
    draw_formation(Fleet1);
    draw_formation(Fleet2);
    % 同样为最终动态对象添加 Tag
    hRect = findobj(gca, 'Type', 'rectangle');
    set(hRect, 'Tag', 'Dynamic');
    hLine = findobj(gca, 'Type', 'line');
    set(hLine, 'Tag', 'Dynamic');
    
    title('无人船编队运动轨迹与菱形编队连接');
    xlabel('X 轴');
    ylabel('Y 轴');
end
