function plot_dynamic_obstacle_avoidance(boatPath, dynTraj1, dynTraj2, MAP, goal)
    figure;
    hold on;
    axis([0 size(MAP,2)+1 0 size(MAP,1)+1]);
    axis equal;
    
    % 绘制无人船规划路径
    plot(boatPath(:,1), boatPath(:,2), 'b-', 'LineWidth', 2);
    
    % 绘制动态障碍物轨迹
    plot(dynTraj1(:,1), dynTraj1(:,2), 'r--', 'LineWidth', 1.5);
    plot(dynTraj2(:,1), dynTraj2(:,2), 'g--', 'LineWidth', 1.5);
    
    % 绘制目标点
    plot(goal(1), goal(2), 'kx', 'MarkerSize', 10, 'LineWidth',2);
    
    % 绘制静态障碍物（参考给定文件程序风格）
    [obs_i, obs_j] = find(MAP == 0);
    plot(obs_j, obs_i, 'ks', 'MarkerSize',8, 'MarkerFaceColor','k');
    
    % 检测并绘制无人船路径与动态障碍物轨迹的交点
    % 使用辅助函数计算交点（包含交点对应的段索引）
    intersections1 = compute_polylineIntersections(boatPath, dynTraj1, 0.1);
    intersections2 = compute_polylineIntersections(boatPath, dynTraj2, 0.1);
    
    % 绘制交点：若对应的段索引相差较小（例如小于等于1）视为碰撞（实心），否则为空心
    collision_thresh = 1; 
    for k = 1:length(intersections1)
        if abs(intersections1(k).boat_index - intersections1(k).dyn_index) <= collision_thresh
            plot(intersections1(k).point(1), intersections1(k).point(2), 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
        else
            plot(intersections1(k).point(1), intersections1(k).point(2), 'ro', 'MarkerSize',8);
        end
    end
    for k = 1:length(intersections2)
        if abs(intersections2(k).boat_index - intersections2(k).dyn_index) <= collision_thresh
            plot(intersections2(k).point(1), intersections2(k).point(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
        else
            plot(intersections2(k).point(1), intersections2(k).point(2), 'go', 'MarkerSize',8);
        end
    end
    
    xlabel('X');
    ylabel('Y');
    title('动态避障过程及路径交叉情况');
    hold off;
end
