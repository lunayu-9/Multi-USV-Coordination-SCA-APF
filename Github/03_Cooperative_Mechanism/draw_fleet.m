function draw_fleet(Fleet, leader_color, follower_color)
    for j = 1:size(Fleet, 1)
        if j == 1
            rectangle('Position', [Fleet(j,1)-0.2, Fleet(j,2)-0.2, 0.4, 0.4], ...
                'Curvature', 1, 'FaceColor', leader_color);
        else
            rectangle('Position', [Fleet(j,1)-0.2, Fleet(j,2)-0.2, 0.4, 0.4], ...
                'Curvature', 1, 'FaceColor', follower_color);
        end
    end
end
