function [total_path_length, max_angle] = calculate_path_metrics(...
    paths_leaders1, paths_followers1, paths_leaders2, paths_followers2)
    
    total_path_length = 0;
    max_angle = 0;
    
    % 目标1（第一组）
    path_length = calculate_path_length(paths_leaders1);
    total_path_length = total_path_length + path_length;
    angle = compute_max_angle(paths_leaders1);
    max_angle = max(max_angle, angle);
    for j = 2:length(paths_followers1)
        path = paths_followers1{j};
        path_length = calculate_path_length(path);
        total_path_length = total_path_length + path_length;
        angle = compute_max_angle(path);
        max_angle = max(max_angle, angle);
    end
    
    % 目标2（第二组）
    path_length = calculate_path_length(paths_leaders2);
    total_path_length = total_path_length + path_length;
    angle = compute_max_angle(paths_leaders2);
    max_angle = max(max_angle, angle);
    for j = 2:length(paths_followers2)
        path = paths_followers2{j};
        path_length = calculate_path_length(path);
        total_path_length = total_path_length + path_length;
        angle = compute_max_angle(path);
        max_angle = max(max_angle, angle);
    end
end
