function path_length = calculate_path_length(path)
    path_length = 0;
    for i = 1:size(path,1)-1
        dx = path(i+1,1) - path(i,1);
        dy = path(i+1,2) - path(i,2);
        path_length = path_length + sqrt(dx^2 + dy^2);
    end
end
