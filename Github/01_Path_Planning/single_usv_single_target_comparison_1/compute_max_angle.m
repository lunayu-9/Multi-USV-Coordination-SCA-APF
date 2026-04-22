function max_angle = compute_max_angle(path)
    if size(path, 1) < 3
        max_angle = 0;
        return;
    end
    max_angle = 0;
    for i = 2:size(path, 1)-1
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        if norm(v1)==0 || norm(v2)==0
            continue;
        end
        angle = acosd(dot(v1, v2) / (norm(v1) * norm(v2)));
        if angle > max_angle
            max_angle = angle;
        end
    end
end
