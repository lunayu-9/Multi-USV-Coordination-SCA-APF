function intersections = compute_polylineIntersections(poly1, poly2, tol)
    if nargin < 3
        tol = 1e-3;
    end
    intersections = [];
    count = 0;
    n1 = size(poly1,1);
    n2 = size(poly2,1);
    for i = 1:n1-1
        p1 = poly1(i,:);
        p2 = poly1(i+1,:);
        for j = 1:n2-1
            q1 = poly2(j,:);
            q2 = poly2(j+1,:);
            [isIntersect, pt, t, u] = lineSegmentIntersect(p1, p2, q1, q2, tol);
            if isIntersect
                count = count + 1;
                % 保存交点及对应的分段“时间”（近似于迭代索引）
                intersections(count).point = pt;
                intersections(count).boat_index = i + t;
                intersections(count).dyn_index  = j + u;
            end
        end
    end
end

function [isIntersect, intersectPoint, t, u] = lineSegmentIntersect(p1, p2, q1, q2, tol)
    % 计算两线段 p1-p2 与 q1-q2 是否相交
    % 若相交，返回交点、以及参数 t 和 u（各自在线段上的比例位置）
    if nargin < 5
        tol = 1e-3;
    end
    r = p2 - p1;
    s = q2 - q1;
    rxs = r(1)*s(2) - r(2)*s(1);
    qp = q1 - p1;
    qpxr = qp(1)*r(2) - qp(2)*r(1);
    
    if abs(rxs) < tol  % 平行或共线
        isIntersect = false;
        intersectPoint = [];
        t = NaN;
        u = NaN;
        return;
    end
    
    t = (qp(1)*s(2) - qp(2)*s(1)) / rxs;
    u = qpxr / rxs;
    
    if (t >= -tol) && (t <= 1+tol) && (u >= -tol) && (u <= 1+tol)
        isIntersect = true;
        intersectPoint = p1 + t*r;
    else
        isIntersect = false;
        intersectPoint = [];
    end
end
