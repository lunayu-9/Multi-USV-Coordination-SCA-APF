function D = custom_pdist(X)
% 手动实现 pdist 的欧氏距离计算（返回组内所有两两距离之和）
% 输入: X (N×2) 为无人船位置矩阵
% 输出: D 为所有两两距离之和

N = size(X, 1);
total_dist = 0;

for i = 1:N
    for j = i+1:N
        total_dist = total_dist + sqrt(sum((X(i,:) - X(j,:)).^2));
    end
end

D = total_dist;
end