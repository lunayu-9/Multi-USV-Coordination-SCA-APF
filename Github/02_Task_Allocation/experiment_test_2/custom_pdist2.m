function D = custom_pdist2(A, B)
    % 手动实现 pdist2 的欧氏距离计算
    M = size(A, 1);
    N = size(B, 1);
    D = zeros(M, N);
    for i = 1:M
        for j = 1:N
            D(i,j) = sqrt(sum((A(i,:) - B(j,:)).^2));
        end
    end
end