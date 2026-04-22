function [assignment, cost] = traditional_hungarian(cost_matrix)
    % 输入: cost_matrix-代价矩阵(N×K)
    % 输出: assignment-分配结果, cost-总代价
    
    % 修正参数：第二个参数应为未匹配代价（标量数值）
    [assignment, cost] = matchpairs(cost_matrix, 1e6); % 1e6为未匹配惩罚值
end