function file = sortObj(file)  
% Alphanumeric / Natural-Order sort the strings in a cell array of strings (1xN char).
%
% (c) 2012 Stephen Cobeldick

    % 处理空文件列表
    if isempty(file)
        return;
    end
    
    % 初始化cell数组
    A = cell(1, length(file));
    
    for i = 1 : length(file)  
        A{i} = file(i).name;  
    end  
    [~, ind] = natsort(A);  

    % 预分配结构体数组
    files = file;  % 保持原结构
    for j = 1 : length(file)  
        files(j) = file(ind(j));  
    end  
    
    % 返回排序后的结构体数组（保持原始方向）
    file = files;