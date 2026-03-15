function [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list_nosort(subFoldersPathSet, imgType, iSubFolder)
% 临时版本：不使用sortObj排序，直接返回dir结果
% 用于调试sortObj问题
    
    % get current sub floder path from the path set of all sub folders
    curFolderPath =  subFoldersPathSet{iSubFolder}; 
    
    % get all image file path list in current sub folder
    % 使用fullfile正确拼接路径（自动添加分隔符）
    imgFilesPathList = dir(fullfile(curFolderPath, imgType));
    
    % 临时：跳过sortObj排序
    % imgFilesPathList = sortObj(imgFilesPathList);
    
    % 使用MATLAB内置sort（按文件名排序）
    if ~isempty(imgFilesPathList)
        [~, sortIdx] = sort({imgFilesPathList.name});
        imgFilesPathList = imgFilesPathList(sortIdx);
    end

    % get the num of all images in current sub folder
    numImgs = length(imgFilesPathList); 
    
end
