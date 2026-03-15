classdef matlab_ablation_config < handle
    % 消融实验配置类
    % 用于切换Pure Visual/Pure IMU/Full Fusion模式
    %
    % 使用方法：
    %   config = matlab_ablation_config('full');  % 'pure_visual', 'pure_imu', 'full'
    %   if config.should_use_visual()
    %       % 使用visual处理
    %   end
    
    properties
        mode  % 'pure_visual', 'pure_imu', 'full'
    end
    
    methods
        function obj = matlab_ablation_config(mode)
            % 构造函数
            if nargin < 1
                mode = 'full';
            end
            
            valid_modes = {'pure_visual', 'pure_imu', 'full'};
            if ~ismember(mode, valid_modes)
                error('Invalid mode: %s. Must be one of: %s', mode, strjoin(valid_modes, ', '));
            end
            
            obj.mode = mode;
            fprintf('🔬 Ablation Mode: %s\n', mode);
        end
        
        function result = should_use_visual(obj)
            % 是否使用视觉
            result = ismember(obj.mode, {'pure_visual', 'full'});
        end
        
        function result = should_use_imu(obj)
            % 是否使用IMU
            result = ismember(obj.mode, {'pure_imu', 'full'});
        end
        
        function fusion_type = get_fusion_type(obj)
            % 获取融合类型
            if strcmp(obj.mode, 'full')
                fusion_type = 'complementary';
            else
                fusion_type = 'none';
            end
        end
        
        function print_config(obj)
            % 打印当前配置
            fprintf('\n=== Ablation Configuration ===\n');
            fprintf('Mode: %s\n', obj.mode);
            fprintf('Use Visual: %d\n', obj.should_use_visual());
            fprintf('Use IMU: %d\n', obj.should_use_imu());
            fprintf('Fusion Type: %s\n', obj.get_fusion_type());
            fprintf('==============================\n\n');
        end
    end
end
