function cleanupInstruments()
    % 检查基础工作区和后台，强制关闭并删除所有仪器对象
    fprintf('\n--- 正在强力释放 VISA 仪器资源 ---\n');
    
    % 1. 查找所有内存中的仪器对象 (包括卡死的幽灵连接)
    all_objects = instrfind; 
    
    if ~isempty(all_objects)
        fprintf('⚠️ 发现 %d 个未释放的残留连接，正在强制清理...\n', length(all_objects));
        
        for i = 1:length(all_objects)
            try
                % 尝试优雅关闭
                fclose(all_objects(i)); 
            catch
                % 如果已经断开，直接忽略错误
            end
            % 彻底从内存删除
            delete(all_objects(i)); 
        end
        fprintf('✅ 后台残留对象已成功斩断并销毁。\n');
    else
        fprintf('✅ 没有发现后台挂起的仪器对象，通道处于空闲状态。\n');
    end
    
    % 2. 清理工作区变量 (防止 handle 悬空)
    evalin('base', 'clear scope sigGen;');
end