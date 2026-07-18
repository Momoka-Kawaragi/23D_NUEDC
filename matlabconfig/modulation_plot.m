%% modulation_plot.m — STM32F4 调制度测量实时显示
%  串口格式 (FM):
%    MOD_FM,mf=2.50,df=2500.0,fc=10000.0,fm=1000.0,car=1.23,side1=0.45,side2=0.15,fit=0.0035,auto=1,status=0\r\n
%  串口格式 (AM):
%    MOD_AM,ma=0.80,fc=10000.0,fm=1000.0,car=1.23,side=0.49,status=0\r\n

function modulation_plot()
    %% ============ 用户配置区 ============
    comPort   = 'COM10';       % 串口号
    baudRate  = 230400;       % 波特率 (与 UART1 一致)
    logFile   = 'modulation_log.csv';  % 日志文件 (留空 '' 则不记录)
    %% ====================================

    % 清理旧的串口对象
    old = instrfind('Port', comPort);
    if ~isempty(old), fclose(old); delete(old); end

    % 打开串口
    try
        s = serialport(comPort, baudRate, 'Timeout', 5);
    catch ME
        error('无法打开串口 %s: %s', comPort, ME.message);
    end
    configureTerminator(s, "CR/LF");
    s.InputBufferSize = 64 * 1024;
    flush(s);
    pause(0.1);

    % ==== 历史缓冲区 ====
    MAX_HIST = 200;          % 显示最近 N 个点
    hist.time   = zeros(MAX_HIST, 1);
    hist.fm_mf  = nan(MAX_HIST, 1);
    hist.fm_df  = nan(MAX_HIST, 1);
    hist.fm_err = nan(MAX_HIST, 1);
    hist.am_ma  = nan(MAX_HIST, 1);
    hist.idx    = 0;

    % ==== 创建图形窗口 ====
    fig = figure('Name', 'Modulation Meter — Real-time', ...
                 'NumberTitle', 'off', ...
                 'CloseRequestFcn', @onClose);
    drawnow;

    % 子图1: FM 调频指数 mf + 频偏
    ax1 = subplot(2, 1, 1, 'Parent', fig);
    yyaxis(ax1, 'left');
    hMf = plot(ax1, NaN, NaN, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    ylabel(ax1, 'mf (modulation index)'); grid(ax1, 'on');
    yyaxis(ax1, 'right');
    hDf = plot(ax1, NaN, NaN, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
    ylabel(ax1, '\Deltaf (Hz)'); grid(ax1, 'on');
    title(ax1, 'FM — Waiting for MOD_FM...');
    legend(ax1, 'mf', '\Deltaf', 'Location', 'best');

    % 子图2: AM 调制度 ma
    ax2 = subplot(2, 1, 2, 'Parent', fig);
    hMa = plot(ax2, NaN, NaN, 'g-^', 'LineWidth', 1.5, 'MarkerSize', 4);
    ylabel(ax2, 'ma (0~1)'); grid(ax2, 'on');
    title(ax2, 'AM — Waiting for MOD_AM...');
    ylim(ax2, [0, 1.2]);
    yline(ax2, 1.0, 'r--', 'Overmod');

    % ==== 日志文件 ====
    logFid = -1;
    if ~isempty(logFile)
        logFid = fopen(logFile, 'w');
        if logFid > 0
            fprintf(logFid, 'timestamp,type,mf,df,fc,fm,car,side1,side2,fit,auto,status,ma,side\n');
        end
    end

    fprintf('串口 %s 已打开, 波特率 %d, 等待数据...\n', comPort, baudRate);
    tStart = tic;

    % ============== 主循环 ==============
    while isvalid(fig) && ishandle(fig)
        try
            if s.NumBytesAvailable > 0
                raw = readline(s);
                if strlength(raw) == 0, continue; end
                line = strtrim(raw);

                % --- FM 帧 ---
                if startsWith(line, 'MOD_FM')
                    parsed = parseKeyVal(line);
                    if isempty(parsed), continue; end

                    hist.idx = mod(hist.idx, MAX_HIST) + 1;
                    t = toc(tStart);
                    hist.time(hist.idx)   = t;
                    hist.fm_mf(hist.idx)  = getField(parsed, 'mf');
                    hist.fm_df(hist.idx)  = getField(parsed, 'df');
                    hist.fm_err(hist.idx) = getField(parsed, 'fit');

                    % 更新 FM 子图 (显示最近的有效点)
                    valid = ~isnan(hist.fm_mf);
                    if any(valid)
                        tVals = hist.time(valid);
                        mfVals = hist.fm_mf(valid);
                        dfVals = hist.fm_df(valid);
                        [tSort, idxSort] = sort(tVals);

                        set(hMf, 'XData', tSort, 'YData', mfVals(idxSort));
                        set(hDf, 'XData', tSort, 'YData', dfVals(idxSort));
                        ylim(ax1, 'auto');
                        xlabel(ax1, sprintf('Time (s)  [fc=%.0f Hz, fm=%.0f Hz, fit=%.4f, auto=%d, status=%d]', ...
                            getField(parsed, 'fc'), getField(parsed, 'fm'), ...
                            getField(parsed, 'fit'), getField(parsed, 'auto'), getField(parsed, 'status')));
                    end

                    % 记录日志
                    if logFid > 0
                        fprintf(logFid, '%.3f,FM,%.3f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.4f,%d,%d,,,\n', ...
                            t, ...
                            getField(parsed, 'mf'), getField(parsed, 'df'), ...
                            getField(parsed, 'fc'), getField(parsed, 'fm'), ...
                            getField(parsed, 'car'), getField(parsed, 'side1'), ...
                            getField(parsed, 'side2'), getField(parsed, 'fit'), ...
                            getField(parsed, 'auto'), getField(parsed, 'status'));
                    end

                    drawnow limitrate;
                end

                % --- AM 帧 ---
                if startsWith(line, 'MOD_AM')
                    parsed = parseKeyVal(line);
                    if isempty(parsed), continue; end

                    hist.idx = mod(hist.idx, MAX_HIST) + 1;
                    t = toc(tStart);
                    hist.time(hist.idx)  = t;
                    hist.am_ma(hist.idx) = getField(parsed, 'ma');

                    % 更新 AM 子图
                    valid = ~isnan(hist.am_ma);
                    if any(valid)
                        tVals = hist.time(valid);
                        maVals = hist.am_ma(valid);
                        [tSort, idxSort] = sort(tVals);

                        set(hMa, 'XData', tSort, 'YData', maVals(idxSort));
                        xlabel(ax2, sprintf('Time (s)  [fc=%.0f Hz, fm=%.0f Hz, car=%.3f, side=%.3f, status=%d]', ...
                            getField(parsed, 'fc'), getField(parsed, 'fm'), ...
                            getField(parsed, 'car'), getField(parsed, 'side'), ...
                            getField(parsed, 'status')));
                    end

                    % 记录日志
                    if logFid > 0
                        fprintf(logFid, '%.3f,AM,,,,%.1f,%.1f,%.3f,,,,-1,%d,%.3f,%.3f\n', ...
                            t, ...
                            getField(parsed, 'fc'), getField(parsed, 'fm'), ...
                            getField(parsed, 'car'), ...
                            getField(parsed, 'status'), ...
                            getField(parsed, 'ma'), getField(parsed, 'side'));
                    end

                    drawnow limitrate;
                end
            end
        catch ME
            fprintf(2, '[ERR] %s\n', ME.message);
        end
        pause(0.01);
    end

    % ============== 清理 ==============
    if logFid > 0, fclose(logFid); end
    if isvalid(s), fclose(s); delete(s); end
    fprintf('串口已关闭.\n');

    %% ==== 嵌套函数 ====
    function onClose(~, ~)
        if isvalid(s), fclose(s); delete(s); end
        if logFid > 0, fclose(logFid); end
        delete(fig);
    end
end

%% ============== 工具函数 ==============

function map = parseKeyVal(line)
    % 解析 "TYPE,key1=val1,key2=val2,..." 格式的行
    map = containers.Map('KeyType', 'char', 'ValueType', 'double');
    parts = strsplit(line, ',');
    if length(parts) < 2, return; end
    for i = 2:length(parts)
        kv = strsplit(parts{i}, '=');
        if length(kv) == 2
            map(kv{1}) = str2double(kv{2});
        end
    end
end

function v = getField(map, key)
    % 从 containers.Map 安全取字段, 不存在时返回 NaN
    if map.isKey(key)
        v = map(key);
    else
        v = NaN;
    end
end
