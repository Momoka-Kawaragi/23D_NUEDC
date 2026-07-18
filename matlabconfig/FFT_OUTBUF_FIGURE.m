%% FFT_OUTBUF_FIGURE.m — DFT FFT_OutputBuf 频谱实时显示
%  串口格式: FFT_BEGIN,Fs=xxx.xx,N=4096,F0=xxx.x\r\n
%            bin,val\r\n  × N/2行
%            FFT_END\r\n

function FFT_OUTBUF_FIGURE()
    %% ============ 用户配置 ============
    comPort   = 'COM10';       % 串口号
    baudRate  = 230400;       % 波特率
    yMaxAuto  = true;         % true=自动量程, false=固定量程
    yMaxFixed = 3.3;          % 固定量程时Y轴上限(V)
    MOD_WIN   = 10;           % ma 窗口帧数 (与STM32端MOD_WIN_FRAMES一致)
    %% ==================================

    old = instrfind('Port', comPort);
    if ~isempty(old), fclose(old); delete(old); end

    try
        s = serialport(comPort, baudRate, 'Timeout', 5);
    catch ME
        error('无法打开串口 %s: %s', comPort, ME.message);
    end
    configureTerminator(s, "CR/LF");
    s.InputBufferSize = 512 * 1024;
    flush(s);
    pause(0.1);

    frame.buf       = {};
    frame.capturing = false;
    FS_DEFAULT = 1024390;
    NFFT       = 4096;
    halfN      = 2048;
    state.f0   = NaN;
    state.thd  = NaN;
    state.type = '---';
    state.mod  = '---';
    state.mod_depth = NaN;
    state.mod_peak  = NaN;
    state.mod_raw   = NaN;
    state.ask_fm    = NaN;
    state.ask_h1    = NaN;
    state.ask_h3    = NaN;
    state.ask_h5    = NaN;
    state.fm_df     = NaN;
    state.fm_fm     = NaN;
    state.fm_left   = NaN;
    state.fm_right  = NaN;

    % ma 滑动窗口
    state.ma_win      = zeros(1, MOD_WIN);
    state.ma_win_idx  = 1;
    state.ma_win_fill = 0;

    fig = figure('Name', 'FFT OutputBuf Spectrum — DFT', ...
                 'NumberTitle', 'off', ...
                 'Position', [100 100 1000 580]);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    hLine  = plot(ax, nan(1,halfN), nan(1,halfN), 'b-', 'LineWidth', 1.2);
    hPeak  = plot(ax, NaN, NaN, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
    hFmL   = plot(ax, NaN, NaN, 'gv', 'MarkerSize', 8, 'LineWidth', 1.5);
    hFmR   = plot(ax, NaN, NaN, 'g^', 'MarkerSize', 8, 'LineWidth', 1.5);
    hTitle = title(ax, '等待数据...');
    xlabel(ax, 'Frequency (Hz)');
    ylabel(ax, 'Magnitude (V)');
    grid(ax, 'on');
    xlim(ax, [0 1]);
    if ~yMaxAuto, ylim(ax, [0 yMaxFixed]); end

    hText = uicontrol('Style', 'text', 'String', 'F0: -- Hz,  Peak: -- V', ...
                      'Units', 'normalized', 'Position', [0.01 0.95 0.65 0.04], ...
                      'HorizontalAlignment', 'left', 'FontSize', 11, ...
                      'BackgroundColor', get(fig, 'Color'));

    % ma 窗口柱状图 (右侧小图)
    axMa = axes('Parent', fig, 'Units', 'normalized', ...
                'Position', [0.72 0.02 0.26 0.22]);
    hMaBar = bar(axMa, 1:MOD_WIN, zeros(1, MOD_WIN), 'FaceColor', [0.2 0.6 1.0]);
    hold(axMa, 'on');
    hMaMax = bar(axMa, 1, 0, 'FaceColor', [1.0 0.3 0.3]);  % 最高值用红色
    ylim(axMa, [0 100]);
    xlim(axMa, [0.5 MOD_WIN + 0.5]);
    xlabel(axMa, 'Frame', 'FontSize', 8);
    ylabel(axMa, 'ma (%)', 'FontSize', 8);
    title(axMa, 'AM ma Window', 'FontSize', 9);
    grid(axMa, 'on');

    % ma 窗口文字
    hMaText = uicontrol('Style', 'text', 'String', '', ...
                        'Units', 'normalized', 'Position', [0.72 0.24 0.26 0.06], ...
                        'HorizontalAlignment', 'center', 'FontSize', 9, ...
                        'BackgroundColor', get(fig, 'Color'));

    % 按空格键暂停/继续
    state.paused = false;
    set(fig, 'KeyPressFcn', @(src, evt) togglePause(src, evt));
    function togglePause(~, evt)
        if strcmp(evt.Key, 'space')
            state.paused = ~state.paused;
        end
    end

    disp('等待串口数据... (空格暂停, Ctrl+C 停止)');

    try
        while isvalid(fig) && isvalid(s)
            if state.paused
                set(hTitle, 'String', '■ PAUSED — 按空格继续');
                drawnow;
                pause(0.1);
                continue;
            end

            line = readline(s);
            line = strtrim(line);

            if startsWith(line, 'FFT_BEGIN')
                % 解析帧头: FFT_BEGIN,Fs=xxx.xx,N=4096,F0=xxx.x
                tokens = regexp(line, 'FFT_BEGIN,Fs=(\d+)\.(\d+),N=(\d+),F0=(\-?\d+)\.(\d+)', 'tokens');
                if ~isempty(tokens)
                    t = tokens{1};
                    fs_val = str2double(t{1}) + str2double(t{2})/100;
                    NFFT   = str2double(t{3});
                    halfN  = NFFT / 2;
                    state.f0 = str2double(t{4}) + str2double(t{5})/10;
                end
                frame.buf = cell(halfN, 1);
                frame.capturing = true;
                frame.count = 0;

            elseif startsWith(line, 'FFT_END')
                if frame.capturing && frame.count == halfN
                    % 解析数据
                    spectrum = zeros(halfN, 1);
                    for k = 1:halfN
                        parts = regexp(frame.buf{k}, '(\-?\d+),(\-?\d+)\.(\d+)', 'tokens');
                        if ~isempty(parts)
                            p = parts{1};
                            spectrum(k) = str2double(p{2}) + str2double(p{3})/1e6;
                            if startsWith(p{1}, '-'), spectrum(k) = -spectrum(k); end
                        end
                    end

                    % 频率轴
                    freq = (0:halfN-1) * fs_val / NFFT;

                    % 更新曲线
                    set(hLine, 'XData', freq, 'YData', spectrum);

                    % 标记峰值
                    [peak_val, peak_idx] = max(spectrum(2:end));
                    peak_idx = peak_idx + 1;
                    set(hPeak, 'XData', freq(peak_idx), 'YData', peak_val);
                    set(hTitle, 'String', sprintf('FFT OutputBuf Spectrum (Fs=%.0f Hz, N=%d)', fs_val, NFFT));

                    % MOD 信息字符串
                    if strcmp(state.mod, 'FM') && ~isnan(state.fm_df)
                        if ~isnan(state.fm_fm)
                            modStr = sprintf('FM  df=%.1f Hz  fm=%.1f Hz  [bin %d-%d]', ...
                                state.fm_df, state.fm_fm, state.fm_left, state.fm_right);
                        else
                            modStr = sprintf('FM  df=%.1f Hz  [bin %d-%d]', ...
                                state.fm_df, state.fm_left, state.fm_right);
                        end
                        % 标注左右边界谱线
                        if ~isnan(state.fm_left)
                            set(hFmL, 'XData', freq(state.fm_left+1), ...
                                'YData', spectrum(state.fm_left+1), 'Visible', 'on');
                        end
                        if ~isnan(state.fm_right)
                            set(hFmR, 'XData', freq(state.fm_right+1), ...
                                'YData', spectrum(state.fm_right+1), 'Visible', 'on');
                        end
                    elseif strcmp(state.mod, 'ASK') && ~isnan(state.ask_fm)
                        modStr = sprintf('ASK fm=%dHz  H1=%.0fmV H3=%.0fmV H5=%.0fmV', ...
                            state.ask_fm, state.ask_h1*1000, state.ask_h3*1000, state.ask_h5*1000);
                        set(hFmL, 'Visible', 'off'); set(hFmR, 'Visible', 'off');
                    elseif isnan(state.mod_depth)
                        set(hFmL, 'Visible', 'off'); set(hFmR, 'Visible', 'off');
                        modStr = sprintf('%s', state.mod);
                    elseif isnan(state.mod_raw)
                        set(hFmL, 'Visible', 'off'); set(hFmR, 'Visible', 'off');
                        modStr = sprintf('%s  %.1f%%', state.mod, state.mod_depth);
                    elseif isnan(state.mod_peak)
                        set(hFmL, 'Visible', 'off'); set(hFmR, 'Visible', 'off');
                        modStr = sprintf('%s  out=%.1f%%  cur=%.1f%%', state.mod, state.mod_depth, state.mod_raw);
                    else
                        set(hFmL, 'Visible', 'off'); set(hFmR, 'Visible', 'off');
                        modStr = sprintf('%s  out=%.1f%%  pk=%.1f%%  cur=%.1f%%', state.mod, state.mod_depth, state.mod_peak, state.mod_raw);
                    end
                    set(hText, 'String', sprintf('F0: %.1f Hz | Type: %s | %s', ...
                                 state.f0, state.type, modStr));

                    % 更新 ma 窗口柱状图
                    updateMaWindow();

                    if yMaxAuto
                        ymax = max(spectrum(2:end)) * 1.2;
                        if ymax < 0.01, ymax = 0.01; end
                        ylim(ax, [0 ymax]);
                    end
                    xlim(ax, [0 freq(end)]);
                    drawnow;
                end
                frame.capturing = false;

            elseif startsWith(line, 'THD:')
                t = regexp(line, 'THD:(\-?\d+)\.(\d+)%', 'tokens');
                if ~isempty(t), state.thd = str2double(t{1}{1}) + str2double(t{1}{2})/100; end

            elseif startsWith(line, 'HARM:')
                t = regexp(line, 'HARM:', 'split');
                if numel(t) > 1, state.harmLine = t{2}; end

            elseif startsWith(line, 'TYPE:')
                state.type = strtrim(extractAfter(line, 'TYPE:'));

            elseif startsWith(line, 'MOD:')
                t = strtrim(extractAfter(line, 'MOD:'));
                parts = split(t, ',');
                state.mod = parts{1};
                if strcmp(state.mod, 'FM')
                    if numel(parts) > 4
                        state.fm_df    = str2double(parts{2});
                        state.fm_fm    = str2double(parts{3});
                        state.fm_left  = str2double(parts{4});
                        state.fm_right = str2double(parts{5});
                    elseif numel(parts) > 3
                        state.fm_df    = str2double(parts{2});
                        state.fm_fm    = NaN;
                        state.fm_left  = str2double(parts{3});
                        state.fm_right = str2double(parts{4});
                    elseif numel(parts) > 1
                        state.fm_df    = str2double(parts{2});
                        state.fm_left  = NaN;
                        state.fm_right = NaN;
                    else
                        state.fm_df    = NaN;
                        state.fm_left  = NaN;
                        state.fm_right = NaN;
                    end
                    state.mod_depth = NaN;
                    state.mod_peak  = NaN;
                    state.mod_raw   = NaN;
                elseif numel(parts) > 3
                    state.mod_depth = str2double(parts{2});
                    state.mod_peak  = str2double(parts{3});
                    state.mod_raw   = str2double(parts{4});
                    % 推入滑动窗口 (用 raw)
                    if ~isnan(state.mod_raw)
                        state.ma_win(state.ma_win_idx) = state.mod_raw;
                        state.ma_win_idx = mod(state.ma_win_idx, MOD_WIN) + 1;
                        if state.ma_win_fill < MOD_WIN, state.ma_win_fill = state.ma_win_fill + 1; end
                    end
                elseif numel(parts) > 2
                    state.mod_depth = str2double(parts{2});
                    state.mod_peak  = str2double(parts{3});
                    state.mod_raw   = NaN;
                elseif numel(parts) > 1
                    state.mod_depth = str2double(parts{2});
                    state.mod_peak  = NaN;
                    state.mod_raw   = NaN;
                else
                    state.mod_depth = NaN;
                    state.mod_peak  = NaN;
                    state.mod_raw   = NaN;
                end

            elseif startsWith(line, 'ASK:')
                t = regexp(line, 'ASK:fm=(\d+),H1=(\d+)mV,H3=(\d+)mV,H5=(\d+)mV', 'tokens');
                if ~isempty(t)
                    state.ask_fm = str2double(t{1}{1});
                    state.ask_h1 = str2double(t{1}{2}) / 1000;
                    state.ask_h3 = str2double(t{1}{3}) / 1000;
                    state.ask_h5 = str2double(t{1}{4}) / 1000;
                end

            elseif frame.capturing && frame.count < halfN
                frame.count = frame.count + 1;
                frame.buf{frame.count} = line;
            end
        end
    catch ME
        if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
            warning('错误: %s', ME.message);
        end
    end

    if isvalid(s), fclose(s); delete(s); end
    if isvalid(fig), close(fig); end
    disp('已停止。');

    %% ========== 更新 ma 窗口柱状图 ==========
    function updateMaWindow()
        if state.ma_win_fill == 0, return; end

        n = state.ma_win_fill;
        vals = state.ma_win(1:n);

        [maxVal, maxIdx] = max(vals);

        % 更新蓝色柱 (全量)
        set(hMaBar, 'XData', 1:n, 'YData', vals);

        % 更新红色柱 (仅最高值位置, 其余置 NaN)
        redVals = nan(1, n);
        redVals(maxIdx) = maxVal;
        set(hMaMax, 'XData', 1:n, 'YData', redVals);

        % 文字: 窗口内各值 + 输出值
        valStr = sprintf('%5.1f  ', vals);
        outStr = '';
        if ~isnan(state.mod_depth)
            outStr = sprintf('out=%.1f%%  ', state.mod_depth);
        end
        if ~isnan(state.mod_peak)
            outStr = [outStr sprintf('pk=%.1f%%  ', state.mod_peak)];
        end
        set(hMaText, 'String', sprintf('%spk=%.1f%%  [ %s]', outStr, maxVal, strtrim(valStr)));

        ylim(axMa, [0 max(maxVal * 1.3, 10)]);
    end
end
