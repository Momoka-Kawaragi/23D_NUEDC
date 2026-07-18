%% fft_phase_plot.m — STM32H7 FFT相频曲线实时显示
%  解析PHASE帧, -1000标记自动转为NaN
%  stem图 + 相对相位(以峰值为参考) + 自适应横轴

function fft_phase_plot()
    %% ============ 用户配置区 ============
    comPort  = 'COM10';       % 串口号
    baudRate = 576000;       % 波特率
    xZoom    = 6;            % 横轴自适应时保留的倍数(峰值附近 ±xZoom倍的主瓣宽度)
    %% ====================================

    old = instrfind('Port', comPort);
    if ~isempty(old), fclose(old); delete(old); end

    try
        s = serialport(comPort, baudRate, 'Timeout', 5);
    catch ME
        error('Cannot open %s: %s', comPort, ME.message);
    end
    configureTerminator(s, "CR/LF");
    s.InputBufferSize = 512 * 1024;
    flush(s);
    pause(0.1);

    frame.buf = {};
    frame.capturing = false;
    state.fs    = 64000;
    state.nfft  = 4096;
    state.halfN = 2048;
    state.f0    = 0;

    fig = figure('Name', 'FFT Phase Spectrum', ...
                 'NumberTitle', 'off', ...
                 'CloseRequestFcn', @onClose, ...
                 'KeyPressFcn', @onKeyPress);
    ax = axes('Parent', fig, 'XGrid', 'on', 'YGrid', 'on');
    xlabel(ax, 'Frequency (kHz)');
    ylabel(ax, 'Phase (deg)');
    title(ax, 'Waiting for data...');
    hold(ax, 'on');

    hStem = stem(ax, NaN, NaN, 'b', 'LineWidth', 1.5, 'MarkerSize', 4);
    txtInfo = text(ax, 0.02, 0.96, '', 'Units', 'normalized', ...
                   'VerticalAlignment', 'top', 'FontName', 'Consolas', ...
                   'FontSize', 9, 'BackgroundColor', [1 1 1 0.7]);

    while isvalid(fig) && ishandle(fig)
        try
            if s.NumBytesAvailable == 0
                pause(0.01);
                continue;
            end

            raw = readline(s);
            if strlength(raw) == 0, continue; end
            line = strtrim(raw);

            if ~frame.capturing && ~startsWith(line, 'PHASE_BEGIN')
                continue;
            end

            if startsWith(line, 'PHASE_BEGIN')
                frame.buf = {};
                frame.capturing = true;
                tok = regexp(line, 'Fs=([\d.]+),N=(\d+)', 'tokens');
                if ~isempty(tok)
                    state.fs    = str2double(tok{1}{1});
                    state.nfft  = str2double(tok{1}{2});
                    state.halfN = state.nfft / 2;
                end
                tok2 = regexp(line, 'F0=([\d.]+)', 'tokens');
                if ~isempty(tok2)
                    state.f0 = str2double(tok2{1}{1});
                end
                continue;
            end

            if startsWith(line, 'PHASE_END')
                frame.capturing = false;
                if isempty(frame.buf), continue; end

                nPlot = min(length(frame.buf), state.halfN);
                freq  = zeros(nPlot, 1);
                phase = zeros(nPlot, 1);
                for i = 1:nPlot
                    parts = strsplit(frame.buf{i}, ',');
                    if length(parts) >= 2
                        freq(i)  = str2double(parts{1});
                        phase(i) = str2double(parts{2});
                        if phase(i) < -900, phase(i) = NaN; end
                    end
                end

                validMask = ~isnan(phase);
                freq_valid  = freq(validMask);
                phase_valid = phase(validMask);
                nValid = length(phase_valid);

                freq_kHz_all = freq * state.fs / state.nfft / 1000;
                freq_kHz_valid = freq_valid * state.fs / state.nfft / 1000;

                % ── 固件已做谐波相对相位 (φ[h] - h·φ[1])，直接使用 ──
                phase_disp = phase_valid;
                if nValid > 0
                    peakBin = freq_valid(1);
                    peakPh  = phase_valid(1);
                    peakFreq_kHz = freq_kHz_valid(1);
                end

                % ── 绘图 ──
                delete(hStem);
                if nValid > 0
                    hStem = stem(ax, freq_kHz_valid, phase_disp, ...
                                 'b', 'LineWidth', 1.5, 'MarkerSize', 4);
                else
                    hStem = stem(ax, NaN, NaN, 'b', 'LineWidth', 1.5, 'MarkerSize', 4);
                end

                % ── Y轴 ──
                if nValid > 0
                    yMin = min(phase_disp) - 10;
                    yMax = max(phase_disp) + 10;
                    if yMax - yMin < 10  % 太窄就撑开
                        yMin = yMin - 20; yMax = yMax + 20;
                    end
                    ylim(ax, [yMin, yMax]);
                else
                    ylim(ax, [-190, 190]);
                end

                % ── 自适应横轴 ──
                if nValid > 0
                    % Hanning窗主瓣宽度 ≈ 4个bin, 左右各留 xZoom 倍
                    halfSpan_kHz = xZoom * 2 * (state.fs / state.nfft / 1000);
                    xCenter_kHz = peakFreq_kHz;
                    xLimLo = max(0, xCenter_kHz - halfSpan_kHz);
                    xLimHi = min(freq_kHz_all(end), xCenter_kHz + halfSpan_kHz);
                    % 如果有多簇, 扩展覆盖所有有效bin
                    fMin = min(freq_kHz_valid); fMax = max(freq_kHz_valid);
                    if fMin < xLimLo, xLimLo = max(0, fMin - halfSpan_kHz*0.5); end
                    if fMax > xLimHi, xLimHi = min(freq_kHz_all(end), fMax + halfSpan_kHz*0.5); end
                    xlim(ax, [xLimLo, xLimHi]);
                else
                    xlim(ax, [0, freq_kHz_all(end)]);
                end

                % ── 信息 ──
                if nValid > 0
                    % 统计谐波簇数(连续非NaN段数)
                    gaps = diff(freq_valid) > 3;
                    nClusters = 1 + sum(gaps);
                    if state.f0 > 0
                        set(txtInfo, 'String', sprintf( ...
                            'Fs: %.1f Hz | F0: %.1f Hz | N: %d\nValid bins: %d/%d | Clusters: %d', ...
                            state.fs, state.f0, state.nfft, nValid, nPlot, nClusters));
                    else
                        set(txtInfo, 'String', sprintf( ...
                            'Fs: %.1f Hz | N: %d | Valid bins: %d/%d | Clusters: %d', ...
                            state.fs, state.nfft, nValid, nPlot, nClusters));
                    end
                else
                    set(txtInfo, 'String', sprintf( ...
                        'Fs: %.1f Hz | N: %d | Valid bins: 0/%d (阈值未通过)', ...
                        state.fs, state.nfft, nPlot));
                end

                if state.f0 > 0
                    title(ax, sprintf('Phase Spectrum (F0=%.1f Hz, ref=fundamental)', state.f0));
                else
                    title(ax, 'Phase Spectrum (referenced to peak)');
                end
                drawnow limitrate;
                continue;
            end

            if frame.capturing
                frame.buf{end+1} = line;
            end

        catch ME
            fprintf(2, '[ERR] %s\n', ME.message);
            pause(0.5);
        end
    end

    function onClose(~, ~)
        try fclose(s); delete(s); end
        delete(fig);
    end

    function onKeyPress(~, evt)
        switch lower(evt.Key)
            case 'q'
                onClose([], []);
        end
    end
end
