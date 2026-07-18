%% fft_waveform_plot.m — STM32H7 FFT→IFFT重构波形实时显示
%  串口格式: WAVE_BEGIN,Fs=xxx.xx,N=4096\r\n
%            index,voltage\r\n  * N行
%            WAVE_END\r\n

function fft_waveform_plot()
    %% ============ 用户配置区 ============
    comPort  = 'COM10';       % 串口号
    baudRate = 115200;       % 波特率
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
    state.fs   = 64000;
    state.nfft = 4096;

    fig = figure('Name', 'FFT→IFFT Reconstructed Waveform', ...
                 'NumberTitle', 'off', ...
                 'CloseRequestFcn', @onClose);
    ax = axes('Parent', fig, 'XGrid', 'on', 'YGrid', 'on');
    xlabel(ax, 'Time (ms)');
    ylabel(ax, 'Voltage (V)');
    title(ax, 'Waiting for data...');
    hold(ax, 'on');

    hLine = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.2);
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

            % 跳过幅值/相位帧, 只处理WAVE帧
            if startsWith(line, 'FFT_BEGIN')
                skipUntilEnd(s, 'FFT_END');
                continue;
            end
            if startsWith(line, 'PHASE_BEGIN')
                skipUntilEnd(s, 'PHASE_END');
                continue;
            end

            if startsWith(line, 'WAVE_BEGIN')
                frame.buf = {};
                frame.capturing = true;
                tok = regexp(line, 'Fs=([\d.]+),N=(\d+)', 'tokens');
                if ~isempty(tok)
                    state.fs   = str2double(tok{1}{1});
                    state.nfft = str2double(tok{1}{2});
                end
                continue;
            end

            if startsWith(line, 'WAVE_END')
                frame.capturing = false;
                if isempty(frame.buf), continue; end

                nPlot = min(length(frame.buf), state.nfft);
                t = zeros(nPlot, 1);
                v = zeros(nPlot, 1);
                for i = 1:nPlot
                    parts = strsplit(frame.buf{i}, ',');
                    if length(parts) >= 2
                        t(i) = str2double(parts{1});
                        v(i) = str2double(parts{2});
                    end
                end
                t_ms = t / state.fs * 1000;  % 采样点 → 毫秒

                set(hLine, 'XData', t_ms, 'YData', v);
                xlim(ax, [0, t_ms(end)]);

                vpp = max(v) - min(v);
                vrms = rms(v);
                set(txtInfo, 'String', sprintf(...
                    'Fs: %.1f Hz | N: %d | Vpp: %.3f V | Vrms: %.3f V', ...
                    state.fs, state.nfft, vpp, vrms));

                title(ax, sprintf('FFT→IFFT Waveform (Fs=%.1f Hz, N=%d)', ...
                      state.fs, state.nfft));
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
end

function skipUntilEnd(s, endMarker)
    t0 = tic;
    while toc(t0) < 2.0
        if s.NumBytesAvailable == 0
            pause(0.005);
            continue;
        end
        raw = readline(s);
        if strlength(raw) == 0, continue; end
        if startsWith(strtrim(raw), endMarker)
            return;
        end
    end
end
