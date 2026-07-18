%% dft_phase_plot.m — STM32H7 DFT谐波相频曲线
%  格式: DFT_BEGIN,Fs=xxx\r\n
%        谐波号,频率Hz,幅值V,相位deg\r\n
%        DFT_END\r\n

function dft_phase_plot()
    %% ============ 用户配置区 ============
    comPort  = 'COM10';
    baudRate = 115200;
    relToFund = true;        % true=相对基波相位(基波归零), false=绝对相位
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
    state.fs = 64000;

    fig = figure('Name', 'DFT Harmonic Phase', ...
                 'NumberTitle', 'off', ...
                 'CloseRequestFcn', @onClose);
    ax = axes('Parent', fig, 'XGrid', 'on', 'YGrid', 'on');
    xlabel(ax, 'Frequency (kHz)');
    ylabel(ax, 'Phase (deg)');
    title(ax, 'Waiting for data...');
    hold(ax, 'on');

    hLine = plot(ax, NaN, NaN, 'ro-', 'LineWidth', 1.5, ...
                 'MarkerSize', 8, 'MarkerFaceColor', 'r');
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

            if ~frame.capturing && ~startsWith(line, 'DFT_BEGIN')
                continue;
            end

            if startsWith(line, 'DFT_BEGIN')
                frame.buf = {};
                frame.capturing = true;
                tok = regexp(line, 'Fs=([\d.]+)', 'tokens');
                if ~isempty(tok)
                    state.fs = str2double(tok{1}{1});
                end
                continue;
            end

            if startsWith(line, 'DFT_END')
                frame.capturing = false;
                if isempty(frame.buf), continue; end

                n = length(frame.buf);
                hnum  = zeros(n, 1);
                freq  = zeros(n, 1);
                mag   = zeros(n, 1);
                phase = zeros(n, 1);

                for i = 1:n
                    parts = strsplit(frame.buf{i}, ',');
                    if length(parts) >= 4
                        hnum(i)  = str2double(parts{1});
                        freq(i)  = str2double(parts{2});
                        mag(i)   = str2double(parts{3});
                        phase(i) = str2double(parts{4});
                    end
                end

                % 以基波为0°参考: 所有谐波相位减基波相位
                if relToFund && n >= 1 && hnum(1) == 1
                    phase = phase - phase(1);
                    phase = mod(phase + 180, 360) - 180;
                end

                freq_kHz = freq / 1000;

                set(hLine, 'XData', freq_kHz, 'YData', phase);
                ylim(ax, [min(phase)-10, max(phase)+10]);
                xlim(ax, [0, freq_kHz(end) * 1.1]);

                % 标注谐波号
                delete(findobj(ax, 'Tag', 'hlabel'));
                for i = 1:n
                    text(ax, freq_kHz(i), phase(i) + 5, ...
                         sprintf('h=%d', hnum(i)), ...
                         'FontSize', 7, 'HorizontalAlignment', 'center', ...
                         'Tag', 'hlabel');
                end

                fund_freq = (n >= 1) * freq(1);
                set(txtInfo, 'String', sprintf(...
                    'Fund: %.1f Hz | Harmonics: %d | Ref-to-Fund: %s', ...
                    fund_freq, n, mat2str(relToFund)));

                title(ax, sprintf('DFT Harmonic Phase (Fund=%.1f Hz)', fund_freq));
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
