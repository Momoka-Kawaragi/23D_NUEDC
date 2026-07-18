%% fft_live_plot.m — STM32H7 FFT频谱实时显示
%  串口格式: FFT_BEGIN,Fs=xxx.xx,N=4096\r\n
%            bin,mag\r\n  × N/2行
%            FFT_END\r\n

function fft_live_plot()
    %% ============ 用户配置区 ============
    comPort   = 'COM10';       % 串口号
    baudRate  = 576000;       % 波特率 (与usart.c UART1一致)
    yMaxAuto  = true;         % true=自动量程, false=固定量程
    yMaxFixed = 3.3;          % 固定量程时的Y轴上限(V)
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
    s.InputBufferSize = 512 * 1024;  % 512KB缓冲，防止丢帧
    flush(s);
    pause(0.1);

    % 状态变量
    frame.buf       = {};       % 当前帧累积行
    frame.capturing = false;    % 是否在帧内
    frame.count     = 0;        % 已接收数据行数
    FS_DEFAULT = 1024390;   % 默认采样率(Hz), F4: 84MHz / (81+1) ≈ 1,024,390
    NFFT       = 1024;     % FFT 点数
    halfN      = 512;      % N/2, 帧头解析时更新
    state.f0   = NaN;      % STM32插值精修的峰值频率(Hz)

    % 创建图形窗口
    fig = figure('Name', 'FFT Spectrum — Real-time', ...
                 'NumberTitle', 'off', ...
                 'CloseRequestFcn', @onClose, ...
                 'KeyPressFcn', @onKeyPress);
    ax = axes('Parent', fig, 'XGrid', 'on', 'YGrid', 'on');
    xlabel(ax, 'Frequency (kHz)'); ylabel(ax, 'Magnitude (V)');
    title(ax, 'Waiting for data...');
    hold(ax, 'on');

    % 初始空曲线
    hLine = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.2);
    hPeak = plot(ax, NaN, NaN, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    txtInfo = text(ax, 0.02, 0.96, '', 'Units', 'normalized', ...
                   'VerticalAlignment', 'top', 'FontName', 'Consolas', ...
                   'FontSize', 9, 'BackgroundColor', [1 1 1 0.7]);

    fprintf('串口 %s 已打开, 波特率 %d, 等待数据...\n', comPort, baudRate);
    diagCount = 0;
    % 主循环
    while isvalid(fig) && ishandle(fig)
        try
            if s.NumBytesAvailable > 0
                raw = readline(s);
                if strlength(raw) == 0, continue; end

                line = strtrim(raw);
                if diagCount < 5
                    fprintf('[RAW] %s\n', raw);
                    diagCount = diagCount + 1;
                end

                % --- 帧头 ---
                if startsWith(line, 'FFT_BEGIN')
                    frame.buf = {};
                    frame.capturing = true;
                    frame.count = 0;
                    % 逐字段解析: Fs=, N=, F0=
                    tFs = regexp(line, 'Fs=([\d.]+)', 'tokens', 'once');
                    tN  = regexp(line, 'N=(\d+)', 'tokens', 'once');
                    tF0 = regexp(line, 'F0=([\d.]+)', 'tokens', 'once');
                    if ~isempty(tFs), FS_DEFAULT    = str2double(tFs{1}); end
                    if ~isempty(tN),  NFFT  = str2double(tN{1}); halfN = NFFT / 2; end
                    if ~isempty(tF0)
                        state.f0 = str2double(tF0{1});
                    else
                        state.f0 = NaN;
                    end
                    continue;
                end

                % --- 帧尾 → 绘图 ---
                if startsWith(line, 'FFT_END')
                    frame.capturing = false;
                    if isempty(frame.buf), continue; end

                    % 解析数据
                    nRows = length(frame.buf);
                    if nRows ~= halfN
                        fprintf(2, '[WARN] 数据行%d ≠ 预期%d\n', nRows, halfN);
                    end
                    nPlot = min(nRows, halfN);

                    freq  = zeros(nPlot, 1);
                    mag   = zeros(nPlot, 1);
                    for i = 1:nPlot
                        parts = strsplit(frame.buf{i}, ',');
                        if length(parts) >= 2
                            freq(i) = str2double(parts{1});
                            mag(i)  = str2double(parts{2});
                        end
                    end
                    % 频率轴 → kHz
                    freq_kHz = freq * FS_DEFAULT / NFFT / 1000;

                    % 更新曲线
                    set(hLine, 'XData', freq_kHz, 'YData', mag);

                    % 找峰值 (忽略DC bin0)
                    [peakVal, idx] = max(mag(2:end));
                    if ~isempty(peakVal) && peakVal > 0
                        set(hPeak, 'XData', freq_kHz(idx+1), 'YData', peakVal);
                        set(hPeak, 'Visible', 'on');
                    else
                        set(hPeak, 'Visible', 'off');
                    end

                    % 频率显示: 优先用STM32抛物线插值精修的F0
                    if ~isnan(state.f0)
                        f0_kHz = state.f0 / 1000;
                        f0_str = sprintf('%.3f kHz (Rife-Hanning)', f0_kHz);
                    else
                        f0_str = sprintf('%.3f kHz (bin peak)', freq_kHz(idx+1));
                    end

                    % Y轴范围
                    if yMaxAuto
                        ylim(ax, [0, max(mag(2:end)) * 1.15 + 1e-6]);
                    else
                        ylim(ax, [0, yMaxFixed]);
                    end
                    xlim(ax, [0, freq_kHz(end)]);

                    % 信息文本
                    set(txtInfo, 'String', sprintf(...
                        'Fs: %.1f Hz | N: %d | Peak: bin=%d  %.4f V  %s', ...
                        FS_DEFAULT, NFFT, freq(idx+1), peakVal, f0_str));

                    title(ax, sprintf('FFT Spectrum (Fs=%.1f Hz, N=%d)', FS_DEFAULT, NFFT));
                    drawnow limitrate;
                    continue;
                end

                % --- 数据行: "bin,mag" ---
                if frame.capturing
                    frame.buf{end+1} = line;  %#ok<AGROW>
                    frame.count = frame.count + 1;
                end

            else
                pause(0.01);  % 无数据时短暂休眠
            end
        catch ME
            fprintf(2, '[ERR] %s\n', ME.message);
            pause(0.5);
        end
    end

    %% --- 关闭回调 ---
    function onClose(~, ~)
        try  %#ok<TRYNC>
            fclose(s); delete(s);
        end
        delete(fig);
    end

    %% --- 按键回调 ---
    function onKeyPress(~, evt)
        switch lower(evt.Key)
            case 'a'  % 切换自动/固定量程
                yMaxAuto = ~yMaxAuto;
                fprintf('Y轴自动量程: %s\n', mat2str(yMaxAuto));
            case 'q'  % 退出
                onClose([], []);
        end
    end
end
