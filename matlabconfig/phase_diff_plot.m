%% phase_diff_plot.m — STM32H7 双通道相位差实时显示
%  显示: 极坐标相位指针 + 幅值柱状图 + 时间历史(含众数均值)
%  V2.0: 众数滤波替代简单滑动平均 — 自动剔除F0跳动/竞态导致的离群点

function phase_diff_plot()
    %% ============ 用户配置区 ============
    comPort    = 'COM10';
    baudRate   = 460800;
    historyLen = 100;     % 历史数据点数
    modeBufLen = 30;      % 众数滤波器缓冲区大小 (建议 20~50)
    binWidth   = 1.0;     % 众数直方图 bin 宽度(deg)
    %% ====================================

    old = instrfind('Port', comPort);
    if ~isempty(old), fclose(old); delete(old); end

    try
        s = serialport(comPort, baudRate, 'Timeout', 5);
    catch ME
        error('Cannot open %s: %s', comPort, ME.message);
    end
    configureTerminator(s, "CR/LF");
    s.InputBufferSize = 64 * 1024;
    flush(s);
    pause(0.1);

    % 众数滤波器缓冲区
    mode_buf_phase = NaN(modeBufLen, 1);
    mode_buf_f0    = NaN(modeBufLen, 1);
    mode_buf_mag1  = NaN(modeBufLen, 1);
    mode_buf_mag2  = NaN(modeBufLen, 1);
    mode_idx       = 1;

    % 历史缓冲区
    hist_raw      = NaN(historyLen, 1);
    hist_filtered = NaN(historyLen, 1);
    hist_time     = NaN(historyLen, 1);
    hist_idx      = 1;
    t0 = tic;

    % 创建图形
    fig = figure('Name', 'Phase Difference — Dual Channel (Mode Filter)', ...
                 'NumberTitle', 'off', ...
                 'CloseRequestFcn', @onClose, ...
                 'Position', [100, 100, 1000, 700]);

    % ── 左上: 极坐标指针(显示滤波后值) ──
    ax1 = subplot(2, 2, 1, 'Parent', fig);
    hold(ax1, 'on');
    axis(ax1, 'equal');
    xlim(ax1, [-1.5, 1.5]); ylim(ax1, [-1.5, 1.5]);
    th = linspace(0, 2*pi, 200);
    plot(ax1, cos(th), sin(th), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    plot(ax1, [-1.3 1.3], [0 0], 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
    plot(ax1, [0 0], [-1.3 1.3], 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
    for deg = 0:30:330
        r = 1.0;
        plot(ax1, [0.92*r*cosd(deg), r*cosd(deg)], ...
                  [0.92*r*sind(deg), r*sind(deg)], 'k-', 'LineWidth', 0.5);
    end
    hArrow = quiver(ax1, 0, 0, 0, 0, 0, 'b', 'LineWidth', 3, ...
                    'MaxHeadSize', 0.5);
    txtDeg = text(ax1, 0, -0.3, '--°', ...
                  'HorizontalAlignment', 'center', 'FontSize', 28, ...
                  'FontWeight', 'bold', 'Color', [0 0 0.6]);
    title(ax1, 'Phase Difference 0~180° (Mode Filter)');
    set(ax1, 'XTick', [], 'YTick', []);

    % ── 左下: 幅值柱状图 ──
    ax2 = subplot(2, 2, 3, 'Parent', fig);
    title(ax2, 'Channel Magnitudes');
    ylabel(ax2, 'Voltage (V)');
    hBar = bar(ax2, [1, 2], [0, 0], 'FaceColor', [0.2 0.6 1.0]);
    xlim(ax2, [0, 3]);
    set(ax2, 'XTickLabel', {'CH1 (PA0)', 'CH2 (PA5)'});

    % ── 右侧: 时间历史 ──
    ax3 = subplot(2, 2, [2, 4], 'Parent', fig);
    title(ax3, 'Phase Difference History');
    xlabel(ax3, 'Time (s)');
    ylabel(ax3, 'Phase (deg)');
    grid(ax3, 'on');
    hold(ax3, 'on');
    hHistRaw = plot(ax3, NaN, NaN, 'r.', 'MarkerSize', 6, 'DisplayName', 'Raw');
    hHistFlt = plot(ax3, NaN, NaN, 'b-', 'LineWidth', 2.5, ...
                    'DisplayName', sprintf('Mode(bin=%.1f°)', binWidth));
    yline(ax3, 0, 'k--');
    yline(ax3, 90, 'r:');  yline(ax3, -90, 'r:');
    legend(ax3, 'Location', 'northeast');

    % 信息栏(顶部)
    txtInfo = uicontrol('Style', 'text', 'String', 'Waiting for data...', ...
                        'Units', 'normalized', 'Position', [0.01, 0.96, 0.50, 0.04], ...
                        'HorizontalAlignment', 'left', 'FontSize', 10, ...
                        'FontName', 'Consolas', 'BackgroundColor', [1 1 1 0.7]);

    while isvalid(fig) && ishandle(fig)
        try
            if s.NumBytesAvailable == 0
                pause(0.01);
                continue;
            end

            raw = readline(s);
            if strlength(raw) == 0, continue; end
            line = strtrim(raw);

            if ~startsWith(line, 'PDIFF,'), continue; end
            parts = strsplit(line, ',');
            if length(parts) < 5, continue; end

            f0_hz   = str2double(parts{2});
            diff_deg = str2double(parts{3});
            mag1    = str2double(parts{4});
            mag2    = str2double(parts{5});

            if isnan(diff_deg), continue; end

            % 换算到 0~180°
            diff_deg = abs(diff_deg);
            while diff_deg > 180.0, diff_deg = 360.0 - diff_deg; end

            % ── 众数滤波器: 将新值推入环形缓冲 ──
            mode_buf_phase(mode_idx) = diff_deg;
            mode_buf_f0(mode_idx)    = f0_hz;
            mode_buf_mag1(mode_idx)  = mag1;
            mode_buf_mag2(mode_idx)  = mag2;
            mode_idx = mode_idx + 1;
            if mode_idx > modeBufLen, mode_idx = 1; end

            % 滤波输出: 取出现最多的聚集区间的均值
            [filt_phase, filt_f0, filt_mag1, filt_mag2, inliers] = ...
                modeFilter(mode_buf_phase, mode_buf_f0, ...
                           mode_buf_mag1, mode_buf_mag2, binWidth);

            % ── 极坐标: 滤波后相位 ──
            if ~isnan(filt_phase)
                rad_f = filt_phase * pi / 180;
                set(hArrow, 'UData', cos(rad_f), 'VData', sin(rad_f));
                set(txtDeg, 'String', sprintf('%.1f°', filt_phase));
            end

            % ── 更新幅值 (用滤波后的幅值) ──
            if ~isnan(filt_mag1) && ~isnan(filt_mag2)
                set(hBar, 'YData', [filt_mag1, filt_mag2]);
                ylim(ax2, [0, max(filt_mag1, filt_mag2) * 1.3 + 0.001]);
            end

            % ── 更新历史 ──
            t_elapsed = toc(t0);
            hist_raw(hist_idx)      = diff_deg;
            hist_filtered(hist_idx) = filt_phase;
            hist_time(hist_idx)     = t_elapsed;
            hist_idx = hist_idx + 1;
            if hist_idx > historyLen, hist_idx = 1; end

            vmask = ~isnan(hist_raw);
            if sum(vmask) > 1
                tv = hist_time(vmask);
                rv = hist_raw(vmask);
                fv = hist_filtered(vmask);
                [tv_sorted, si] = sort(tv);
                set(hHistRaw, 'XData', tv_sorted, 'YData', rv(si));
                set(hHistFlt, 'XData', tv_sorted, 'YData', fv(si));
                xlim(ax3, [max(0, tv_sorted(end) - 30), tv_sorted(end) + 1]);
            end

            % ── 信息 ──
            nInliers = sum(inliers);
            set(txtInfo, 'String', sprintf(...
                ['F0: %.0f Hz | Phase: %.2f° | ModeFilt: %.2f° (%d/%d inliers) | ' ...
                 'CH1: %.3fV | CH2: %.3fV'], ...
                f0_hz, diff_deg, filt_phase, nInliers, modeBufLen, ...
                filt_mag1, filt_mag2));

            title(ax3, sprintf('DeltaPhase (ModeFilt=%d bins=%.1f°)', modeBufLen, binWidth));
            drawnow limitrate;

        catch ME
            fprintf(2, '[ERR] %s\n', ME.message);
            pause(0.5);
        end
    end

    function onClose(~, ~)
        try fclose(s); delete(s); end %#ok<TRYNC>
        delete(fig);
    end
end

%% ============ 众数滤波器 ============
%  从环形缓冲区中找样本最密集的区间，只取该区间内值的均值
%  自动剔除离群点 (F0误判、竞态撕裂数据)
%
%  输入:
%    phase_buf - 相位差数组 (含NaN占位)
%    f0_buf    - 频率数组
%    mag1_buf  - CH1幅值数组
%    mag2_buf  - CH2幅值数组
%    binWidth  - 直方图 bin 宽度 (deg)
%
%  输出:
%    filt_phase - 众数区间均值相位
%    filt_f0    - 众数区间均值频率
%    filt_mag1  - 众数区间均值 CH1 幅值
%    filt_mag2  - 众数区间均值 CH2 幅值
%    inliers    - 逻辑数组 (哪些样本属于众数区间)
function [filt_phase, filt_f0, filt_mag1, filt_mag2, inliers] = ...
    modeFilter(phase_buf, f0_buf, mag1_buf, mag2_buf, binWidth)

    % 剔除 NaN
    valid = ~isnan(phase_buf);
    if sum(valid) < 3
        % 数据太少，直接用均值
        filt_phase = mean(phase_buf, 'omitnan');
        filt_f0    = mean(f0_buf,    'omitnan');
        filt_mag1  = mean(mag1_buf,  'omitnan');
        filt_mag2  = mean(mag2_buf,  'omitnan');
        inliers    = valid;
        return;
    end

    pv = phase_buf(valid);

    % 构建直方图
    p_min = min(pv);
    p_max = max(pv);
    p_range = p_max - p_min;
    if p_range < binWidth
        % 所有样本都在一个 bin 内
        filt_phase = mean(pv);
        filt_f0    = mean(f0_buf(valid));
        filt_mag1  = mean(mag1_buf(valid));
        filt_mag2  = mean(mag2_buf(valid));
        inliers    = valid;
        return;
    end

    edges = p_min : binWidth : (p_max + binWidth);
    [counts, ~, binIdx] = histcounts(pv, edges);
    if isempty(counts)
        filt_phase = mean(pv);
        filt_f0    = mean(f0_buf(valid));
        filt_mag1  = mean(mag1_buf(valid));
        filt_mag2  = mean(mag2_buf(valid));
        inliers    = valid;
        return;
    end

    % 找最多的 bin
    [~, peakBin] = max(counts);

    % 提取该 bin 内的所有样本
    inBin = (binIdx == peakBin);
    % 映射回原始有效索引
    validIdx = find(valid);
    inlierIdx = validIdx(inBin);

    inliers = false(size(phase_buf));
    inliers(inlierIdx) = true;

    % 只对众数区间内的样本取均值
    filt_phase = mean(phase_buf(inliers));
    filt_f0    = mean(f0_buf(inliers));
    filt_mag1  = mean(mag1_buf(inliers));
    filt_mag2  = mean(mag2_buf(inliers));
end
