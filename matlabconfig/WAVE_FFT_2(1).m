%% 采集脚本 - 纯读取屏幕波形（BYTE格式，不改变示波器任何设置）
%  用法：先在示波器上调好时基/垂直/触发 → 再按 a 读取
%  推荐：5 ms/div（70ms窗口），1000Hz 可看到 ~70 周期，采样率够用

clear; close all; clc;

% ==================== 用户可调参数 ====================
FUND_FREQ   = 1000;       % 预期基频 Hz（用于谐波搜索范围）
SOURCE_CHAN = 'CHAN1';    % 采集通道
% =====================================================

scopeAddr = 'USB0::0x1AB1::0x04B0::DS2D264601334::INSTR';

%% 连接示波器
scope = [];
try
    fprintf('正在连接示波器...\n');
    scope = visa('ni', scopeAddr);
    scope.Timeout = 5;              % 将超时时间从 30s 缩短（通常查询响应很快）
    scope.InputBufferSize = 1000000; % 增加到 1MB，防止大数据量截断
    fopen(scope);
    idn = strtrim(query(scope, '*IDN?'));
    fprintf('✅ 成功连接: %s\n', idn);
catch err
    fprintf('❌ 连接失败: %s\n', err.message);
    return;
end

%% ========== 主循环 ==========
fprintf('\n========================================\n');
fprintf('  ⚠️  ！！本脚本不改变示波器任何设置！！\n');
fprintf('  请先在示波器上手动调好时基和垂直档位\n');
fprintf('  推荐：5 ms/div，使1000Hz信号屏幕上约35~70周期\n');
fprintf('========================================\n');
fprintf('  通道: %s | 预期基频: %d Hz | 格式: BYTE\n', SOURCE_CHAN, FUND_FREQ);
fprintf('  按 "a" 读取当前屏幕波形并计算 THD\n');
fprintf('  按 "q" 退出\n');
fprintf('========================================\n');

params = struct('f0', FUND_FREQ, 'chan', SOURCE_CHAN, 'scope', scope);

hFig = figure('KeyPressFcn', @keyPressCallback, ...
    'Visible', 'off', 'Name', '示波器控制台', ...
    'NumberTitle', 'off', 'UserData', params);

uiwait(hFig);

fprintf('程序退出...\n');
try fclose(scope); end
try delete(scope); end


%% ========== 键盘回调 ==========
function keyPressCallback(src, event)
    ud = src.UserData;
    if strcmp(event.Key, 'a')
        fprintf('\n>>> 读取当前屏幕波形...\n');
        acquireAndPlot(ud.scope, ud.chan, ud.f0);
    elseif strcmp(event.Key, 'q')
        uiresume(src);
    end
end


%% ========== 纯读取波形（不发任何设置命令）==========
function [voltage, time, fs, N] = readScreenWaveform(scope_obj, chan)
    % ★ 不碰 :STOP :RUN :TIMebase 等任何设置 ★

    % 1. 选通道
    fprintf(scope_obj, [':WAVeform:SOURce ', chan]);

    % 2. 确保是 BYTE 格式（只设格式，不设点数/时基）
    fprintf(scope_obj, ':WAVeform:FORMat BYTE');

    % 3. 读前导 → 获取当前屏幕的实际参数
    pre_str = query(scope_obj, ':WAVeform:PREamble?');
    if isempty(pre_str)
        error('无法读取 Preamble，请检查示波器连接。');
    end
    pre = sscanf(pre_str, '%f,');
    
    if length(pre) < 10
        error('Preamble 格式错误或读取不完整，请尝试增大 Timeout。');
    end
    
    N     = round(pre(3));    % 实际点数
    x_inc = pre(5);           % 采样间隔 (s)
    x_org = pre(6);           % 起点时间 (s)
    y_inc = pre(8);           % V/ADC count
    y_org = pre(9);           % 偏置 (V)
    y_ref = pre(10);          % BYTE 参考电平

    % 4. binblockread 读字节
    fprintf(scope_obj, ':WAVeform:DATA?');
    raw_bytes = binblockread(scope_obj, 'uint8');

    % 5. BYTE → 电压
    voltage = (double(raw_bytes(:)) - y_ref) * y_inc + y_org;

    if length(voltage) ~= N
        fprintf('   ⚠️ preamble=%d 实际=%d → 以实际为准\n', N, length(voltage));
        N = length(voltage);
    end

    fs   = 1 / x_inc;
    time = x_org + (0:N-1)' * x_inc;
end


%% ========== 采集 & FFT THD ==========
function acquireAndPlot(scope_obj, chan, f0_expected)
    try
        [voltage, time, fs, N] = readScreenWaveform(scope_obj, chan);
        time_ms = time * 1000;
        T_total = N / fs;

        fprintf('   点数: %d | 采样率: %.3f kSa/s\n', N, fs/1e3);
        fprintf('   窗口: %.3f ms | 分辨率: %.2f Hz\n', T_total*1e3, fs/N);
        fprintf('   每周期约 %.1f 点 (%.0f Hz 信号)\n', fs/f0_expected, f0_expected);
        fprintf('   Vmin=%.4f V, Vmax=%.4f V, Vpp=%.4f V, Mean=%.4f V\n', ...
            min(voltage), max(voltage), max(voltage)-min(voltage), mean(voltage));

        if N < 100
            warning('点数太少 (%d)，请增大示波器时基或存储深度。', N);
            return;
        end

        % ===== 时域图 =====
        figure('Name', sprintf('%s 屏幕波形 (%d点)', chan, N), 'NumberTitle','off');
        plot(time_ms, voltage, 'b-', 'LineWidth', 0.8);
        xlabel('时间 (ms)'); ylabel('电压 (V)');
        title(sprintf('%s 屏幕波形 — %d 点 | %.2f ms | %.2f kSa/s', ...
            chan, N, T_total*1e3, fs/1e3));
        grid on;

        % ===== FFT THD =====
        v_ac = voltage(:) - mean(voltage);
        if max(abs(v_ac)) < 1e-6
            warning('信号太弱（Vpp≈0），跳过 THD。');
            return;
        end

        win   = hann(N);
        v_win = v_ac .* win;
        cg    = sum(win) / N;

        Y      = fft(v_win);
        Y_half = Y(1:floor(N/2)+1);
        P_raw  = abs(Y_half / N);
        P1     = P_raw / cg;
        P1(2:end-1) = 2 * P1(2:end-1);
        f_axis = (0:floor(N/2))' * fs / N;

        % 基频搜索
        f_res  = fs / N;
        f_lo   = f0_expected * 0.7;   % 放宽到 ±30%
        f_hi   = f0_expected * 1.3;
        idx_range = find(f_axis >= f_lo & f_axis <= f_hi);

        if isempty(idx_range)
            fprintf('   ⚠️  FFT 分辨率: %.2f Hz\n', f_res);
            fprintf('   ⚠️  搜索范围: %.0f ~ %.0f Hz 内无谱峰\n', f_lo, f_hi);
            fprintf('   💡 建议：增大示波器时基（如 5 ms/div）以获更多周期\n');
            fprintf('   💡 或者调整 FUND_FREQ 变量匹配实际信号频率\n');

            % 不报错，还是画出频谱让用户看看
            figure('Name', 'FFT 频谱（未找到基频）', 'NumberTitle','off');
            f_max = f0_expected * 6;
            mask  = f_axis <= f_max;
            plot(f_axis(mask)/1000, P1(mask), 'b-', 'LineWidth', 1);
            xlabel('频率 (kHz)'); ylabel('幅度 (V)');
            title(sprintf('FFT 频谱 (%d 点 | 分辨率 %.2f Hz | 基频未找到)', N, f_res));
            grid on; xlim([0, f_max/1000]);
            return;
        end

        [fund_amp, idx_peak] = max(P1(idx_range));
        fund_freq = f_axis(idx_range(idx_peak));

        fprintf('\n   ===== FFT THD 结果 =====\n');
        fprintf('   基频: %.1f Hz  幅度: %.5f V (分辨率: %.2f Hz)\n', ...
            fund_freq, fund_amp, f_res);

        harm_amps = zeros(4,1);
        for k = 1:4
            f_tgt = (k+1) * fund_freq;
            [~, idx] = min(abs(f_axis - f_tgt));
            harm_amps(k) = P1(idx);
            fprintf('   %d 次 (%6.1f Hz): %.5f V\n', k+1, f_axis(idx), harm_amps(k));
        end

        THD = sqrt(sum((harm_amps/sqrt(2)).^2)) / (fund_amp/sqrt(2)) * 100;
        fprintf('   ★ THD (2~5次) = %.4f %%\n', THD);
        fprintf('   =========================\n\n');

        % ===== 频谱图 =====
        figure('Name', 'FFT 频谱', 'NumberTitle','off');
        f_max = f0_expected * 6;
        mask  = f_axis <= f_max;
        plot(f_axis(mask)/1000, P1(mask), 'b-', 'LineWidth', 1); hold on;
        stem(fund_freq/1000, fund_amp, 'go', 'MarkerSize',8,'LineWidth',2);
        stem((2:5)'*fund_freq/1000, harm_amps, 'ro', 'MarkerSize',8,'LineWidth',2);
        xlabel('频率 (kHz)'); ylabel('幅度 (V)');
        title(sprintf('FFT 频谱 (%d 点 | THD = %.4f%%)', N, THD));
        legend('频谱','基波','谐波','Location','best');
        grid on; xlim([0, f_max/1000]);

    catch err
        fprintf(2, '❌ 采集出错: %s (行 %d)\n', err.message, err.stack(1).line);
    end
end