%% 自动扫频、数据采集与误差分析脚本
%
% 版本: V5.3.2 - 恢复标准误差计算公式
%
% 功能描述:
% 1. 控制 RIGOL DG1062 信号源进行扫频 (100Hz-3kHz)。
% 2. 控制 RIGOL 示波器在每个频率点测量输出信号的峰峰值 (Vpp)。
% 3. 根据用户提供的传递函数 H(s) 计算理论上的 Vpp 值。
% 4. 绘制实测曲线、理论曲线，并在每个实测点上标注其与理论值的百分比误差。
%
% 使用前请务必填写您的设备 VISA 地址。
clear; clc; close all;

%% 1. 参数设置
% --- 设备 VISA 地址 (请务必修改为您的实际地址) ---
sigGenAddr = 'USB0::0xF4EC::0x1103::SDG1XCAX4R1040::0::INSTR'; % 信号发生器 (RIGOL DG1062)
scopeAddr  = 'USB0::0xF4ED::0xEE3A::SDS1EDEX4R2763::0::INSTR'; % 示波器

% --- 扫频参数 ---
startFreq  = 100;     % 起始频率 (Hz)
stopFreq   = 4000;    % 终止频率 (Hz)
stepFreq   = 100;     % 步进频率 (Hz)
amplitude  = 1.5;     % 峰峰值电压 (Vpp)
offset     = 0.0;     % 直流偏置 (V)

% --- 初始化 VISA 设备对象 ---
sigGen = [];
scope  = [];

%% 2. 定义理论模型 (传递函数)
% 预测波形部分已按要求暂时注释，保留频率点列表和占位变量。
% 如果以后需要恢复预测，请取消下面被注释的代码并移除占位赋值。
%
% % 根据您提供的图片 H(s) = 5 / (10^-8*s^2 + 3e-4*s + 1)
% s = tf('s');
% H_s = 5 / (1e-8 * s^2 + 3e-4 * s + 1);
%
% % 生成频率点列表
freqList = startFreq:stepFreq:stopFreq;
%
% % 计算理论响应
% % bode 函数会计算在指定频率点上的幅度和相位
% % 我们需要将频率从 Hz 转换为 rad/s
% [mag, ~] = bode(H_s, freqList * 2 * pi);
% % mag 是线性增益，不是dB。我们需要将其从 3D 数组中 squeeze 出来
% mag = squeeze(mag); 
% % 理论输出 Vpp = 输入 Vpp * 幅度增益
% theoreticalVpp = amplitude * mag;

% 由于预测波形（理论值）已被注释，创建占位 NaN 向量以避免后续引用报错
theoreticalVpp = NaN(size(freqList));

%% 3. 建立设备连接与初始化
try
    % --- 3.1 连接信号发生器 ---
    fprintf('正在连接信号源...\n');
    sigGen = visa('ni', sigGenAddr);
    sigGen.Timeout = 15;
    fopen(sigGen);
    fprintf('✅ 已连接信号源: %s\n', query(sigGen, '*IDN?'));
    
    % --- 3.2 连接示波器 ---
    fprintf('正在连接示波器...\n');
    scope = visa('ni', scopeAddr);
    scope.Timeout = 15;
    fopen(scope);
    fprintf('✅ 已连接示波器: %s\n', query(scope, '*IDN?'));
    
    % --- 3.3 仪器初始化 ---
    fprintf('正在初始化仪器设置...\n');
    % 示波器自动设置以适应初始信号（等待操作完成以提高可靠性）
    fprintf(scope, ':AUTOSCALE');
    fprintf('等待示波器自动调整...');
    tStart = tic;
    autoscaleTimeout = 10; % 最大等待时间 (秒)
    while toc(tStart) < autoscaleTimeout
        try
            resp = strtrim(query(scope, '*OPC?'));
            if strcmp(resp, '1')
                break;
            end
        catch
            % 忽略查询错误，稍后重试
        end
        pause(0.2);
    end
    fprintf(' 完成。\n');
    
    % 设置信号源 CH1 输出 (Siglent SDG 系列推荐指令)
    % C1:OUTP ON 开启通道1
    % C1:OUTP LOAD,HZ 设置高阻抗匹配 (防止电压显示翻倍)
    fprintf(sigGen, 'C1:OUTP ON'); 
    fprintf(sigGen, 'C1:OUTP LOAD,HZ'); 
    
    % 设置示波器 CH1
    fprintf(scope, ':CHAN1:DISP ON');
    fprintf(scope, ':TRIG:EDGE:SOUR CHAN1'); % 设置触发源为通道1
    fprintf(scope, ':CHAN1:COUP DC');   % 显式设置为DC耦合
    
    % 【重要】检查并设置探头衰减比
    % 如果您的物理探头拨到了 1X，这里设为 1
    % 如果您的物理探头拨到了 10X，这里设为 10
    % 如果设反了，读数会差 10 倍
    % 这里默认设为 1X (PROB 1)
    fprintf(scope, ':CHAN1:PROB 1');    
    
    % 开启 Vpp 测量项，使其出现在屏幕上
    fprintf(scope, 'PACU PKPK,C1');
catch err
    fprintf('❌ 设备连接或初始化失败: %s\n', err.message);
    % 出错时确保资源被释放
    if ~isempty(sigGen), fclose(sigGen); delete(sigGen); end
    if ~isempty(scope), fclose(scope); delete(scope); end
    clear sigGen scope;
    return; % 提前退出脚本
end

%% 4. 执行扫频与测量
fprintf('\n=== 开始扫频测量 ===\n');
% 在产生初始频率前先输出一个测试频率 (初始频率的一半)，并让示波器进行一次自动调整
testFreq = startFreq / 2;
fprintf('输出测试频率 %.2f Hz (初始频率一半) 以便示波器预设...\n', testFreq);
try
    cmdTest = sprintf('C1:BSWV WVTP,SINE,FRQ,%.2f,AMP,%.3f,OFST,%.3f', testFreq, amplitude, offset);
    fprintf(sigGen, cmdTest);
    fprintf(sigGen, 'C1:OUTP ON');
    % 调整示波器时基以显示大约 1 个周期（每屏 10 格）
    timeScaleTest = (1 / testFreq) * 0.1;
    fprintf(scope, sprintf(':TIM:SCAL %e', timeScaleTest));
    pause(0.8);
    % 对示波器执行一次自动调整，确保接下来读数稳定（等待完成）
    fprintf(scope, ':AUTOSCALE');
    tStart = tic;
    autoscaleTimeout = 5;
    while toc(tStart) < autoscaleTimeout
        try
            resp = strtrim(query(scope, '*OPC?'));
            if strcmp(resp, '1')
                break;
            end
        catch
        end
        pause(0.2);
    end
    fprintf('测试频率设置并自动调整完成，准备进入正式测量。\n');
catch
    fprintf('警告: 设置测试频率或示波器自动调整失败，继续执行正式测量。\n');
end

measuredVpp = zeros(size(freqList)); % 预分配数组以存储测量结果
for i = 1:length(freqList)
    freq = freqList(i);
    
    % --- 4.1 设置信号源 ---
    % Siglent SDG 系列推荐使用 C1:BSWV (Basic Wave) 指令
    % 参数: WVTP(波形类型), FRQ(频率), AMP(幅度Vpp), OFST(偏置)
    cmd = sprintf('C1:BSWV WVTP,SINE,FRQ,%.2f,AMP,%.3f,OFST,%.3f', freq, amplitude, offset);
    fprintf(sigGen, cmd);
    fprintf(sigGen, 'C1:OUTP ON'); % 确保每次循环都开启输出
    
    % --- [新增] 动态调整示波器时基和垂直档位 ---
    % 1. 调整时基: 设定屏幕显示约 1 个周期（每屏 10 格）
    timeScale = (1 / freq) * 0.1; 
    fprintf(scope, sprintf(':TIM:SCAL %e', timeScale));
    
    % 等待信号源波形输出稳定 (1秒)
    pause(1.0); 

    % 2. 调整垂直档位 (Scale)
    % 每次更改频率后，让示波器执行一次自动调整，并等待其完成以保证稳定
    fprintf(scope, ':AUTOSCALE');
    tStart = tic;
    autoscaleTimeout = 5; % 等待上限 (秒)
    while toc(tStart) < autoscaleTimeout
        try
            resp = strtrim(query(scope, '*OPC?'));
            if strcmp(resp, '1')
                break;
            end
        catch
        end
        pause(0.2);
    end

    % Autoscale 之后，重新设置时基以确保显示期望的周期数量
    fprintf(scope, sprintf(':TIM:SCAL %e', timeScale));
    pause(0.5);
    
    % --- 4.2 示波器测量 ---
    % Siglent SDS 系列标准 SCPI 指令
    vppVal = NaN;
    maxRetries = 3;
    
    for retry = 1:maxRetries
        try
            % 尝试读取测量值
            % 步骤 A: 确保测量源是通道 1 (这很重要)
            fprintf(scope, ':MEAS:SOUR CHAN1');
            
            % 步骤 B: 使用 C1:PAVA? PKPK 查询 (Siglent 最常用的指令)
            fprintf(scope, 'C1:PAVA? PKPK');
            
            % 读取并清理返回值
            rawStr = strtrim(fscanf(scope));
            
            % [调试] 强制打印每次读取到的原始字符串，以便诊断问题
            fprintf('  [调试] 频率 %.0f Hz | Raw: "%s" | ', freq, rawStr);
            
            % 解析数值
            % 典型返回: "C1:PAVA PKPK,5.00E-01V"
            % 或者: "PKPK,5.00E-01V"
            
            % 1. 尝试提取逗号后的部分
            if contains(rawStr, ',')
                parts = strsplit(rawStr, ',');
                valPart = parts{end}; % 取最后一部分
            else
                valPart = rawStr;
            end
            
            % 2. 使用正则提取其中的浮点数
            numStr = regexp(valPart, '[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?', 'match', 'once');
            
            if ~isempty(numStr)
                vppVal = str2double(numStr);
                fprintf('Parsed: %.4f V\n', vppVal);
            else
                fprintf('Parsed: NaN\n');
            end
            
            % 验证数值合理性
            if ~isnan(vppVal) && abs(vppVal) < 1000
                break; % 成功
            end
        catch
            fprintf('  [调试] 读取异常\n');
        end
        
        pause(0.5); % 重试间隔
    end
    
    % 检查是否是有效的数值 (示波器在未捕捉到稳定波形时可能返回 >1e30 的值)
    if vppVal > 1e30
        vppVal = NaN; % 标记为无效数据
        fprintf('频率 %.0f Hz -> 测量无效\n', freq);
    else
        measuredVpp(i) = vppVal;
        fprintf('频率 %.0f Hz -> 实测 Vpp = %.4f V\n', freq, vppVal);
    end
end
fprintf('=== 扫频测量结束 ===\n\n');
% --- 输出最简格式的频率与响应数据 ---
% 格式 (CSV 风格): freq(Hz), measuredVpp(V) [, theoreticalVpp(V)]
fprintf('freq(Hz),measuredVpp(V)');
if ~all(isnan(theoreticalVpp))
    fprintf(',theoreticalVpp(V)');
end
fprintf('\n');
for idx = 1:numel(freqList)
    if ~all(isnan(theoreticalVpp))
        fprintf('%.0f,%.6f,%.6f\n', freqList(idx), measuredVpp(idx), theoreticalVpp(idx));
    else
        fprintf('%.0f,%.6f\n', freqList(idx), measuredVpp(idx));
    end
end
fprintf('--- end of data ---\n\n');

%% 5. 数据处理与绘图
fprintf('正在计算误差并生成图表...\n');
% 询问用户是否生成理论幅频曲线，并接收滤波器参数
try
    doTheo = input('是否计算理论幅频曲线？输入 1 计算，0 跳过（默认 0）：');
catch
    doTheo = 0;
end
if isempty(doTheo), doTheo = 0; end
if doTheo
    ftype = lower(strtrim(input('滤波器类型 (low/high/bandpass/bandstop)：','s')));
    nOrder = input('阶数 (正整数)：');
    fc_user = input('截止/中心频率 fc (Hz)：');
    Q_user = input('Q 因子 (带通/带阻 使用；低/高通可输入 0)：');
    omega = freqList * 2 * pi;
    theoreticalVpp = NaN(size(freqList));
    try
        switch ftype
            case {'low','lowpass'}
                Wn = 2*pi*fc_user;
                try
                    [b,a] = butter(nOrder, Wn, 's');
                catch
                    [b,a] = butter(nOrder, Wn, 'low', 's');
                end
            case {'high','highpass'}
                Wn = 2*pi*fc_user;
                try
                    [b,a] = butter(nOrder, Wn, 'high', 's');
                catch
                    [b,a] = butter(nOrder, Wn, 's');
                end
            case {'bandpass','bp','band'}
                bw = fc_user / max(Q_user, eps);
                w1 = 2*pi*max(fc_user - bw/2, eps);
                w2 = 2*pi*(fc_user + bw/2);
                Wn = [w1 w2];
                try
                    [b,a] = butter(nOrder, Wn, 'bandpass', 's');
                catch
                    [b,a] = butter(nOrder, Wn, 's');
                end
            case {'bandstop','bs','notch','bandreject'}
                bw = fc_user / max(Q_user, eps);
                w1 = 2*pi*max(fc_user - bw/2, eps);
                w2 = 2*pi*(fc_user + bw/2);
                Wn = [w1 w2];
                try
                    [b,a] = butter(nOrder, Wn, 'stop', 's');
                catch
                    [b,a] = butter(nOrder, Wn, 'bandstop', 's');
                end
            otherwise
                error('未知滤波器类型');
        end
        Hfreq = freqs(b,a,omega);
        theoreticalVpp = amplitude * abs(Hfreq);
    catch err
        fprintf('生成理论曲线失败: %s\n', err.message);
        theoreticalVpp = NaN(size(freqList));
    end
else
    theoreticalVpp = NaN(size(freqList));
end
% 【修正】计算误差百分比: (实测值 - 理论值) / 理论值 * 100%
% 如果理论值已被注释，则不计算误差，全部设为 NaN
if all(isnan(theoreticalVpp))
    errorPercentage = NaN(size(measuredVpp));
else
    errorPercentage = ((measuredVpp - theoreticalVpp) ./ theoreticalVpp) * 100;
end

% --- 5.1 绘制幅频特性对比图并在图上标注误差 ---
figure('Name', '幅频特性分析 (含误差标注)', 'NumberTitle', 'off', 'Position', [100, 100, 900, 600]);
plot(freqList, measuredVpp, 'b-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
hold on;
% 如果存在理论曲线则绘制
if ~all(isnan(theoreticalVpp))
    plot(freqList, theoreticalVpp, 'r--', 'LineWidth', 1.5);
end

% --- 5.2 在每个测量点上标注误差百分比 ---
for i = 1:length(freqList)
    % 仅为有效数据点添加标注
    if ~isnan(errorPercentage(i))
        % 在数据点的右上方一点的位置显示文本
        text(freqList(i) * 1.02, measuredVpp(i), ...
             sprintf('%.2f%%', errorPercentage(i)), ...
             'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', ...
             'FontSize', 8, 'Color', 'm');
    end
end

grid on;
title('实测 幅频特性曲线 (带误差标注)');
xlabel('频率 (Hz)');
ylabel('峰峰值电压 (Vpp)');
if ~all(isnan(theoreticalVpp))
    legend('实测值', '理论值', 'Location', 'best');
else
    legend('实测值', 'Location', 'best');
end
hold off;

fprintf('图表生成完毕。\n');

%% 6. 释放资源
fprintf('正在关闭设备连接...\n');
if ~isempty(sigGen)
    fprintf(sigGen, ':OUTP1 OFF'); % 关闭信号源输出
    fclose(sigGen);
    delete(sigGen);
end
if ~isempty(scope)
    fclose(scope);
    delete(scope);
end
clear sigGen scope;
fprintf('脚本执行完毕。\n');
