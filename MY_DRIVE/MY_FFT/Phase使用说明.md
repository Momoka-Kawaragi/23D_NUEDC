# Phase FFT 频谱分析驱动使用说明 (STM32H7)

> @author POLO_HU
> @version V2.3
> @date 2026-06-07

## 1. 功能概述

基于 FFT 的频谱分析与双通道相位差测量驱动，支持 Rife-Hanning 亚 bin 频率精修、跨功率谱法相位差计算、FFT→IFFT 波形重构。

| 功能 | 说明 |
|------|------|
| 单通道 FFT 频谱分析 | ADC1 + TIM3 TRGO 触发，DMA NORMAL 模式采集 4096 点，Hanning 窗 |
| Rife-Hanning 频率精修 | Hanning 窗频谱解析解 → 亚 bin 精度 (~0.1 Hz) |
| Rife 幅值修正 | 回收 Hanning 窗泄漏能量，峰值幅值更准确 |
| 双通道相位差测量 | ADC1(PC4) + ADC2(PC5) 同步采集，跨功率谱法 → PDIFF 帧 |
| FFT→IFFT 波形重构 | 频谱数据 IFFT 还原时域波形 → WAVE 帧输出 |
| 幅值归一化 | 输出实际电压值 (V)，补偿 CFFT 缩放 + 窗函数相干增益 |
| 模拟信号调试 | `FFT_TEST_SIMULATION` 宏启用内部双音信号，无需外部信号源 |

---

## 2. 数据流

```
TIM3 TRGO @ PHASE_TARGET_FS (默认 1.024 MHz)
    → ADC1 CH4 (PC4) + ADC2 CH8 (PC5) 同步触发
    → DMA NORMAL 模式 → ADC_Buffer / ADC_Buffer2
    → HAL_ADC_ConvCpltCallback:
        双路完成 → 停TIM3 → memcpy快照 → 同步重启DMA → 开TIM3
        置 g_frame_ready + g_phase_ready
    → main loop:
        ├─ FFT_SendSpectrumFrame()
        │   ├─ Phase_FFT_OneChannel(adc_snapshot)
        │   │   ├─ 去直流 → Hanning窗 → FFT → 幅值归一化(4/N)
        │   │   ├─ 全频谱寻峰 → ±5bin局部精修
        │   │   └─ Rife-Hanning插值 → 频率精修 + 幅值修正
        │   └─ 串口: FFT_BEGIN,F0=xxx → 2048行 bin,mag → FFT_END
        ├─ FFT_SendPhaseDiffFrame()
        │   ├─ Phase_ComputeDiff()
        │   │   ├─ 双路独立FFT (cplx1/cplx2/mag1/mag2 独立缓冲)
        │   │   ├─ Rife-Hanning频率精修 + 幅值修正
        │   │   └─ 跨功率谱法: S12 = CH1 × conj(CH2) → 相位差
        │   └─ 串口: PDIFF,f0,phase,mag1,mag2,raw1,raw2
        └─ FFT_IFFT_SendWaveform() (可选)
            └─ 串口: WAVE_BEGIN → N/4行 idx,V → WAVE_END
```

---

## 3. 关键配置 (Phase.h)

| 宏 | 默认值 | 说明 |
|------|--------|------|
| `PHASE_CLOCK_SOURCE` | `PHASE_CLK_INTERNAL` | 时钟源: `PHASE_CLK_INTERNAL`=APB1(240MHz), `PHASE_CLK_EXTERNAL`=Si5351(1.024MHz) |
| `PHASE_TARGET_FS` | `1024000.0f` | 目标采样率 (Hz) |
| `FFT_LENGTH` | `4096` | FFT 点数 |
| `ADC_EFFECTIVE_BITS` | `16` | ADC 分辨率 |
| `SAMPLING_RATE_DEFAULT` | `64000.0f` | 兜底采样率 (实际由 TIM3 计算) |
| `HARM_BIN_HALF_WIDTH` | `5U` | 基波局部精修搜索半宽 (bin) |
| `PHASE_CAL_OFFSET_DEG` | `-66.5f` | 双 ADC 相位偏置校准值 (°) |
| `CH2_AMP_CAL_SCALE` | `1.000f` | CH2 幅值校准系数 (1.0=不校准) |
| `FFT_TEST_SIMULATION` | (注释) | 取消注释启用内部双音模拟信号 |

---

## 4. 外部接口

### 4.1 `void FFT_App_Init(void)`

初始化 FFT 实例，配置 TIM3 时钟源和采样周期，启动双路 ADC DMA NORMAL 模式采集。在 `main.c` 初始化阶段调用一次。

### 4.2 `void FFT_SendSpectrumFrame(void)`

主循环调用。检测 `g_frame_ready` → 从 ADC 快照计算单通道 FFT + Rife-Hanning 频率精修 → 串口输出全频谱帧。

```
FFT_BEGIN,Fs=1025641.00,N=4096,F0=1000.583\r\n
0,0.000000\r\n
1,0.000123\r\n
...
2047,0.000456\r\n
FFT_END\r\n
```

- `F0`: Rife-Hanning 插值精修的峰值频率 (Hz)
- 每行: `bin索引,幅值(V)`，共 2048 行
- 配合 `fft_live_plot.m` 实时显示

### 4.3 `void FFT_SendPhaseDiffFrame(void)`

主循环调用。检测 `g_phase_ready` → 双通道独立 FFT + 跨功率谱法相位差 → 串口输出。

```
PDIFF,1000.6,-45.32,1.2345,1.1987,raw1=1.2100,raw2=1.1800\r\n
```

| 字段 | 说明 |
|------|------|
| `f0` | Rife-Hanning 精修基波频率 (Hz) |
| `phase_diff` | 相位差 (°), 范围 ±180 |
| `mag1` | CH1 基波幅值 (V), Rife 修正后 |
| `mag2` | CH2 基波幅值 (V), Rife 修正后 |
| `raw1` | CH1 原始 FFT 峰值 (V), Rife 修正前 |
| `raw2` | CH2 原始 FFT 峰值 (V), Rife 修正前 |

### 4.4 `void FFT_IFFT_SendWaveform(void)`

可选。FFT→IFFT 还原时域波形 (不加窗，纯往返验证)，降采样 4:1 发送。

```
WAVE_BEGIN,Fs=1025641.00,N=4096\r\n
0,1.650123\r\n
4,1.648901\r\n
...
WAVE_END\r\n
```

### 4.5 `void FFT_App_Process(void)`

NORMAL 模式下为占位函数，回调已负责数据捕获和 DMA 重启。主循环中保留调用即可。

---

## 5. 如何接入工程 (三步)

### 第一步: CMakeLists.txt 确认源文件

打开项目根目录的 `CMakeLists.txt`，确认 `Phase.c` 已启用 (DFT.c 已注释)：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT/Phase.c       # ← 确认取消注释
    # MY_DRIVE/MY_FFT/DFT.c       # ← 确认已注释
    MY_DRIVE/MY_FFT/WindowFunction.c
)
```

> **互斥原因**: Phase.c 和 DFT.c 都定义 `HAL_ADC_ConvCpltCallback`，同时编译会链接冲突。

### 第二步: main.c 配置调用

```c
/* ── 头文件 ── */
#include "Phase.h"
#include "ZPN_Uart.h"

/* ── 初始化 (USER CODE BEGIN 2) ── */
ZPN_UART_Init();
FFT_App_Init();
UART2_SendPrintf("FFT Spectrum Start\r\n");

/* ── 主循环 (USER CODE BEGIN 3) ── */
while (1)
{
    FFT_App_Process();
    FFT_SendSpectrumFrame();        // 频谱帧 + F0
    // FFT_SendPhaseDiffFrame();    // 相位差帧 (按需启用)
    // FFT_IFFT_SendWaveform();     // 波形重构 (按需启用)
}
```

### 第三步: 编译烧录

```bash
cd build && cmake --build .
```

---

## 6. 采样率配置

修改 `Phase.h` 中的 `PHASE_TARGET_FS` 宏，`FFT_App_Init()` 自动计算 TIM3 周期。

### 内部时钟模式 (PHASE_CLK_INTERNAL)

TIM3 使用 APB1 定时器时钟 (240 MHz)：

| PHASE_TARGET_FS | TIM3 Period | 实际采样率 | 频率分辨率 | 奈奎斯特 |
|------|------|------|------|------|
| 1024000 | 234 | 1,025,641 Hz | 250.4 Hz | 512.8 kHz |
| 512000 | 469 | 512,196 Hz | 125.1 Hz | 256.1 kHz |
| 256000 | 938 | 256,196 Hz | 62.6 Hz | 128.1 kHz |
| 128000 | 1875 | 128,000 Hz | 31.25 Hz | 64.0 kHz |
| 64000 | 3750 | 64,000 Hz | 15.625 Hz | 32.0 kHz |

### 外部时钟模式 (PHASE_CLK_EXTERNAL)

TIM3 使用 Si5351 CLK0 (1.024 MHz) 外部时钟：

| PHASE_TARGET_FS | TIM3 Period | 实际采样率 | 频率分辨率 |
|------|------|------|------|
| 512000 | 2 | 512,000 Hz | 125.0 Hz |
| 256000 | 4 | 256,000 Hz | 62.5 Hz |
| 64000 | 16 | 64,000 Hz | 15.625 Hz |

---

## 7. 算法原理

### 7.1 Rife-Hanning 频率精修

传统 FFT 频率精度受限于 bin 分辨率 (Δf = Fs/N)。Rife-Hanning 插值利用 Hanning 窗频谱的解析解，通过峰值 bin 及其左右邻居的幅值比，计算亚 bin 频偏 δ：

```
若 A[k+1] ≥ A[k-1]:  δ = (2·A[k+1] - A[k]) / (A[k] + A[k+1])
若 A[k+1] <  A[k-1]:  δ = -(2·A[k-1] - A[k]) / (A[k] + A[k-1])

精修频率: f0 = (k + δ) × Fs/N
```

### 7.2 Rife 幅值修正

Hanning 窗会将 off-bin 信号的能量泄漏到旁瓣。Rife 幅值修正回收这部分能量：

```
A_corrected = A[k] × (πδ) / sin(πδ) × (1 - δ²)
```

### 7.3 跨功率谱法相位差

对 CH1 和 CH2 的复数频谱在基波 bin 处计算互功率谱：

```
S12 = CH1[k] × conj(CH2[k])
phase_diff = arg(S12) = atan2(Im(S12), Re(S12))
```

优点：不受各通道独立相位估计误差影响，精度更高。

---

## 8. CubeMX 关键配置

| 外设 | 配置项 | 值 | 说明 |
|------|--------|------|------|
| ADC1 | External Trigger | TIM3 TRGO, Rising Edge | 定时触发 |
| ADC1 | DMA | ONESHOT, HalfWord | NORMAL 模式，回调重启 |
| ADC1 | Channel | CH4 (PC4) | 信号输入 |
| ADC2 | External Trigger | TIM3 TRGO, Rising Edge | 与 ADC1 同步 |
| ADC2 | DMA | ONESHOT, HalfWord | NORMAL 模式，回调重启 |
| ADC2 | Channel | CH8 (PC5) | 信号输入 |
| TIM3 | Prescaler | 0 (÷1) | 不分频 |
| TIM3 | TRGO | Update Event | 每个周期触发 ADC |
| UART1 | Baud Rate | 115200 | 串口波特率 |

> **注意**: CubeMX 可能默认 DMA CIRCULAR 模式。`Phase_StartAcq()` 会在运行时强制覆盖为 NORMAL 模式 + 回调重启，无需手动修改 CubeMX 配置。

---

## 9. 相位校准

双 ADC 同一 TIM3 TRGO 触发，但内部采样延迟不一致 → 固定偏置。校准步骤：

1. PC4 和 PC5 接**同一信号源** (T 型三通分两路)
2. 运行程序，读取 `PDIFF` 中 `phase_diff_deg` 值
3. 修改 `Phase.c` 中 `PHASE_CAL_OFFSET_DEG` 为该值的相反数
4. 当前校准值: `-66.5°`

幅值校准：记录 `PDIFF` 中 `mag1/mag2` 均值，设 `CH2_AMP_CAL_SCALE = mag1 / mag2`。

---

## 10. MATLAB 脚本对照

| 脚本 | 对应帧格式 | 功能 |
|------|------|------|
| `fft_live_plot.m` | FFT_BEGIN / FFT_END | 频谱实时显示，F0 标注 |
| `phase_diff_plot.m` | PDIFF | 相位差实时显示 |
| `fft_waveform_plot.m` | WAVE_BEGIN / WAVE_END | 重构波形显示 |

---

## 11. 注意事项

1. **与 DFT.c 互斥**: 两个驱动都占用 ADC1 DMA 和 `HAL_ADC_ConvCpltCallback`，同一时间只能编译一个
2. **不可在回调中加串口打印**: UART TX DMA 与 ADC DMA 同级优先级会死锁
3. **奈奎斯特**: 输入信号频率 < 采样率/2，否则混叠
4. **幅值单位**: 已归一化到实际电压 (V)，1V 正弦输入 → 峰值约 1.0
5. **FFT_TEST_SIMULATION**: 取消注释 `Phase.h` 中的宏可启用内部双音信号 (bin 50 + bin 120)，无需外部信号源调试

---

## 12. CMake 完整配置

```cmake
# 源文件 (Phase模式)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT/Phase.c       # ← FFT/Phase模式
    # MY_DRIVE/MY_FFT/DFT.c       # ← DFT模式 (与Phase互斥)
    MY_DRIVE/MY_FFT/WindowFunction.c
)

# 头文件路径 (Phase和DFT共用)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT
)
```

---

## 13. 完整接入示例 (main.c)

```c
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Phase.h"
#include "ZPN_Uart.h"
#include <stdio.h>

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();

  ZPN_UART_Init();
  FFT_App_Init();
  UART2_SendPrintf("FFT Spectrum Start\r\n");

  while (1)
  {
    FFT_App_Process();
    FFT_SendSpectrumFrame();        // 频谱帧 + F0
    // FFT_SendPhaseDiffFrame();    // 相位差帧 (按需启用)
    // FFT_IFFT_SendWaveform();     // 波形重构 (按需启用)
  }
}
```
