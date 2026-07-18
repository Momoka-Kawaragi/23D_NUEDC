# DFT 谐波分析驱动使用说明 (STM32H7)

> @author POLO_HU
> @version V1.2
> @date 2026-06-07

## 1. 功能概述

基于单频 DFT 的 THD（总谐波失真）测量驱动。先用粗 FFT 找基波频率，再用单频 DFT 精确计算 1~10 次谐波幅值，最后算 THD 并通过串口输出百分比结果。

| 特性 | 说明 |
|------|------|
| 基波检测 | 4096点 FFT + 抛物线插值精修，精度 < 1 Hz |
| 谐波计算 | 单频 DFT，一次遍历 1~10 次谐波，三角递推加速 |
| 归一化 | Hanning窗相干增益补偿 (4/N)，输出实际电压幅值 (V) |
| 串口输出 | `THD:6.12%` 百分比格式，简洁直观 |
| 自包含 | 驱动内部管理 ADC DMA、TIM3 时钟、数据快照 |

## 2. 数据流

```
TIM3 TRGO @ 用户指定的采样率
  → ADC1 CH4 (PC4)
  → DMA → dft_adc_buf[4096]
  → HAL_ADC_ConvCpltCallback: memcpy → dft_snapshot
  → DFT_Process() → DFT_ProcessFrame(snapshot)
      ├─ 去直流 → 转电压
      ├─ 4096点 CFFT → 峰值搜索 → 抛物线插值 → 基波频率
      ├─ 单频 DFT: H1~H10 谐波幅值 (加Hanning窗)
      └─ THD = sqrt(Σ mag²[h=2..10]) / mag[1]
  → DFT_SendHarmonicFrame()
      └─ 串口发送: "THD:6.12%\r\n"
```

## 3. 外部接口

```c
// ========================================
// 初始化 (含ADC/TIM3/DMA硬件配置，只调一次)
// ========================================
void DFT_App_Init(
    float32_t sampling_rate,   // 采样率 (Hz), 如 1024000.0f
    float32_t ref_voltage,     // ADC参考电压 (V), 如 3.3f
    float32_t search_min_hz,   // 基波搜索下限 (Hz), 0=使用默认1Hz
    float32_t search_max_hz    // 基波搜索上限 (Hz), 0=使用默认256kHz
);

// ========================================
// 主循环调用 (两个一组)
// ========================================
void DFT_Process(void);             // 检测新帧 → 计算THD
void DFT_SendHarmonicFrame(void);   // 串口输出 "THD:6.12%"

// ========================================
// 获取计算结果 (在 DFT_Process 之后可读)
// ========================================
float32_t DFT_GetFundFreq(void);          // 基波频率 (Hz)
float32_t DFT_GetTHD(void);               // THD (0~1, 即 0~100%)
float32_t DFT_GetHarmonicMag(uint32_t h); // 第h次谐波幅值 (V), h=1~10
float32_t DFT_GetHarmonicPhase(uint32_t h); // 第h次谐波相位 (°)

// ========================================
// 兼容接口: 直接传外部ADC数据处理
// ========================================
void DFT_ProcessFrame(const uint16_t *adc_data, uint32_t length);

// ========================================
// 诊断: FFT峰值检测原始数据
// ========================================
void DFT_DiagGetPeak(uint32_t *bin, float32_t *y0, float32_t *y1, float32_t *y2);
```

## 4. 如何接入工程 (三步)

### 第一步: CMakeLists.txt 切换源文件

打开项目根目录的 `CMakeLists.txt`，找到 `target_sources` 区域，**注释掉 Phase.c，取消注释 DFT.c**：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ... 其他驱动 ...
    # MY_DRIVE/MY_FFT/Phase.c    # ← 注释掉 (FFT模式)
    MY_DRIVE/MY_FFT/DFT.c       # ← 取消注释 (DFT模式)
    MY_DRIVE/MY_FFT/WindowFunction.c
    # ... 其他驱动 ...
)
```

> **互斥原因**: Phase.c 和 DFT.c 都定义了 `HAL_ADC_ConvCpltCallback`，同时编译会链接冲突。
> 切回 FFT 模式时反过来操作即可。

### 第二步: main.c 修改头文件和调用

```c
/* ── 头文件 ── */
// #include "Phase.h"     ← 注释掉
#include "DFT.h"          // ← 替换为 DFT

/* ── 初始化 (USER CODE BEGIN 2 区域) ── */
ZPN_UART_Init();
DFT_App_Init(
    1024000.0f,   // 采样率 1.024 MHz
    3.3f,         // ADC 参考电压 3.3V
    800.0f,       // 基波搜索下限 800 Hz (按实际信号调整)
    1200.0f       // 基波搜索上限 1200 Hz (按实际信号调整)
);

/* ── 主循环 (USER CODE BEGIN 3 区域) ── */
while (1)
{
    DFT_Process();             // 检测新ADC帧 → 自动计算THD
    DFT_SendHarmonicFrame();   // 串口发送 "THD:6.12%"
}
```

### 第三步: 编译烧录

```bash
# CLion 中直接 Build，或命令行:
cd build && cmake --build .
```

编译后烧录，打开串口助手 (115200 baud) 即可看到 THD 输出。

## 5. 串口输出格式

每计算完一帧，串口输出一行：

```
THD:6.12%
THD:5.87%
THD:6.03%
...
```

- THD 值为百分比，如 `6.12%` 表示总谐波失真 6.12%
- 更新频率取决于采样率和 FFT 长度，约 0.25 秒一帧 (Fs=1MHz, N=4096)

## 6. DFT_App_Init 参数详解

```c
DFT_App_Init(sampling_rate, ref_voltage, search_min_hz, search_max_hz);
```

| 参数 | 类型 | 说明 | 示例 |
|------|------|------|------|
| `sampling_rate` | float | 目标采样率 (Hz)，驱动会回算实际值 | `1024000.0f` |
| `ref_voltage` | float | ADC 满量程参考电压 (V) | `3.3f` |
| `search_min_hz` | float | 基波搜索下限 (Hz)，排除低频噪声 | `800.0f` |
| `search_max_hz` | float | 基波搜索上限 (Hz)，排除高频干扰 | `1200.0f` |

> **search_min/max 设置技巧**: 设为基波频率的 ±20% 范围。例如基波 1000Hz → 设 `800~1200`。
> 传 `0` 则使用默认值 (1Hz ~ 256kHz)。

## 7. 宏配置 (DFT.h)

| 宏 | 默认值 | 说明 |
|------|--------|------|
| `DFT_CLOCK_SOURCE` | `DFT_CLK_INTERNAL` | 时钟源选择: `DFT_CLK_INTERNAL`=APB1内部时钟, `DFT_CLK_EXTERNAL`=Si5351外部时钟 |
| `DFT_MAX_HARMONIC` | 10 | THD 计算包含的最高谐波次数 |
| `DFT_FFT_LENGTH` | 4096 | FFT 点数 (影响频率分辨率: Δf = Fs/N) |
| `DFT_SEARCH_MIN_HZ` | 1.0 | 基波搜索默认下限 (可在 Init 中覆盖) |
| `DFT_SEARCH_MAX_HZ` | 256000.0 | 基波搜索默认上限 (可在 Init 中覆盖) |

## 8. 采样率配置

### 内部时钟模式 (DFT_CLK_INTERNAL)

TIM3 使用 APB1 定时器时钟 (240 MHz)：

```
TIM3 Period = 240,000,000 / 采样率
实际采样率 = 240,000,000 / Period
```

| 传入采样率 | TIM3 Period | 实际采样率 | 频率分辨率 |
|------|------|------|------|
| 1024000 | 234 | 1,025,641 Hz | 250.4 Hz |
| 512000 | 469 | 512,196 Hz | 125.1 Hz |
| 256000 | 938 | 256,196 Hz | 62.6 Hz |

### 外部时钟模式 (DFT_CLK_EXTERNAL)

TIM3 使用 Si5351 CLK0 (1.024 MHz) 外部时钟：

```
TIM3 Period = 1,024,000 / 采样率
实际采样率 = 1,024,000 / Period
```

| 传入采样率 | TIM3 Period | 实际采样率 |
|------|------|------|
| 512000 | 2 | 512,000 Hz |
| 256000 | 4 | 256,000 Hz |
| 64000 | 16 | 64,000 Hz |

## 9. 算法原理

### 9.1 基波检测

1. 对 4096 点 ADC 数据去直流、加 Hanning 窗
2. 执行 4096 点 FFT (CMSIS DSP `arm_cfft_f32`)
3. 在搜索范围内找最大幅值 bin
4. 抛物线插值精修: 3 点拟合 → 亚 bin 精度 (< 1 Hz)

### 9.2 谐波 DFT

对基波频率 f0 的整数倍频率直接计算 DFT：

```
X[h] = Σ(n=0..N-1) x[n] · win[n] · e^(-j·2π·h·f0·n/fs)
```

- 使用三角递推 (`cos(hθ)`, `sin(hθ)`) 避免重复计算，一次遍历完成 1~10 次谐波
- 加 Hanning 窗抑制旁瓣泄漏
- 幅值归一化: `4.0/N` (补偿 Hanning 窗相干增益 0.5 和单边谱系数 2)

### 9.3 THD 计算

```
THD = sqrt(Σ mag²[h=2..10]) / mag[1]
```

- 输出为 0~1 的比值，串口显示时乘 100 转为百分比
- 谐波幅值 < 5mV 的跳过 (噪声门限)

## 10. THD 理论参考值

| 波形 | THD |
|------|------|
| 纯正弦波 | ≈ 0% (实测 0.5~2%，来自ADC噪声) |
| 三角波 | ≈ 12.1% |
| 方波 | ≈ 48.3% |
| 锯齿波 | ≈ 80.3% |

## 11. 注意事项

1. **信号输入**: PC4 (ADC1 通道 4)，信号幅度 0~3.3V，建议加 1.65V 直流偏置
2. **与 Phase.c 互斥**: 两个驱动都占用 ADC1 DMA 和 `HAL_ADC_ConvCpltCallback`，同一时间只能编译一个
3. **搜索范围**: `search_min_hz` 和 `search_max_hz` 应包围基波频率，过大会误抓噪声
4. **采样率 vs 基波频率**: 建议 N/f0 ≥ 10 个周期，即 4096 点 / 1000 Hz ≈ 4 个完整周期以上
5. **DFT_CLOCK_SOURCE**: 保持 `DFT_CLK_INTERNAL` 除非硬件上 PD2 已连接 Si5351 CLK0

## 12. 完整接入示例 (main.c)

```c
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "DFT.h"            // ← DFT 头文件
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
  MX_TIM3_Init();

  ZPN_UART_Init();

  // 初始化 DFT: 1MHz采样, 3.3V参考, 基波搜索 800~1200Hz
  DFT_App_Init(1024000.0f, 3.3f, 800.0f, 1200.0f);

  while (1)
  {
    DFT_Process();             // 检测新帧 → 计算THD
    DFT_SendHarmonicFrame();   // 串口输出 "THD:6.12%"
  }
}
```

## 13. CMakeLists.txt 完整配置

```cmake
# 添加源文件 (DFT模式)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # MY_DRIVE/MY_FFT/Phase.c    # ← FFT模式: 取消注释这行, 注释下一行
    MY_DRIVE/MY_FFT/DFT.c       # ← DFT模式: 取消注释这行, 注释上一行
    MY_DRIVE/MY_FFT/WindowFunction.c
)

# 头文件路径 (Phase和DFT共用, 不需要改)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT
)
```

切换回 FFT 模式时，反过来注释即可：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_FFT/Phase.c     # ← 取消注释
    # MY_DRIVE/MY_FFT/DFT.c     # ← 注释掉
    MY_DRIVE/MY_FFT/WindowFunction.c
)
```
