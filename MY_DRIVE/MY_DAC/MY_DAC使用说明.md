# MY_DAC 波形发生器

## 概述

使用 DAC1 + TIM6 + DMA 输出模拟波形，支持正弦波和方波谐波合成。

## CubeMX 配置

- **TIM6** → Activated（内部定时器，不用配引脚）
- **DAC1** → OUT1 → Trigger 选 `Timer 6 Trigger Out event`
- **DMA** → DAC1_CH1 → Mode 选 `Circular`，Data Width 选 `Half Word`

## 外部接口

### 正弦波

| 函数 | 说明 |
|------|------|
| `MY_DAC_Sine_StartAuto(freq, vpp, offset)` | **推荐**。自动适配采样点，一键启动正弦波 |
| `MY_DAC_Sine_Init(samples, vpp, offset)` | 手动指定采样点(8~256)生成正弦波表 |
| `MY_DAC_Sine_Start(freq)` | 启动 DAC+DMA+TIM6 输出 |
| `MY_DAC_Sine_SetFrequency(freq)` | 动态调频，不重建波形表 |
| `MY_DAC_Sine_SetAmplitudeOffset(vpp, offset)` | 动态调幅/调偏置 |
| `MY_DAC_Sine_SetSamples(samples)` | 动态修改采样点数并重建波形表 |
| `MY_DAC_Sine_GetFrequency()` | 查询当前实际频率 (Hz) |
| `MY_DAC_Sine_GetTheoreticalMaxFrequency()` | 查询理论最高频率 |
| `MY_DAC_Sine_GetRecommendedMaxFrequency()` | 查询推荐最高频率 |
| `MY_DAC_GetSampleUpdateHz()` | 查询当前 DAC 更新率 (Hz) |

### 方波谐波合成

| 函数 | 说明 |
|------|------|
| `MY_DAC_Square_Harmonic_Init(samples, f1_vpp, f3_vpp, f5_vpp, offset)` | 基波+3次+5次谐波合成方波 |
| `MY_DAC_Square_Harmonic_11_Init(samples, harmonic_vpps[6], offset)` | 基波+3/5/7/9/11次谐波合成方波，`harmonic_vpps` 数组依次为 f1/f3/f5/f7/f9/f11 的 Vpp |

> 谐波合成函数只生成波形表，启动输出仍需调用 `MY_DAC_Sine_Start(freq)`。

## 使用示例

```c
#include "my_dac.h"

// 正弦波：1kHz, 2Vpp, 偏置1.65V
MY_DAC_Sine_StartAuto(1000.0f, 2.0f, 1.65f);

// 方波谐波合成：1kHz
float h[6] = {2.0f, 0.67f, 0.4f, 0.29f, 0.22f, 0.18f};
MY_DAC_Square_Harmonic_11_Init(128, h, 1.65f);
MY_DAC_Sine_Start(1000.0f);

// 动态调频
MY_DAC_Sine_SetFrequency(2000.0f);
```

## 参数说明

- `freq` — 频率 (Hz)，实用范围 1Hz ~ 500kHz
- `vpp` — 峰峰值 (V)，最大 3.3V
- `offset` — 直流偏置 (V)，(offset  vpp/2) 需在 0~3.3V 内
- `samples` — 每周期采样点，范围 8~256

## 输出引脚

**PA4** (DAC1_OUT1)

## 注意事项

- 输出缓冲带宽约 100kHz，超过 50kHz 正弦波会明显衰减
- 关掉输出缓冲（`DAC_OUTPUTBUFFER_DISABLE`）可到 1MHz+，但需外部运放跟随
- 频率越高每周期采样点越少，波形越粗糙

## CMake 工程配置

```cmake
# 添加驱动源文件
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_DAC/my_dac.c
)

# 添加驱动头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    MY_DRIVE/MY_DAC
)
```
