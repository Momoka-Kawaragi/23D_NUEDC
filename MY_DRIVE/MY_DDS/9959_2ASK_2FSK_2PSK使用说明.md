# AD9959 2ASK / 2FSK / 2PSK 调制使用说明

## 原理概述

三种调制共享同一机制：**函数只调一次**，把两套参数预写入 profile 寄存器。调制由定时器中断翻转 **PS0 引脚**实现：

```
PS0 = 0 → Profile 0 生效
PS0 = 1 → Profile 1 生效
```

| 调制 | Profile 0 (PS0=0) | Profile 1 (PS0=1) | 切换的参数 | CFR byte 0 |
|------|-------------------|-------------------|-----------|------------|
| 2ASK | 载波 + 低幅度 | 载波 + 高幅度 | 幅度 | 0x40 |
| 2FSK | 频率 f0 (逻辑 0) | 频率 f1 (逻辑 1) | 频率 | 0x80 |
| 2PSK | 载波 + 相位 0° | 载波 + 相位 180° | 相位 | 0x00 |

## 核心函数

### SET_2ASK

```c
void SET_2ASK(uint8_t Channel, double f, uint16_t A_start, uint16_t A_stop);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `Channel` | 0~3 | AD9959 通道号 |
| `f` | 1~500M (Hz) | 载波频率，两个 profile 共用 |
| `A_start` | 0~1023 | Profile 0 幅度 (PS0=0)，对应逻辑 0 |
| `A_stop` | 0~1023 | Profile 1 幅度 (PS0=1)，对应逻辑 1 |

写寄存器: FR1 → CFR → ACR(Profile0幅度) → CW1[31:22](Profile1幅度) → CFTW0(载波)

### SET_2FSK

```c
void SET_2FSK(uint8_t Channel, double f_start, double f_stop);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `Channel` | 0~3 | AD9959 通道号 |
| `f_start` | 1~500M (Hz) | Profile 0 频率 (PS0=0)，对应逻辑 0 |
| `f_stop` | 1~500M (Hz) | Profile 1 频率 (PS0=1)，对应逻辑 1 |

写寄存器: FR1 → CFR → CFTW0(Profile0频率) → CW1(Profile1频率)

### SET_2PSK

```c
void SET_2PSK(uint8_t Channel, double f, uint16_t Phase_start, uint16_t Phase_stop);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `Channel` | 0~3 | AD9959 通道号 |
| `f` | 1~500M (Hz) | 载波频率，两个 profile 共用 |
| `Phase_start` | 0~359 (°) | Profile 0 相位 (PS0=0)，对应逻辑 0 |
| `Phase_stop` | 0~359 (°) | Profile 1 相位 (PS0=1)，对应逻辑 1 |

写寄存器: FR1 → CFR → CPOW0(Profile0相位) → CW1[21:6](Profile1相位) → CFTW0(载波)

**注意**：三个函数都需调用 `AD9959_IO_Update()` 使寄存器生效。FR1 均写为 `0xD0`（PLL=16x），如果与 `Init_AD9959` 的 `0xD3`（PLL=19x）不一致，载波频率会有偏差。

---

## 定时器配置 (CubeMX / tim.c)

TIM3 在 APB1 上，定时器时钟 = 84 MHz。配置为 10 kHz 中断（100 µs = 10 kbps）：

```
PSC = 83    → 84 MHz / 84 = 1 MHz
ARR = 99    → 1 MHz / 100 = 10 kHz
```

CubeMX 中勾选 TIM3 的 NVIC 中断使能。

如需其他比特率：

| 目标比特率 | 周期 | PSC | ARR | 公式 |
|-----------|------|-----|-----|------|
| 1 kbps | 1 ms | 839 | 99 | 84M / 840 / 100 |
| 10 kbps | 100 µs | 83 | 99 | 84M / 84 / 100 |
| 50 kbps | 20 µs | 83 | 19 | 84M / 84 / 20 |

---

## main.c 完整示例

三种调制共用同一套 `HAL_TIM_PeriodElapsedCallback` 框架，只需改初始化和变量名。

### 2ASK 示例

```c
#include "ad9959.h"
#include "tim.h"

#define ASK_CARRIER_HZ  60000000U
#define ASK_AMP_LOW     100U
#define ASK_AMP_HIGH    1000U

uint8_t  ask_tx_data   = 0xAA;
uint8_t  ask_bit_index = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    Init_AD9959();
    SET_2ASK(0, ASK_CARRIER_HZ, ASK_AMP_LOW, ASK_AMP_HIGH);
    AD9959_IO_Update();
    HAL_TIM_Base_Start_IT(&htim3);

    while (1) { }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (ask_tx_data & (0x80U >> ask_bit_index))
        HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin, GPIO_PIN_RESET);

    ask_bit_index++;
    if (ask_bit_index >= 8U) ask_bit_index = 0;
}
```

### 2FSK 示例

```c
#include "ad9959.h"
#include "tim.h"

#define FSK_FREQ_LOW   100000U     /* 逻辑 0: 100 kHz */
#define FSK_FREQ_HIGH  10000000U   /* 逻辑 1: 10 MHz  */

uint8_t  fsk_tx_data   = 0xAA;
uint8_t  fsk_bit_index = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    Init_AD9959();
    SET_2FSK(0, FSK_FREQ_LOW, FSK_FREQ_HIGH);
    AD9959_IO_Update();
    HAL_TIM_Base_Start_IT(&htim3);

    while (1) { }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (fsk_tx_data & (0x80U >> fsk_bit_index))
        HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin, GPIO_PIN_RESET);

    fsk_bit_index++;
    if (fsk_bit_index >= 8U) fsk_bit_index = 0;
}
```

### 2PSK 示例

```c
#include "ad9959.h"
#include "tim.h"

#define PSK_CARRIER_HZ  1000000U   /* 1 MHz 载波                      */
#define PSK_PHASE_0     0U         /* 逻辑 0: 0°                      */
#define PSK_PHASE_1     180U       /* 逻辑 1: 180°                    */

uint8_t  psk_tx_data   = 0xAA;
uint8_t  psk_bit_index = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    Init_AD9959();
    SET_2PSK(0, PSK_CARRIER_HZ, PSK_PHASE_0, PSK_PHASE_1);
    AD9959_IO_Update();
    HAL_TIM_Base_Start_IT(&htim3);

    while (1) { }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (psk_tx_data & (0x80U >> psk_bit_index))
        HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin, GPIO_PIN_RESET);

    psk_bit_index++;
    if (psk_bit_index >= 8U) psk_bit_index = 0;
}
```

### stm32f4xx_it.c（无需修改）

```c
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);  // → 自动调用 HAL_TIM_PeriodElapsedCallback
}
```

---

## 切换调制类型

只需改 `main.c` 三处，定时器和回调结构完全不变：

| 切换到 | 初始化调用 | 示例参数 |
|--------|-----------|---------|
| 2ASK | `SET_2ASK(0, 60M, 100, 1000)` | 60MHz 载波，幅度 100↔1000 |
| 2FSK | `SET_2FSK(0, 100k, 10M)` | 100kHz ↔ 10MHz |
| 2PSK | `SET_2PSK(0, 1M, 0, 180)` | 1MHz 载波，相位 0°↔180° |

宏定义和变量名也对应改为 `ask_*` / `fsk_*` / `psk_*`。

---

## 信号流

```
tx_data = 0xAA  (1 0 1 0 1 0 1 0 ...)
                    │
                    ▼
         TIM3 中断 @ 10 kHz
         ┌────100µs────┐
         │ bit=1: PS0=H│  → Profile 1
         │ bit=0: PS0=L│  → Profile 0
         └─────────────┘
                    │
                    ▼
         AD9959 输出: 10 kbps 调制信号
         
         2ASK: 包络跳变 (幅度 100↔1000)
         2FSK: 频率跳变 (100kHz↔10MHz)
         2PSK: 相位跳变 (0°↔180°)
```

---

## 注意事项

1. **函数只调一次** — 它写 AD9959 寄存器，不要在中断/循环里反复调用
2. **必须跟 `AD9959_IO_Update()`** — 否则寄存器不生效
3. **FR1 PLL 一致性问题** — 三个函数均写 `FR1 = 0xD0`（16x），`Init_AD9959` 写的是 `0xD3`（19x）。如需精确频率，统一 PLL 配置
4. **SET_2ASK 已知问题** — CW1 低 22 位未填载波频率，Profile 1 切换到 CW1 时频率部分为 0
5. **PS0 必须为 GPIO_Output** — CubeMX 中确认 PE1 为输出模式
6. **定时器必须在配置函数之后启动** — DDS 没配好就翻转 PS0 会导致输出不确定

---

## nASK 调制（n 阶幅度键控，2~16 阶通用）

### 原理

利用 AD9959 的 profile 引脚 (PS0~PS3) 实现多阶 ASK。n 阶 ASK 使用 `ceil(log2(n))` 个 profile 引脚，每个符号周期输出 `log2(n)` bit：

```
PS[3:0] = symbol  →  选中 Profile 0~(n-1)  →  输出对应的幅度
```

| levels | PS 引脚 | 每符号 bit | 使用寄存器 |
|--------|---------|-----------|-----------|
| 2 | PS0 | 1 | ACR + CW1 |
| 3~4 | PS0, PS1 | 2 | ACR + CW1~CW3 |
| 5~8 | PS0~PS2 | 3 | ACR + CW1~CW7 |
| 9~16 | PS0~PS3 | 4 | ACR + CW1~CW15 |

Profile 0 对应幅度存 ACR，Profile 1~(n-1) 对应幅度存 CW1~CW(n-1)[31:22]。

### 核心函数

#### SET_nASK（通用）

```c
void SET_nASK(uint8_t Channel, double f, uint8_t levels, const uint16_t Amplitudes[]);
```

| 参数 | 范围 | 说明 |
|------|------|------|
| `Channel` | 0~3 | AD9959 通道号 |
| `f` | 1~500M (Hz) | 载波频率，所有 profile 共用 |
| `levels` | 2~16 | ASK 阶数 |
| `Amplitudes` | levels× (0~1023) | 幅度值数组，索引对应 PS[3:0] |

写寄存器: FR1 → CFR → ACR(Profile0) → CW1~CW(levels-1) → CFTW0(载波)

#### SET_16ASK（快捷函数）

```c
void SET_16ASK(uint8_t Channel, double f, const uint16_t Amplitudes[16]);
```

等价于 `SET_nASK(Channel, f, 16, Amplitudes)`。

### 定时器配置

符号率 = 比特率 / bits_per_symbol：

| 阶数 | bps/sym | 10 kbps 符号周期 | PSC | ARR |
|------|---------|-----------------|-----|-----|
| 2-ASK | 1 | 100 µs | 83 | 99 |
| 4-ASK | 2 | 200 µs | 83 | 199 |
| 8-ASK | 3 | 300 µs | 83 | 299 |
| 16-ASK | 4 | 400 µs | 83 | 399 |

### main.c 示例（SET_nASK，以 4-ASK 为例）

```c
#include "ad9959.h"
#include "tim.h"

#define ASK_CARRIER_HZ  60000000U
#define ASK_LEVELS      4  /* 4 阶 ASK */

/* 4 阶幅度表 */
const uint16_t ASK_AMP[ASK_LEVELS] = {0, 341, 682, 1023};

/* 每符号 2 bit，一字节含 4 个符号 */
uint8_t  ask_tx_data[] = {0xE4};  /* 11 10 01 00 → 符号 3,2,1,0 */
uint16_t ask_data_len   = sizeof(ask_tx_data);
uint16_t ask_byte_index = 0;
uint8_t  ask_sym_sel    = 0;  /* 0~3: 一字节内 4 个符号 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    Init_AD9959();
    SET_nASK(0, ASK_CARRIER_HZ, ASK_LEVELS, ASK_AMP);
    AD9959_IO_Update();
    HAL_TIM_Base_Start_IT(&htim3);

    while (1) { }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    uint8_t byte = ask_tx_data[ask_byte_index];
    uint8_t bits_per_sym = 2;  /* ceil(log2(ASK_LEVELS)) */
    uint8_t shift = 8 - bits_per_sym - ask_sym_sel * bits_per_sym;
    uint8_t mask  = (1U << bits_per_sym) - 1U;
    uint8_t symbol = (byte >> shift) & mask;

    /* 写 PS[1:0] = symbol（4-ASK 只用 PS0+PS1） */
    HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin,
        (symbol & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS1_9959_GPIO_Port, PS1_9959_Pin,
        (symbol & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    ask_sym_sel++;
    if (ask_sym_sel >= (8U / bits_per_sym))
    {
        ask_sym_sel = 0;
        ask_byte_index++;
        if (ask_byte_index >= ask_data_len)
            ask_byte_index = 0;
    }
}
```

### 16-ASK 完整示例

```c
#include "ad9959.h"
#include "tim.h"

#define ASK16_CARRIER_HZ  60000000U

/* 16 阶幅度表：0x0~0xF 各对应一个幅度值 */
const uint16_t ASK16_AMP[16] = {
      0,   68,  136,  204,  272,  340,  408,  476,
    544,  612,  680,  748,  816,  884,  952, 1023
};

uint8_t  ask16_tx_data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
uint16_t ask16_data_len   = sizeof(ask16_tx_data);
uint16_t ask16_byte_index = 0;
uint8_t  ask16_nibble_sel = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    Init_AD9959();
    SET_16ASK(0, ASK16_CARRIER_HZ, ASK16_AMP);
    /* 或: SET_nASK(0, ASK16_CARRIER_HZ, 16, ASK16_AMP); */
    AD9959_IO_Update();
    HAL_TIM_Base_Start_IT(&htim3);

    while (1) { }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    uint8_t byte = ask16_tx_data[ask16_byte_index];
    uint8_t symbol;

    if (ask16_nibble_sel == 0)
        symbol = (byte >> 4) & 0x0F;
    else
        symbol = byte & 0x0F;

    HAL_GPIO_WritePin(PS0_9959_GPIO_Port, PS0_9959_Pin,
        (symbol & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS1_9959_GPIO_Port, PS1_9959_Pin,
        (symbol & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS2_9959_GPIO_Port, PS2_9959_Pin,
        (symbol & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS3_9959_GPIO_Port, PS3_9959_Pin,
        (symbol & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (ask16_nibble_sel == 0)
        ask16_nibble_sel = 1;
    else
    {
        ask16_nibble_sel = 0;
        ask16_byte_index++;
        if (ask16_byte_index >= ask16_data_len)
            ask16_byte_index = 0;
    }
}
```

### 信号流

```
tx_data[] = {0xE4}  (4-ASK 示例: 11 10 01 00)
     │
     ▼
每个字节拆为 N 个符号（N = 8 / bits_per_sym）
     │
     ▼
TIM3 中断 @ 符号率
┌────T_sym────┐
│ symbol=3: PS[1:0]=11 → CW3  幅度 1023 (最大)
│ symbol=2: PS[1:0]=10 → CW2  幅度 682
│ symbol=1: PS[1:0]=01 → CW1  幅度 341
│ symbol=0: PS[1:0]=00 → ACR  幅度 0
└─────────────┘
     │
     ▼
AD9959 输出: n 阶包络调制信号
```

### 与 2ASK 的关键区别

| 项目 | 2ASK (SET_2ASK) | nASK (SET_nASK) |
|------|-----------------|-----------------|
| Profile 引脚 | 仅 PS0 | 1~4 个（根据阶数） |
| 阶数 | 2 | 2~16 可配 |
| 每符号 bit 数 | 1 | ceil(log2(n)) |
| 使用寄存器 | ACR + CW1 | ACR + CW1~CW(n-1) |
| GPIO 操作 | 翻转 1 个引脚 | 同时写 k 个引脚 |

### 注意事项

1. **CubeMX 中按需配置 PS 引脚为 GPIO_Output** — 4-ASK 只需 PS0+PS1，16-ASK 需要全部 4 个
2. **幅度表索引与 PS 值对应** — `Amplitudes[0]` 对应 PS[3:0]=0x0，`Amplitudes[3]` 对应 PS[3:0]=0x3
3. **PS 引脚同时更新** — 在中断中一次性写完所有需要的引脚，避免中间态毛刺
4. **nASK 覆盖 2ASK** — `SET_nASK(ch, f, 2, amp)` 等价于 `SET_2ASK(ch, f, amp[0], amp[1])`，但 nASK 不含 2ASK 函数中的 CW1 位宽 bug，推荐使用 nASK
5. **其他注意事项与 2ASK 相同** — FR1 PLL 一致性、`AD9959_IO_Update()`、定时器启动顺序
