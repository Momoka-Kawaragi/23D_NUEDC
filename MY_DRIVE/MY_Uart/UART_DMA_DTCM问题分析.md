# STM32H7 DMA 无法访问 DTCM 问题分析与修复

## 问题现象

移植 MY_Uart 驱动后，`UART1_DMAPrintf()` 调用返回成功（返回值 > 0），但串口助手收不到任何数据。而直接调用 `HAL_UART_Transmit()`（阻塞模式）或 `HAL_UART_Transmit_DMA()` 使用字符串字面量（存储在 FLASH 中）可以正常发送。

## 排查过程

通过逐层隔离测试，最终定位到关键差异：

| 测试 | 缓冲区位置 | DMA TX 结果 |
|------|-----------|------------|
| `HAL_UART_Transmit_DMA("string_literal")` | FLASH (0x08000000) | **成功** |
| `HAL_UART_Transmit_DMA(stack_buf)` | DTCM (0x20000000) | **静默失败** |
| `HAL_UART_Transmit_DMA(static_buf)` | DTCM (0x20000000) | **静默失败** |
| `HAL_UART_Transmit_DMA(axi_buf)` | AXI SRAM (0x24000000) | **成功** |
| `HAL_UART_Transmit_DMA(d2_buf)` | D2 SRAM (0x30000000) | **成功** |

所有 DTCM 缓冲区的 DMA 发送都静默失败（`HAL_UART_Transmit_DMA` 返回 `HAL_OK`，但实际没有数据传输），而 AXI SRAM 和 D2 SRAM 缓冲区都正常工作。

## 根因分析

### STM32H7 的内存架构

STM32H743 拥有多块独立的 SRAM，分布在不同的总线域中：

```
┌─────────────────────────────────────────────────────┐
│  Cortex-M7 核心 (D1 域)                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ ITCM     │  │ DTCM     │  │ AXI SRAM (512KB) │   │
│  │ 64KB     │  │ 128KB    │  │ 0x24000000       │   │
│  │ 0x00000000│ │ 0x20000000│ │                  │   │
│  └──────────┘  └──────────┘  └──────────────────┘   │
│       │             │               │                │
│       └─── TCM 接口 ─┘               │                │
│         (直连CPU核)                  │                │
└──────────────────────────────────────┼────────────────┘
                                       │ AXI 总线矩阵
┌──────────────────────────────────────┼────────────────┐
│  D2 域                               │                │
│  ┌──────────────────┐    ┌───────────────────────┐    │
│  │ D2 SRAM (288KB)  │    │ DMA1 / DMA2           │    │
│  │ 0x30000000       │    │ (外设 → 存储器)       │    │
│  └──────────────────┘    └───────────────────────┘    │
└───────────────────────────────────────────────────────┘
```

### DTCM 的特殊性

**DTCM (Tightly Coupled Memory)** 是 Cortex-M7 核心的紧耦合内存，通过专用的 TCM 接口直连 CPU：

- **优点**：零等待访问，确定性延迟，最适合栈和性能关键的代码/数据
- **缺点**：DMA 访问需要通过 AXI 从端口绕行，在某些 STM32H7 配置下可能不可靠或完全无法访问

DTCM 不像 AXI SRAM 那样挂在系统 AXI 总线上。当 DMA1（位于 D2 域）尝试读取 DTCM 时，请求路径是：

```
DMA1 → D2 AHB 总线 → AHB-to-AXI 桥 → AXI 总线矩阵 → D1 域 → TCM 接口 → DTCM
```

这条路径在某些时钟配置或总线仲裁条件下可能出现问题，表现为 **DMA 传输静默失败**——DMA 控制器正常启动、正常返回完成状态，但实际没有数据传输。

### 为什么 HAL 不报错

`HAL_UART_Transmit_DMA()` 的流程：

1. 检查 `gState == READY` → 通过
2. 调用 `HAL_DMA_Start_IT()` 配置 DMA 寄存器
3. 使能 `USART_CR3_DMAT` 位
4. 返回 `HAL_OK`

DMA 的配置和启动由硬件寄存器完成，不依赖对源地址的实际读取验证。即使源地址在 DMA 不可达的 DTCM，DMA 硬件也不会在启动阶段报错。

## 修复方案

修改链接脚本 `STM32H743XX_FLASH.ld`，将 `.data`（已初始化全局变量）和 `.bss`（未初始化全局变量）从 DTCMRAM 迁移到 AXI SRAM (RAM 区域，0x24000000，512KB)：

**修改前：**
```ld
.data :
{
    ...
} >DTCMRAM AT> FLASH

.bss (NOLOAD) :
{
    ...
} >DTCMRAM
```

**修改后：**
```ld
.data :
{
    ...
} >RAM AT> FLASH

.bss (NOLOAD) :
{
    ...
} >RAM
```

### 内存布局变化

| 区域 | 修改前 | 修改后 |
|------|--------|--------|
| DTCMRAM (128KB) | .data + .bss + stack + heap | **仅 stack + heap** |
| RAM / AXI SRAM (512KB) | 空 | .data + .bss（所有静态变量） |

### 为什么保留 DTCM 给栈和堆

- **栈**：每个函数调用、中断进入都会访问栈，DTCM 的零等待特性最适合
- **堆**：`malloc`/`free` 频繁操作，也从低延迟受益
- 栈和堆不需要 DMA 访问，不受 DTCM 限制影响

### 为什么选择 AXI SRAM 而非 D2 SRAM

- AXI SRAM 在 D1 域，和 CPU 核心同域，访问延迟较低
- DMA 可通过 AXI 总线矩阵直接访问，路径相对直接
- 512KB 容量足够当前项目使用

## 附加修复：UART2 中断缺失

在排查过程中还发现 UART2 的两个问题：

1. **NVIC 中断未使能**：`HAL_UART_MspInit()` 中 USART2 没有调用 `HAL_NVIC_EnableIRQ(USART2_IRQn)`
2. **中断处理函数缺失**：`stm32h7xx_it.c` 中没有 `USART2_IRQHandler`

导致 `HAL_UART_Transmit_IT()` / `HAL_UART_Receive_IT()` 启动后无法触发完成回调，UART2 完全无法工作。

## 经验总结

1. **DMA 缓冲区位置是 STM32H7 开发的关键关注点**。默认链接脚本将数据放在 DTCM，对 CPU 最优，但对 DMA 不友好
2. **DMA 操作静默失败不一定会触发 HAL 错误**。在 STM32H7 上调试 DMA 问题时，优先检查缓冲区地址所在的内存区域
3. **分层测试是定位硬件底层问题的最有效方法**。从最简单的 HAL 阻塞调用开始，逐步增加复杂度（阻塞 → 裸 DMA → 驱动封装），每层独立验证

## 相关 Git 提交

| 提交 | 说明 |
|------|------|
| `fd5862c` | 核心修复：移动 .bss/.data 到 AXI SRAM |
| `075a4f9` | 附带修复：UART2 NVIC IRQ 和中断处理函数 |
