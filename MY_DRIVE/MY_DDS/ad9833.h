/* AD9833.h */

#ifndef __AD9833_H
#define __AD9833_H

#include "stm32f4xx_hal.h"
#include "main.h"


/* 保留原有操作宏 */
#define WRT(IO, VAL) HAL_GPIO_WritePin(IO.GPIOx, IO.Pin, (VAL) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define GET(IO)      HAL_GPIO_ReadPin(IO.GPIOx, IO.Pin)

/* IO引脚结构体 - 保留原有定义 */
typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint32_t Pin;
} driverIO;

/* 保留外部引脚变量声明 */
extern driverIO AD9833_FSYNC;
extern driverIO AD9833_SCLK;
extern driverIO AD9833_SDATA;


/* 波形输出类型枚举 */
typedef enum
{
    AD9833_WAVEFORM_SINE = 0,                    /**< 正弦波 */
    AD9833_WAVEFORM_TRIANGLE = (1 << 1),         /**< 三角波 */
    AD9833_WAVEFORM_SQUARE = (1 << 5),           /**< 方波(MSB) */
    AD9833_WAVEFORM_SQUARE_DIV2 = (1 << 5) | (1 << 3) /**< 方波(二分频) */
} ad9833_waveform_t;

/* 频率寄存器选择 */
typedef enum
{
    AD9833_FREQ_REG_0 = 0, /**< 频率寄存器0 */
    AD9833_FREQ_REG_1 = 1  /**< 频率寄存器1 */
} ad9833_freq_reg_t;

/* 相位寄存器选择 */
typedef enum
{
    AD9833_PHASE_REG_0 = 0, /**< 相位寄存器0 */
    AD9833_PHASE_REG_1 = 1  /**< 相位寄存器1 */
} ad9833_phase_reg_t;

/* 寄存器地址定义 */
#define AD9833_REG_CMD      (0u << 14)  /**< 控制寄存器 */
#define AD9833_REG_FREQ0    (1u << 14)  /**< 频率寄存器0 */
#define AD9833_REG_FREQ1    (2u << 14)  /**< 频率寄存器1 */
#define AD9833_REG_PHASE0   (6u << 13)  /**< 相位寄存器0 */
#define AD9833_REG_PHASE1   (7u << 13)  /**< 相位寄存器1 */

/* 控制位定义 */
#define AD9833_CMD_B28      (1 << 13)  /**< 28位频率写入使能 */
#define AD9833_CMD_FSEL     (1 << 11)  /**< 频率寄存器选择 */
#define AD9833_CMD_PSEL     (1 << 10)  /**< 相位寄存器选择 */
#define AD9833_CMD_RESET    (1 << 8)   /**< 复位位 */

/* 频率分辨率 2^28 */
#define AD9833_FREQ_RES     268435456.0f

/* 函数声明 */

/**
 * @brief 初始化AD9833设备
 * @param ref_clk 参考时钟频率(Hz)，默认为25MHz
 * @return HAL状态
 */
HAL_StatusTypeDef AD9833_Init(uint32_t ref_clk);

/**
 * @brief 复位AD9833
 */
void AD9833_Reset(void);

/**
 * @brief 清除复位状态
 */
void AD9833_ClearReset(void);

/**
 * @brief 设置输出频率
 * @param freq_reg 频率寄存器选择
 * @param frequency 目标频率(Hz)
 * @param waveform 波形类型
 */
void AD9833_SetFrequency(ad9833_freq_reg_t freq_reg,
                         float frequency, ad9833_waveform_t waveform);

/**
 * @brief 设置输出相位
 * @param phase_reg 相位寄存器选择
 * @param phase_degrees 相位角度(0-360度)
 */
void AD9833_SetPhase(ad9833_phase_reg_t phase_reg, float phase_degrees);

/**
 * @brief 快速设置频率(使用寄存器0)
 * @param frequency 目标频率(Hz)
 * @param waveform 波形类型
 */
void AD9833_SetFrequencyQuick(float frequency, ad9833_waveform_t waveform);

/**
 * @brief 完整配置输出参数
 * @param freq_reg 频率寄存器选择
 * @param phase_reg 相位寄存器选择
 * @param waveform 波形类型
 */
void AD9833_Setup(ad9833_freq_reg_t freq_reg,
                  ad9833_phase_reg_t phase_reg, ad9833_waveform_t waveform);

#endif /* __AD9833_H */