#include "AD9833.h"
#include <math.h>


driverIO AD9833_FSYNC = {AD9833_FSYNC_GPIO, AD9833_FSYNC_Pin};
driverIO AD9833_SCLK = {AD9833_SCLK_GPIO, AD9833_SCLK_Pin};
driverIO AD9833_SDATA = {AD9833_SDATA_GPIO, AD9833_SDATA_Pin};


/* 静态全局变量保存参考时钟 */
static uint32_t g_ref_clk = 25000000;  /**< 默认25MHz */

/* 内部静态函数 */
static void AD9833_SPI_Write(uint16_t data);
static void AD9833_SetRegisterValue(uint16_t reg_value);

/**
 * @brief 通过模拟SPI发送16位数据
 * @param data 待发送的16位数据
 */
static void AD9833_SPI_Write(uint16_t data)
{
    uint8_t i;
    
    // FSYNC拉低，开始通信
    WRT(AD9833_FSYNC, 0);
    
    // 时钟16位数据，MSB在前
    for (i = 0; i < 16; i++)
    {
        // 设置数据位
        WRT(AD9833_SDATA, (data & 0x8000) ? 1 : 0);
        data <<= 1;
        
        // 时钟下降沿
        WRT(AD9833_SCLK, 0);
        
        // 时钟上升沿
        WRT(AD9833_SCLK, 1);
    }
    
    // FSYNC拉高，锁存数据
    WRT(AD9833_FSYNC, 1);
    WRT(AD9833_SDATA, 1);
}

/**
 * @brief 写入寄存器值
 * @param reg_value 寄存器值
 */
static void AD9833_SetRegisterValue(uint16_t reg_value)
{
    AD9833_SPI_Write(reg_value);
    HAL_Delay(1);  // 寄存器更新延时
}

/**
 * @brief 初始化AD9833设备
 * @param ref_clk 参考时钟频率(Hz)
 * @return HAL状态
 */
HAL_StatusTypeDef AD9833_Init(uint32_t ref_clk)
{
    if (ref_clk == 0)
    {
        return HAL_ERROR;
    }
    
    // 保存参考时钟
    g_ref_clk = ref_clk;
    
    // 设置默认引脚状态
    WRT(AD9833_SCLK, 1);   // SCLK默认高
    WRT(AD9833_FSYNC, 1);  // FSYNC默认高
    WRT(AD9833_SDATA, 1);  // SDATA默认高
    
    // 复位设备
    AD9833_Reset();
    HAL_Delay(10);
    AD9833_ClearReset();
    
    return HAL_OK;
}

/**
 * @brief 复位AD9833
 */
void AD9833_Reset(void)
{
    AD9833_SetRegisterValue(AD9833_REG_CMD | AD9833_CMD_RESET);
}

/**
 * @brief 清除复位状态
 */
void AD9833_ClearReset(void)
{
    AD9833_SetRegisterValue(AD9833_REG_CMD);
}

/**
 * @brief 设置输出频率
 * @param freq_reg 频率寄存器选择
 * @param frequency 目标频率(Hz)
 * @param waveform 波形类型
 */
void AD9833_SetFrequency(ad9833_freq_reg_t freq_reg,
                         float frequency, ad9833_waveform_t waveform)
{
    // 计算频率寄存器值: freq_word = (frequency * 2^28) / fclk
    float freq_factor = AD9833_FREQ_RES / (float)g_ref_clk;
    uint32_t freq_word = (uint32_t)(frequency * freq_factor + 0.5f);
    
    // 限制在28位范围内
    if (freq_word > 0x0FFFFFFF)
    {
        freq_word = 0x0FFFFFFF;
    }
    
    uint16_t freq_reg_base = (freq_reg == AD9833_FREQ_REG_0) ? AD9833_REG_FREQ0 : AD9833_REG_FREQ1;
    uint16_t control_word = AD9833_REG_CMD | AD9833_CMD_B28 |
                           (freq_reg == AD9833_FREQ_REG_1 ? AD9833_CMD_FSEL : 0) |
                           (uint16_t)waveform;
    
    // 写入控制字并启用B28模式
    AD9833_SetRegisterValue(control_word);
    
    // 写入低14位
    AD9833_SetRegisterValue(freq_reg_base | (uint16_t)(freq_word & 0x3FFF));
    
    // 写入高14位
    AD9833_SetRegisterValue(freq_reg_base | (uint16_t)((freq_word >> 14) & 0x3FFF));
}

/**
 * @brief 设置输出相位
 * @param phase_reg 相位寄存器选择
 * @param phase_degrees 相位角度(0-360度)
 */
void AD9833_SetPhase(ad9833_phase_reg_t phase_reg, float phase_degrees)
{
    // 将角度转换为12位寄存器值(0-4095对应0-360°)
    uint16_t phase_word = (uint16_t)((phase_degrees / 360.0f) * 4096.0f);
    phase_word &= 0x0FFF;  // 保留12位
    
    uint16_t phase_reg_base = (phase_reg == AD9833_PHASE_REG_0) ? AD9833_REG_PHASE0 : AD9833_REG_PHASE1;
    AD9833_SetRegisterValue(phase_reg_base | phase_word);
}

/**
 * @brief 快速设置频率(使用寄存器0)
 * @param frequency 目标频率(Hz)
 * @param waveform 波形类型
 */
void AD9833_SetFrequencyQuick(float frequency, ad9833_waveform_t waveform)
{
    AD9833_SetFrequency(AD9833_FREQ_REG_0, frequency, waveform);
}

/**
 * @brief 完整配置输出参数
 * @param freq_reg 频率寄存器选择
 * @param phase_reg 相位寄存器选择
 * @param waveform 波形类型
 */
void AD9833_Setup(ad9833_freq_reg_t freq_reg,
                  ad9833_phase_reg_t phase_reg, ad9833_waveform_t waveform)
{
    uint16_t control_word = AD9833_REG_CMD |
                           (freq_reg == AD9833_FREQ_REG_1 ? AD9833_CMD_FSEL : 0) |
                           (phase_reg == AD9833_PHASE_REG_1 ? AD9833_CMD_PSEL : 0) |
                           (uint16_t)waveform;
                           AD9833_SetRegisterValue(control_word);
}