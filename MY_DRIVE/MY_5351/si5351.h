/**************************************************************************/
/*!
    @file     Adafruit_SI5351.h
    @author   K. Townsend (Adafruit Industries)

    @section LICENSE
    (版权声明保留英文...)

    https://github.com/ProjectsByJRP/si5351-stm32
*/
/**************************************************************************/
#ifndef _SI5351_H_
#define _SI5351_H_

#include "main.h"
#include "si5351_errors.h"
#include "si5351_asserts.h"

#define SI5351_ADDRESS (0x60) // 假设 ADDR 引脚 = 低电平
#define SI5351_READBIT (0x01)

#define USE_FULL_ASSERT (0x01)


extern I2C_HandleTypeDef hi2c1;


/* 来自 SI5351 ClockBuilder 的测试设置
 * -----------------------------------
 * XTAL (晶振): 25     MHz
 * 通道 0:     120.00 MHz
 * 通道 1:     12.00  MHz
 * 通道 2:     13.56  MHz
 */
static const uint8_t m_si5351_regs_15to92_149to170[100][2] =
    {
        {15, 0x00}, /* 输入源 = PLLA 和 PLLB 使用晶振 */
        {16, 0x4F}, /* CLK0 控制: 8mA 驱动, Multisynth 0 作为 CLK0 源, 时钟不反转, 源 = PLLA, Multisynth 0 为整数模式, 时钟上电 */
        {17, 0x4F}, /* CLK1 控制: 8mA 驱动, Multisynth 1 作为 CLK1 源, 时钟不反转, 源 = PLLA, Multisynth 1 为整数模式, 时钟上电 */
        {18, 0x6F}, /* CLK2 控制: 8mA 驱动, Multisynth 2 作为 CLK2 源, 时钟不反转, 源 = PLLB, Multisynth 2 为整数模式, 时钟上电 */
        {19, 0x80}, /* CLK3 控制: 未使用 ... 时钟掉电 */
        {20, 0x80}, /* CLK4 控制: 未使用 ... 时钟掉电 */
        {21, 0x80}, /* CLK5 控制: 未使用 ... 时钟掉电 */
        {22, 0x80}, /* CLK6 控制: 未使用 ... 时钟掉电 */
        {23, 0x80}, /* CLK7 控制: 未使用 ... 时钟掉电 */
        {24, 0x00}, /* 时钟禁用状态 0..3 (禁用时为低电平) */
        {25, 0x00}, /* 时钟禁用状态 4..7 (禁用时为低电平) */
                    /* PLL_A Setup */
        {26, 0x00},
        {27, 0x05},
        {28, 0x00},
        {29, 0x0C},
        {30, 0x66},
        {31, 0x00},
        {32, 0x00},
        {33, 0x02},
        /* PLL_B Setup */
        {34, 0x02},
        {35, 0x71},
        {36, 0x00},
        {37, 0x0C},
        {38, 0x1A},
        {39, 0x00},
        {40, 0x00},
        {41, 0x86},
        /* Multisynth Setup */
        {42, 0x00},
        {43, 0x01},
        {44, 0x00},
        {45, 0x01},
        {46, 0x00},
        {47, 0x00},
        {48, 0x00},
        {49, 0x00},
        {50, 0x00},
        {51, 0x01},
        {52, 0x00},
        {53, 0x1C},
        {54, 0x00},
        {55, 0x00},
        {56, 0x00},
        {57, 0x00},
        {58, 0x00},
        {59, 0x01},
        {60, 0x00},
        {61, 0x18},
        {62, 0x00},
        {63, 0x00},
        {64, 0x00},
        {65, 0x00},
        {66, 0x00},
        {67, 0x00},
        {68, 0x00},
        {69, 0x00},
        {70, 0x00},
        {71, 0x00},
        {72, 0x00},
        {73, 0x00},
        {74, 0x00},
        {75, 0x00},
        {76, 0x00},
        {77, 0x00},
        {78, 0x00},
        {79, 0x00},
        {80, 0x00},
        {81, 0x00},
        {82, 0x00},
        {83, 0x00},
        {84, 0x00},
        {85, 0x00},
        {86, 0x00},
        {87, 0x00},
        {88, 0x00},
        {89, 0x00},
        {90, 0x00},
        {91, 0x00},
        {92, 0x00},
        /* Misc Config Register */
        {149, 0x00},
        {150, 0x00},
        {151, 0x00},
        {152, 0x00},
        {153, 0x00},
        {154, 0x00},
        {155, 0x00},
        {156, 0x00},
        {157, 0x00},
        {158, 0x00},
        {159, 0x00},
        {160, 0x00},
        {161, 0x00},
        {162, 0x00},
        {163, 0x00},
        {164, 0x00},
        {165, 0x00},
        {166, 0x00},
        {167, 0x00},
        {168, 0x00},
        {169, 0x00},
        {170, 0x00}};

/* 参见 http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf 查看寄存器 26..41 */
enum
{
  SI5351_REGISTER_0_DEVICE_STATUS = 0,
  SI5351_REGISTER_1_INTERRUPT_STATUS_STICKY = 1,
  SI5351_REGISTER_2_INTERRUPT_STATUS_MASK = 2,
  SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL = 3,
  SI5351_REGISTER_9_OEB_PIN_ENABLE_CONTROL = 9,
  SI5351_REGISTER_15_PLL_INPUT_SOURCE = 15,
  SI5351_REGISTER_16_CLK0_CONTROL = 16,
  SI5351_REGISTER_17_CLK1_CONTROL = 17,
  SI5351_REGISTER_18_CLK2_CONTROL = 18,
  SI5351_REGISTER_19_CLK3_CONTROL = 19,
  SI5351_REGISTER_20_CLK4_CONTROL = 20,
  SI5351_REGISTER_21_CLK5_CONTROL = 21,
  SI5351_REGISTER_22_CLK6_CONTROL = 22,
  SI5351_REGISTER_23_CLK7_CONTROL = 23,
  SI5351_REGISTER_24_CLK3_0_DISABLE_STATE = 24,
  SI5351_REGISTER_25_CLK7_4_DISABLE_STATE = 25,
  SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1 = 42,
  SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2 = 43,
  SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3 = 44,
  SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4 = 45,
  SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5 = 46,
  SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6 = 47,
  SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7 = 48,
  SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8 = 49,
  SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1 = 50,
  SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2 = 51,
  SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3 = 52,
  SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4 = 53,
  SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5 = 54,
  SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6 = 55,
  SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7 = 56,
  SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8 = 57,
  SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1 = 58,
  SI5351_REGISTER_59_MULTISYNTH2_PARAMETERS_2 = 59,
  SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3 = 60,
  SI5351_REGISTER_61_MULTISYNTH2_PARAMETERS_4 = 61,
  SI5351_REGISTER_62_MULTISYNTH2_PARAMETERS_5 = 62,
  SI5351_REGISTER_63_MULTISYNTH2_PARAMETERS_6 = 63,
  SI5351_REGISTER_64_MULTISYNTH2_PARAMETERS_7 = 64,
  SI5351_REGISTER_65_MULTISYNTH2_PARAMETERS_8 = 65,
  SI5351_REGISTER_66_MULTISYNTH3_PARAMETERS_1 = 66,
  SI5351_REGISTER_67_MULTISYNTH3_PARAMETERS_2 = 67,
  SI5351_REGISTER_68_MULTISYNTH3_PARAMETERS_3 = 68,
  SI5351_REGISTER_69_MULTISYNTH3_PARAMETERS_4 = 69,
  SI5351_REGISTER_70_MULTISYNTH3_PARAMETERS_5 = 70,
  SI5351_REGISTER_71_MULTISYNTH3_PARAMETERS_6 = 71,
  SI5351_REGISTER_72_MULTISYNTH3_PARAMETERS_7 = 72,
  SI5351_REGISTER_73_MULTISYNTH3_PARAMETERS_8 = 73,
  SI5351_REGISTER_74_MULTISYNTH4_PARAMETERS_1 = 74,
  SI5351_REGISTER_75_MULTISYNTH4_PARAMETERS_2 = 75,
  SI5351_REGISTER_76_MULTISYNTH4_PARAMETERS_3 = 76,
  SI5351_REGISTER_77_MULTISYNTH4_PARAMETERS_4 = 77,
  SI5351_REGISTER_78_MULTISYNTH4_PARAMETERS_5 = 78,
  SI5351_REGISTER_79_MULTISYNTH4_PARAMETERS_6 = 79,
  SI5351_REGISTER_80_MULTISYNTH4_PARAMETERS_7 = 80,
  SI5351_REGISTER_81_MULTISYNTH4_PARAMETERS_8 = 81,
  SI5351_REGISTER_82_MULTISYNTH5_PARAMETERS_1 = 82,
  SI5351_REGISTER_83_MULTISYNTH5_PARAMETERS_2 = 83,
  SI5351_REGISTER_84_MULTISYNTH5_PARAMETERS_3 = 84,
  SI5351_REGISTER_85_MULTISYNTH5_PARAMETERS_4 = 85,
  SI5351_REGISTER_86_MULTISYNTH5_PARAMETERS_5 = 86,
  SI5351_REGISTER_87_MULTISYNTH5_PARAMETERS_6 = 87,
  SI5351_REGISTER_88_MULTISYNTH5_PARAMETERS_7 = 88,
  SI5351_REGISTER_89_MULTISYNTH5_PARAMETERS_8 = 89,
  SI5351_REGISTER_90_MULTISYNTH6_PARAMETERS = 90,
  SI5351_REGISTER_91_MULTISYNTH7_PARAMETERS = 91,
  SI5351_REGISTER_092_CLOCK_6_7_OUTPUT_DIVIDER = 92,
  SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET = 165,
  SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET = 166,
  SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET = 167,
  SI5351_REGISTER_168_CLK3_INITIAL_PHASE_OFFSET = 168,
  SI5351_REGISTER_169_CLK4_INITIAL_PHASE_OFFSET = 169,
  SI5351_REGISTER_170_CLK5_INITIAL_PHASE_OFFSET = 170,
  SI5351_REGISTER_177_PLL_RESET = 177,
  SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
};

typedef enum
{
  SI5351_PLL_A = 0,
  SI5351_PLL_B,
} si5351PLL_t;

typedef enum
{
  SI5351_CRYSTAL_LOAD_6PF = (1 << 6),
  SI5351_CRYSTAL_LOAD_8PF = (2 << 6),
  SI5351_CRYSTAL_LOAD_10PF = (3 << 6)
} si5351CrystalLoad_t;

typedef enum
{
  SI5351_CRYSTAL_FREQ_25MHZ = (25000000),
  SI5351_CRYSTAL_FREQ_27MHZ = (27000000)
} si5351CrystalFreq_t;

typedef enum
{
  SI5351_MULTISYNTH_DIV_4 = 4,
  SI5351_MULTISYNTH_DIV_6 = 6,
  SI5351_MULTISYNTH_DIV_8 = 8
} si5351MultisynthDiv_t;

typedef enum
{
  SI5351_R_DIV_1 = 0,
  SI5351_R_DIV_2 = 1,
  SI5351_R_DIV_4 = 2,
  SI5351_R_DIV_8 = 3,
  SI5351_R_DIV_16 = 4,
  SI5351_R_DIV_32 = 5,
  SI5351_R_DIV_64 = 6,
  SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

typedef struct
{
  uint8_t initialised;             /* 初始化标志 */
  si5351CrystalFreq_t crystalFreq; /* 晶振频率 */
  si5351CrystalLoad_t crystalLoad; /* 晶振负载电容 */
  uint32_t crystalPPM;             /* 晶振误差 PPM */
  uint8_t plla_configured;         /* PLL A 配置标志 */
  uint32_t plla_freq;              /* PLL A 频率 */
  uint8_t pllb_configured;         /* PLL B 配置标志 */
  uint32_t pllb_freq;              /* PLL B 频率 */
  uint32_t ms0_freq;               /* MS0 频率 */
  uint32_t ms1_freq;               /* MS1 频率 */
  uint32_t ms2_freq;               /* MS2 频率 */
  uint32_t ms0_r_div;              /* MS0 R 分频器 */
  uint32_t ms1_r_div;              /* MS1 R 分频器 */
  uint32_t ms2_r_div;              /* MS2 R 分频器 */
} si5351Config_t;

/**
 * @brief 初始化 Si5351 芯片和内部状态。
 * 在任何其他操作之前必须调用此函数。
 * @return err_t: 错误状态码。
 */
err_t si5351_Init(void);

/**
 * @brief 设置 PLL (锁相环) 为分数分频模式。
 * 用于设置高精度的 VCO 频率 (PLL_Freq = XTAL_Freq * (mult + num/denom))。
 * @param pll: 目标 PLL (SI5351_PLL_A 或 SI5351_PLL_B)。
 * @param mult: PLL 的整数倍频系数 (a)。
 * @param num: 分数分频器的分子 (b)。
 * @param denom: 分数分频器的分母 (c)。
 * @return err_t: 错误状态码。
 */
err_t si5351_setupPLL(si5351PLL_t pll, uint8_t mult, uint32_t num, uint32_t denom);

/**
 * @brief 设置 PLL (锁相环) 为纯整数分频模式。
 * 用于设置简单的 VCO 频率 (PLL_Freq = XTAL_Freq * mult)。
 * @param pll: 目标 PLL (SI5351_PLL_A 或 SI5351_PLL_B)。
 * @param mult: PLL 的整数倍频系数 (a)。
 * @return err_t: 错误状态码。
 */
err_t si5351_setupPLLInt(si5351PLL_t pll, uint8_t mult);

/**
 * @brief 设置 Multisynth (多重合成器) 为分数分频模式。
 * 用于设置精确的输出频率 (f_out = f_pll / (div + num/denom))。
 * @param output: 目标输出通道 (0 到 7)。
 * @param pllSource: Multisynth 的时钟源 (SI5351_PLL_A 或 SI5351_PLL_B)。
 * @param div: Multisynth 的整数分频系数 (a)。
 * @param num: 分数分频器的分子 (b)。
 * @param denom: 分数分频器的分母 (c)。
 * @return err_t: 错误状态码。
 */
err_t si5351_setupMultisynth(uint8_t output, si5351PLL_t pllSource, uint32_t div, uint32_t num, uint32_t denom);

/**
 * @brief 设置 Multisynth (多重合成器) 为纯整数分频模式。
 * 仅支持 4, 6, 8 分频。
 * @param output: 目标输出通道 (0 到 7)。
 * @param pllSource: Multisynth 的时钟源 (SI5351_PLL_A 或 SI5351_PLL_B)。
 * @param div: 目标整数分频值 (SI5351_MULTISYNTH_DIV_4/6/8)。
 * @return err_t: 错误状态码。
 */
err_t si5351_setupMultisynthInt(uint8_t output, si5351PLL_t pllSource, si5351MultisynthDiv_t div);

/**
 * @brief 启用或禁用所有的时钟输出。
 * @param enabled: 1 表示启用 (ENABLE)，0 表示禁用 (DISABLE)。
 * @return err_t: 错误状态码。
 */
err_t si5351_enableOutputs(uint8_t enabled);

/**
 * @brief 设置输出通道的 R-分频器 (R-Divider)。
 * R-Divider 是一个固定的 $2^n$ 分频器，用于产生极低频时钟。
 * @param output: 目标输出通道 (0 到 7)。
 * @param div: R-分频器的值 (例如 SI5351_R_DIV_32)。
 * @return err_t: 错误状态码。
 */
err_t si5351_setupRdiv(uint8_t output, si5351RDiv_t div);

static si5351Config_t m_si5351Config;

err_t si5351_write8(uint8_t reg, uint8_t value);
err_t si5351_read8(uint8_t reg, uint8_t *value);
err_t SI5351_SetFrequency(uint8_t clk, uint32_t fout);
err_t SI5351_SetChannelPower(uint8_t channel, uint8_t enabled);

#endif
