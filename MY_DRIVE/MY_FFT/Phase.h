/**
 ******************************************************************************
 * @file    Phase.h
 * @brief   相位计算与FFT算法头文件 - 适配STM32H7系列
 * @author  POLO_HU
 * @version V2.3
 * @date    2026-06-07
 ******************************************************************************
 */

#ifndef __PHASE_H
#define __PHASE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含必要的头文件 */
#include "main.h"           // HAL 库头文件
#include "arm_math.h"       // CMSIS DSP数学库
#include "arm_const_structs.h"  // CMSIS FFT常量结构体
#include <string.h>         // 包含memcpy等函数
#include <stdio.h>
#include <math.h>

/* 宏定义 */

/**
 * @brief FFT变换长度
 * @note  支持64, 128, 256, 512, 1024, 2048, 4096
 */
/* TIM3时钟源选择: 0=外部Si5351(1.024MHz), 1=内部APB1(240MHz) */
#define PHASE_CLK_EXTERNAL    0
#define PHASE_CLK_INTERNAL    1
#define PHASE_CLOCK_SOURCE    PHASE_CLK_INTERNAL  /* <-- 改这里切换 */

#define PHASE_TARGET_FS       512000.0f  /* 目标采样率(Hz) */

#define FFT_LENGTH          2048

/**
 * @brief ADC有效位宽
 * @note  STM32H743 默认16位；若切回F4可改为12
 */
#define ADC_EFFECTIVE_BITS 12


/**
 * @brief 默认采样率(Hz)
 * @note  运行时会优先按TIM3当前配置动态计算实际采样率
 */
#define SAMPLING_RATE_DEFAULT 64000.0f

/**
 * @brief 基波搜索范围(Hz)
 * @note  用于在FFT结果中限定基波查找频带，避免低频噪声误判。
 */
#define FUND_SEARCH_MIN_HZ   800.0f
#define FUND_SEARCH_MAX_HZ   1200.0f

/**
 * @brief 谐波能量统计的半宽(bin)
 * @note  每个谐波在中心bin左右各取该半宽范围累加能量。
 */
#define HARM_BIN_HALF_WIDTH  5U

/* 兼容旧代码中对 SAMPLING_RATE 的引用 */
#define SAMPLING_RATE       SAMPLING_RATE_DEFAULT

/* 测试宏定义 (取消注释以启用) */
//#define FFT_TEST_SIMULATION       // 启用内部模拟信号生成(无需外部信号源)
//#define FFT_OUTPUT_FULL_SPECTRUM  // 启用全频谱输出
// #define FFT_OUTPUT_BINARY         // 启用二进制(JustFloat)格式输出，需配合VOFA+使用


/* 类型定义 */

/**
 * @brief FFT 峰值信息
 */
typedef struct {
    float32_t freq_hz;   /* 峰值频率 (Hz) */
    float32_t mag;       /* 峰值幅值 */
    uint16_t  bin;       /* 峰值所在 bin 索引 */
} Phase_PeakInfo_t;

/* 外部变量声明 */

extern float32_t Reference_Voltage;
extern volatile uint8_t ADC_COMPLETED;
extern uint32_t ADC_Raw_Data[FFT_LENGTH];
extern uint8_t ifftFlag;
extern uint8_t doBitReverse;
extern uint16_t ADC_1_Value_DMA[FFT_LENGTH];
extern uint16_t ADC_2_Value_DMA[FFT_LENGTH];
extern float32_t ADC_1_Real_Value[FFT_LENGTH];
extern float32_t ADC_2_Real_Value[FFT_LENGTH];
extern float32_t FFT_InputBuf[FFT_LENGTH * 2];
extern float32_t FFT_OutputBuf[FFT_LENGTH];
extern uint16_t ADC_Buffer[FFT_LENGTH];
extern uint16_t ADC_Buffer2[FFT_LENGTH];
extern volatile uint8_t ADC_Flag;
extern volatile uint8_t g_phase_ready;
extern volatile uint32_t g_cb_count;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern float32_t window;

/* 函数声明 */

/**
 * @brief  FFT应用初始化
 * @note   在主函数初始化阶段调用
 */
void FFT_App_Init(void);

/**
 * @brief  FFT应用主处理函数
 * @note   在主循环中调用
 */
void FFT_App_Process(void);
void FFT_SendSpectrumFrame(void);
void FFT_SendPhaseDiffFrame(void);
void Phase_FFT_OneChannel(const uint16_t *snap,
                          float32_t *out_phase_deg,
                          float32_t *out_mag_v,
                          float32_t *out_freq_hz);
void FFT_IFFT_SendWaveform(void);
void Phase_RequestCalibrate(void);

/* 兼容旧接口函数 */
void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void Process_ADC_RawData(void);
float32_t Get_PhaseDifference(void);
int Find_nMax(const float *ARR);
float32_t Find_PhaseAngle(float32_t *signal);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void CpltCallback(ADC_HandleTypeDef* hadc);
uint16_t Get_FFT_Spectrum(float32_t* buffer, uint16_t length);
uint16_t Phase_Get_PeakFreq(Phase_PeakInfo_t *peaks, uint16_t max_peaks);
void Set_Reference_Voltage(float32_t voltage);
float32_t Get_Reference_Voltage(void);
void ADC_Signal_Collect_To_ADC_Buffer(void);
void arm_cfft_f32_app(float32_t *rawData, const arm_cfft_instance_f32 *fft_instance);
void Apply_Hanning_Window(float32_t *signal, uint16_t length);
void Apply_FlatTop_Window(float32_t *signal, uint16_t length);

/* 内联函数 */

/**
 * @brief  判断FFT长度是否有效
 */
static inline uint8_t Is_FFT_Length_Valid(uint16_t length)
{
  return (length & (length - 1)) == 0;
}

/**
 * @brief  将ADC值转换为电压值
 */
static inline float32_t ADC_Value_To_Voltage(uint16_t adc_value)
{
  const uint32_t adc_fs = (1UL << ADC_EFFECTIVE_BITS) - 1UL;
  return ((float32_t)adc_value / (float32_t)adc_fs) * Reference_Voltage;
}

/**
 * @brief  将电压值转换为ADC值
 */
static inline uint16_t Voltage_To_ADC_Value(float32_t voltage)
{
  const uint32_t adc_fs = (1UL << ADC_EFFECTIVE_BITS) - 1UL;
  return (uint16_t)((voltage / Reference_Voltage) * (float32_t)adc_fs);
}

#ifdef __cplusplus
}
#endif

#endif /* __PHASE_H */
