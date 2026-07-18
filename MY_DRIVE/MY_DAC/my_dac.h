/* USER CODE BEGIN Header */
#ifndef __MY_DAC_H__
#define __MY_DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef MY_DAC_Sine_Init(uint16_t samples_per_cycle, float amplitude_vpp, float offset_v);
HAL_StatusTypeDef MY_DAC_Square_Harmonic_Init(uint16_t samples_per_cycle, float f1_vpp, float f3_vpp, float f5_vpp, float offset_v);
HAL_StatusTypeDef MY_DAC_Square_Harmonic_11_Init(uint16_t samples_per_cycle, float* harmonic_vpps, float offset_v);
HAL_StatusTypeDef MY_DAC_Sine_Start(float freq_hz);
HAL_StatusTypeDef MY_DAC_Sine_SetFrequency(float freq_hz);
HAL_StatusTypeDef MY_DAC_Sine_SetSamples(uint16_t samples_per_cycle);
HAL_StatusTypeDef MY_DAC_Sine_SetAmplitudeOffset(float amplitude_vpp, float offset_v);
HAL_StatusTypeDef MY_DAC_Sine_StartAuto(float freq_hz, float amplitude_vpp, float offset_v);

float MY_DAC_Sine_GetFrequency(void);
float MY_DAC_Sine_GetTheoreticalMaxFrequency(void);
float MY_DAC_Sine_GetRecommendedMaxFrequency(void);
uint32_t MY_DAC_GetSampleUpdateHz(void);

#ifdef __cplusplus
}
#endif

#endif /* __MY_DAC_H__ */
