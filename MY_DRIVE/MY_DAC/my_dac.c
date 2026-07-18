#include "my_dac.h"

#include <stdio.h>
#include "ZPN_Uart.h"

#include "arm_math.h"
#include "dac.h"
#include "tim.h"
#include "stm32h743xx.h"

#define MY_DAC_VREF_V                     3.3f
#define MY_DAC_MAX_CODE                   4095.0f
#define MY_DAC_TABLE_MAX_SAMPLES          256U
#define MY_DAC_DEFAULT_SAMPLES            128U
#define MY_DAC_RECOMMENDED_UPDATE_HZ      240000000U

/* This section is explicitly mapped to AXI SRAM in scatter file for DMA1 accessibility. */
__attribute__((section("DAC_DMA_BUFFER"), aligned(32)))
static uint16_t s_sine_lut[MY_DAC_TABLE_MAX_SAMPLES];
static uint16_t s_samples_per_cycle = MY_DAC_DEFAULT_SAMPLES;
static float s_amplitude_vpp = 2.0f;
static float s_offset_v = 1.65f;
static float s_freq_hz = 0.0f;
static uint32_t s_update_hz = 0U;
static uint32_t s_tim6_psc = 0U;
static uint32_t s_tim6_arr = 0U;
static uint8_t s_inited = 0U;
static uint8_t s_started = 0U;

static uint32_t MY_DAC_GetTim6ClkHz(void)
{
	RCC_ClkInitTypeDef clk_cfg = {0};
	uint32_t flash_latency = 0;
	uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();

	HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);
	if (clk_cfg.APB1CLKDivider == RCC_HCLK_DIV1)
	{
		return pclk1_hz;
	}

	return pclk1_hz * 2U;
}

static void MY_DAC_CleanLutDCache(void)
{
#if (__DCACHE_PRESENT == 1U)
	uint32_t addr = (uint32_t)s_sine_lut;
	uint32_t size = (uint32_t)sizeof(s_sine_lut);

	addr &= ~((uint32_t)31U);
	size = (size + 31U) & ~((uint32_t)31U);
	SCB_CleanDCache_by_Addr((uint32_t *)addr, (int32_t)size);
#endif
}

static uint32_t MY_DAC_RoundDivU32(uint32_t num, uint32_t den)
{
	if (den == 0U)
	{
		return 0U;
	}

	return (num + (den / 2U)) / den;
}

static HAL_StatusTypeDef MY_DAC_ReconfigTim6(uint32_t update_hz, uint32_t *actual_update_hz)
{
	uint32_t tim_clk_hz;
	uint64_t denom;
	uint64_t psc_plus1_64;
	uint32_t psc;
	uint32_t tim_cnt_clk;
	uint32_t arr_plus1;
	uint32_t arr;

	if ((update_hz == 0U) || (actual_update_hz == NULL))
	{
		return HAL_ERROR;
	}

	tim_clk_hz = MY_DAC_GetTim6ClkHz();
	denom = (uint64_t)update_hz * 65536ULL;
	psc_plus1_64 = ((uint64_t)tim_clk_hz + denom - 1ULL) / denom;
	if (psc_plus1_64 == 0ULL)
	{
		psc_plus1_64 = 1ULL;
	}
	if (psc_plus1_64 > 65536ULL)
	{
		return HAL_ERROR;
	}

	psc = (uint32_t)(psc_plus1_64 - 1ULL);
	tim_cnt_clk = tim_clk_hz / (psc + 1U);
	arr_plus1 = MY_DAC_RoundDivU32(tim_cnt_clk, update_hz);
	if (arr_plus1 == 0U)
	{
		arr_plus1 = 1U;
	}
	if (arr_plus1 > 65536U)
	{
		arr_plus1 = 65536U;
	}

	arr = arr_plus1 - 1U;

	if (s_started != 0U)
	{
		HAL_TIM_Base_Stop(&htim6);
	}

	__HAL_TIM_DISABLE(&htim6);
	__HAL_TIM_SET_PRESCALER(&htim6, psc);
	__HAL_TIM_SET_AUTORELOAD(&htim6, arr);
	__HAL_TIM_SET_COUNTER(&htim6, 0U);
	HAL_TIM_GenerateEvent(&htim6, TIM_EVENTSOURCE_UPDATE);

	htim6.Init.Prescaler = psc;
	htim6.Init.Period = arr;
	s_tim6_psc = psc;
	s_tim6_arr = arr;

	*actual_update_hz = tim_cnt_clk / arr_plus1;

	if (s_started != 0U)
	{
		if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

static HAL_StatusTypeDef MY_DAC_GenerateSineLut(void)
{
	uint16_t i;
	float half_amp;

	if ((s_samples_per_cycle < 8U) || (s_samples_per_cycle > MY_DAC_TABLE_MAX_SAMPLES))
	{
		return HAL_ERROR;
	}

	if ((s_amplitude_vpp <= 0.0f) || (s_amplitude_vpp > MY_DAC_VREF_V))
	{
		return HAL_ERROR;
	}

	half_amp = s_amplitude_vpp * 0.5f;
	if ((s_offset_v - half_amp < 0.0f) || (s_offset_v + half_amp > MY_DAC_VREF_V))
	{
		return HAL_ERROR;
	}

	for (i = 0U; i < s_samples_per_cycle; i++)
	{
		float theta = 2.0f * PI * ((float)i / (float)s_samples_per_cycle);
		float val_v = s_offset_v + half_amp * arm_sin_f32(theta);
		float code_f = (val_v / MY_DAC_VREF_V) * MY_DAC_MAX_CODE;

		if (code_f < 0.0f)
		{
			code_f = 0.0f;
		}
		else if (code_f > MY_DAC_MAX_CODE)
		{
			code_f = MY_DAC_MAX_CODE;
		}

		s_sine_lut[i] = (uint16_t)(code_f + 0.5f);
	}

	MY_DAC_CleanLutDCache();

	return HAL_OK;
}

HAL_StatusTypeDef MY_DAC_Sine_Init(uint16_t samples_per_cycle, float amplitude_vpp, float offset_v)
{
	s_samples_per_cycle = samples_per_cycle;
	s_amplitude_vpp = amplitude_vpp;
	s_offset_v = offset_v;

	if (MY_DAC_GenerateSineLut() != HAL_OK)
	{
		return HAL_ERROR;
	}

	s_inited = 1U;
	return HAL_OK;
}

/**
 * @brief 初始化方波谐波合成查找表 (基波 f1, 三次 f3, 五次 f5)
 */
HAL_StatusTypeDef MY_DAC_Square_Harmonic_Init(uint16_t samples_per_cycle, float f1_vpp, float f3_vpp, float f5_vpp, float offset_v)
{
	uint16_t i;
	
	if ((samples_per_cycle < 8U) || (samples_per_cycle > MY_DAC_TABLE_MAX_SAMPLES))
	{
		return HAL_ERROR;
	}

	s_samples_per_cycle = samples_per_cycle;
	s_offset_v = offset_v;
	// 记录一个大概的 Vpp 用于调试显示
	s_amplitude_vpp = f1_vpp + f3_vpp + f5_vpp;

	for (i = 0U; i < s_samples_per_cycle; i++)
	{
		float theta = 2.0f * PI * ((float)i / (float)s_samples_per_cycle);
		
		// 合成波形: y = offset + (A1*sin(x) + A3*sin(3x) + A5*sin(5x))
		// 注意这里的 A 是振幅 (Vpp/2)
		float val_v = s_offset_v + 
					  (f1_vpp * 0.5f) * arm_sin_f32(theta) +
					  (f3_vpp * 0.5f) * arm_sin_f32(3.0f * theta) +
					  (f5_vpp * 0.5f) * arm_sin_f32(5.0f * theta);
		
		float code_f = (val_v / MY_DAC_VREF_V) * MY_DAC_MAX_CODE;

		if (code_f < 0.0f) code_f = 0.0f;
		else if (code_f > MY_DAC_MAX_CODE) code_f = MY_DAC_MAX_CODE;

		s_sine_lut[i] = (uint16_t)(code_f + 0.5f);
	}

	MY_DAC_CleanLutDCache();
	s_inited = 1U;
	return HAL_OK;
}

/**
 * @brief 初始化方波谐波合成查找表，支持到 11 次谐波 (f1, f3, f5, f7, f9, f11)
 * @param harmonic_vpps 数组指针，包含 [0]=f1_vpp, [1]=f3_vpp, [2]=f5_vpp, [3]=f7_vpp, [4]=f9_vpp, [5]=f11_vpp
 */
HAL_StatusTypeDef MY_DAC_Square_Harmonic_11_Init(uint16_t samples_per_cycle, float* harmonic_vpps, float offset_v)
{
	uint16_t i;
	
	if ((samples_per_cycle < 8U) || (samples_per_cycle > MY_DAC_TABLE_MAX_SAMPLES) || (harmonic_vpps == NULL))
	{
		return HAL_ERROR;
	}

	s_samples_per_cycle = samples_per_cycle;
	s_offset_v = offset_v;
	s_amplitude_vpp = 0.0f;
	for(int j=0; j<6; j++) s_amplitude_vpp += harmonic_vpps[j];

	for (i = 0U; i < s_samples_per_cycle; i++)
	{
		float theta = 2.0f * PI * ((float)i / (float)s_samples_per_cycle);
		
		float val_v = s_offset_v + 
					  (harmonic_vpps[0] * 0.5f) * arm_sin_f32(theta) +        // f1
					  (harmonic_vpps[1] * 0.5f) * arm_sin_f32(3.0f * theta) + // f3
					  (harmonic_vpps[2] * 0.5f) * arm_sin_f32(5.0f * theta) + // f5
					  (harmonic_vpps[3] * 0.5f) * arm_sin_f32(7.0f * theta) + // f7
					  (harmonic_vpps[4] * 0.5f) * arm_sin_f32(9.0f * theta) + // f9
					  (harmonic_vpps[5] * 0.5f) * arm_sin_f32(11.0f * theta); // f11
		
		float code_f = (val_v / MY_DAC_VREF_V) * MY_DAC_MAX_CODE;

		if (code_f < 0.0f) code_f = 0.0f;
		else if (code_f > MY_DAC_MAX_CODE) code_f = MY_DAC_MAX_CODE;

		s_sine_lut[i] = (uint16_t)(code_f + 0.5f);
	}

	MY_DAC_CleanLutDCache();
	s_inited = 1U;
	return HAL_OK;
}

HAL_StatusTypeDef MY_DAC_Sine_SetFrequency(float freq_hz)
{
	uint32_t target_update_hz;
	uint32_t actual_update_hz;
	float actual_freq;

	if ((s_inited == 0U) || (freq_hz <= 0.0f))
	{
		return HAL_ERROR;
	}

	target_update_hz = (uint32_t)(freq_hz * (float)s_samples_per_cycle + 0.5f);
	if (target_update_hz == 0U)
	{
		return HAL_ERROR;
	}

	if (MY_DAC_ReconfigTim6(target_update_hz, &actual_update_hz) != HAL_OK)
	{
		return HAL_ERROR;
	}

	actual_freq = (float)actual_update_hz / (float)s_samples_per_cycle;
	s_update_hz = actual_update_hz;
	s_freq_hz = actual_freq;

	return HAL_OK;
}

HAL_StatusTypeDef MY_DAC_Sine_SetAmplitudeOffset(float amplitude_vpp, float offset_v)
{
	float keep_freq = s_freq_hz;

	s_amplitude_vpp = amplitude_vpp;
	s_offset_v = offset_v;

	if (MY_DAC_GenerateSineLut() != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (s_started != 0U)
	{
		if (keep_freq <= 0.0f)
		{
			keep_freq = 1000.0f;
		}

		if (MY_DAC_Sine_Start(keep_freq) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

HAL_StatusTypeDef MY_DAC_Sine_Start(float freq_hz)
{
	if (s_inited == 0U)
	{
		if (MY_DAC_Sine_Init(MY_DAC_DEFAULT_SAMPLES, 2.0f, 1.65f) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}

	if (MY_DAC_Sine_SetFrequency(freq_hz) != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_TIM_Base_Stop(&htim6);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)s_sine_lut, s_samples_per_cycle, DAC_ALIGN_12B_R) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
	{
		return HAL_ERROR;
	}

	s_started = 1U;

	{
		int fi = (int)s_freq_hz;
		int ff = (int)((s_freq_hz - (float)fi) * 1000.0f + 0.5f);
		float rec = MY_DAC_Sine_GetRecommendedMaxFrequency();
		int ri = (int)rec;
		int rf = (int)((rec - (float)ri) * 1000.0f + 0.5f);
		UART1_DMAPrintf("DAC sin: f=%d.%03d Hz pts=%u upd=%lu rec=%d.%03d\r\n",
		                fi, ff, s_samples_per_cycle, s_update_hz, ri, rf);
	}
	UART1_DMAPrintf("TIM6: clk=%lu Hz PSC=%lu ARR=%lu LUT@0x%08lX\r\n",
	                MY_DAC_GetTim6ClkHz(), s_tim6_psc, s_tim6_arr,
	                (uint32_t)s_sine_lut);

	return HAL_OK;
}

HAL_StatusTypeDef MY_DAC_Sine_SetSamples(uint16_t samples_per_cycle)
{
	if ((samples_per_cycle < 8U) || (samples_per_cycle > MY_DAC_TABLE_MAX_SAMPLES))
	{
		return HAL_ERROR;
	}

	s_samples_per_cycle = samples_per_cycle;
	if (MY_DAC_GenerateSineLut() != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (s_started != 0U)
	{
		float keep_freq = s_freq_hz;
		if (keep_freq <= 0.0f)
		{
			keep_freq = 1000.0f;
		}

		if (MY_DAC_Sine_Start(keep_freq) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

HAL_StatusTypeDef MY_DAC_Sine_StartAuto(float freq_hz, float amplitude_vpp, float offset_v)
{
	uint16_t samples;

	if (freq_hz <= 0.0f) return HAL_ERROR;

	samples = (uint16_t)(MY_DAC_RECOMMENDED_UPDATE_HZ / freq_hz);
	if (samples < 8U)  samples = 8U;
	if (samples > MY_DAC_TABLE_MAX_SAMPLES) samples = MY_DAC_TABLE_MAX_SAMPLES;

	if (MY_DAC_Sine_Init(samples, amplitude_vpp, offset_v) != HAL_OK)
		return HAL_ERROR;
	return MY_DAC_Sine_Start(freq_hz);
}

float MY_DAC_Sine_GetFrequency(void)
{
	return s_freq_hz;
}

float MY_DAC_Sine_GetTheoreticalMaxFrequency(void)
{
	return (float)MY_DAC_GetTim6ClkHz() / (float)s_samples_per_cycle;
}

float MY_DAC_Sine_GetRecommendedMaxFrequency(void)
{
	return (float)MY_DAC_RECOMMENDED_UPDATE_HZ / (float)s_samples_per_cycle;
}

uint32_t MY_DAC_GetSampleUpdateHz(void)
{
	return s_update_hz;
}
