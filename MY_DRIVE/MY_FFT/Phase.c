/**
 ******************************************************************************
 * @file    Phase.c
 * @brief   相位计算与FFT算法源文件 - 适配STM32H7系列
 * @author  POLO_HU（基于学长唐韩宇）
 * @version V2.3
 * @date    2026-06-07
 ******************************************************************************
 * @attention
 *
 * 本文件实现了基于FFT算法的频谱与相位分析功能，适用于STM32H7系列微控制器。
 * 主要功能：
 * 1. ADC1/ADC2 双路 DMA 采集 (TIM3 TRGO触发, NORMAL模式+回调重装)
 * 2. 4096点FFT变换 (CMSIS DSP arm_cfft_f32)
 * 3. Hanning窗 / 幅值归一化 (输出实际电压)
 * 4. 串口频谱输出 (FFT_BEGIN/FFT_END 帧, 含 F0 Rife-Hanning插值频率)
 * 5. 双通道相位差计算 (PDIFF 帧输出)
 * 6. FFT→IFFT 波形重构 (WAVE_BEGIN/WAVE_END 帧)
 *
 * @note   V2.3 变更:
 *         - Rife-Hanning插值替代抛物线插值: 基于Hanning窗频谱解析解
 *         - Rife幅值修正: 回收Hanning窗泄漏能量, 峰值精度更高
 *         - Phase_ComputeDiff 使用独立缓冲区, 不再复用全局FFT缓冲
 *         - DFT精修相位: 在Rife精修频率处做DFT取相位, 消除bin对齐误差
 *         - 跨功率谱法相位差: S12 = DFT_CH1 × conj(DFT_CH2)
 *         - PDIFF帧新增 raw1/raw2 字段 (Rife修正前的原始FFT峰值)
 *         - 双参数相位校准: 固定偏置 + 时间延迟补偿
 *         V2.2 变更:
 *         - 新增 Phase_FFT_OneChannel(): 单通道FFT+基波搜索, 输出相位/幅值/频率
 *         - 新增抛物线插值精修频率: 三点拟合 → 亚bin精度 (~0.1 Hz)
 *         - 串口 header 新增 F0 字段: MATLAB 脚本可直接读取精修频率
 *         - 新增双通道相位差计算: Phase_ComputeDiff() + FFT_SendPhaseDiffFrame()
 *         - 新增 FFT→IFFT 波形重构: FFT_IFFT_SendWaveform()
 *         - ADC 切换双路 NORMAL 模式 + 抛物线插值精修相位差, 消除数据竞争
 *         - 新增模拟信号生成: Phase_GenerateSimulatedSignal() 用于调试
 *         V2.1 变更:
 *         - 移除 DFT/THD 计算, 简化为纯FFT频谱输出
 *         - 窗函数改用 WindowFunction.c 的 Hanning 窗
 *         - TIM3 周期覆盖为 2 (采样率 512kHz)
 *         - 修正 Si5351 ETR 时钟下的采样率自动计算
 *         - 添加 FFT_SendSpectrumFrame() 输出 MATLAB 脚本兼容格式
 *
 ******************************************************************************
 */

#include "Phase.h"
#include "WindowFunction.h"
#include "stm32f4xx.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "ZPN_Uart.h"
#include <stdio.h>

#define PHASE_MAX_HARMONIC 20  /* 最多谐波次数(受Nyquist限制) */

/* ── 通道幅值校准 ── */
#define CH2_AMP_CAL_SCALE   1.000f

/* 当前工程ADC配置为16bit，满量程65535 */
static uint32_t ADC_MaxValue = ((1UL << ADC_EFFECTIVE_BITS) - 1UL);
static float32_t FFT_SamplingRate_Hz = SAMPLING_RATE_DEFAULT;
static volatile uint8_t g_frame_ready = 0;
volatile uint8_t g_phase_ready = 0;
volatile uint32_t g_cb_count = 0;
static volatile uint8_t g_transmitting = 0;
static volatile uint8_t g_adc1_done = 0;
static volatile uint8_t g_adc2_done = 0;

/* 双通道快照缓冲区 */
static uint16_t adc_snapshot[FFT_LENGTH] = {0};
static uint16_t adc_snapshot2[FFT_LENGTH] = {0};
uint16_t ADC_Buffer2[FFT_LENGTH] = {0};

/* 相位差校准 (双参数: 固定偏置 + 时间延迟)
 * 偏置 = OFFSET_DEG + 360 × f0 × DELAY_US × 1e-6
 * 校准: PC4+PC5接同一信号, 10kHz测得6.87°, 20kHz测得-8.75°
 *   解方程: φ0 + 360×10k×Δt×1e-6 = 6.87
 *           φ0 + 360×20k×Δt×1e-6 = -8.75
 *   → φ0 = 22.49°, Δt = -3.928μs */

static float32_t g_phase_diff_deg = 0.0f;
static float32_t g_phase_offset = 0.0f;
static volatile uint8_t g_cal_request = 0;
static float32_t g_f0_hz = 0.0f;
static float32_t g_mag_ch1 = 0.0f;
static float32_t g_mag_ch2 = 0.0f;
static float32_t g_mag_ch1_raw = 0.0f;
static float32_t g_mag_ch2_raw = 0.0f;

/**
 * @brief 启动双路ADC DMA采集与定时器触发
 */

static void Phase_StartAcq(void)
{
  HAL_ADC_Stop_DMA(&hadc1);
  hadc1.DMA_Handle->State = HAL_DMA_STATE_RESET;
  hadc1.State = HAL_ADC_STATE_RESET;
  hadc1.Init.ExternalTrigConv          = ADC_EXTERNALTRIGCONV_T2_TRGO;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  { ADC_ChannelConfTypeDef s = {0};
    s.Channel = ADC_CHANNEL_0; s.Rank = 1;   /* PA0 */
    s.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) Error_Handler(); }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Buffer, FFT_LENGTH) != HAL_OK) Error_Handler();

  { ADC_ChannelConfTypeDef s = {0};
    HAL_ADC_Stop_DMA(&hadc2);
    hadc2.DMA_Handle->State = HAL_DMA_STATE_RESET;
    hadc2.State = HAL_ADC_STATE_RESET;
    hadc2.Init.ExternalTrigConv          = ADC_EXTERNALTRIGCONV_T3_TRGO;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
    s.Channel = ADC_CHANNEL_5; s.Rank = 1;   /* PA5 */
    s.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &s) != HAL_OK) Error_Handler();
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_Buffer2, FFT_LENGTH) != HAL_OK) Error_Handler(); }
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) Error_Handler();
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
}

/**
 * @brief 生成双音测试信号并同步写入ADC缓存
 */
#ifdef FFT_TEST_SIMULATION
static void Phase_GenerateSimulatedSignal(float32_t *signal, uint16_t length)
{
  static float32_t phase1 = 0.0f;
  static float32_t phase2 = 0.0f;
  const float32_t a1 = 0.80f;
  const float32_t a2 = 0.35f;
  const float32_t two_pi = 2.0f * PI;
  float32_t fs = FFT_SamplingRate_Hz;
  float32_t f1;
  float32_t f2;
  const uint16_t k1 = 50U;
  const uint16_t k2 = 120U;
  if (fs < 1.0f) fs = SAMPLING_RATE_DEFAULT;
  f1 = ((float32_t)k1 * fs) / (float32_t)length;
  f2 = ((float32_t)k2 * fs) / (float32_t)length;
  for (uint16_t i = 0; i < length; i++)
  {
    float32_t ac = a1 * arm_sin_f32(phase1) + a2 * arm_sin_f32(phase2);
    float32_t sample = 1.65f + ac;
    if (sample < 0.0f) sample = 0.0f;
    else if (sample > 3.3f) sample = 3.3f;
    signal[i] = ac;
    ADC_Buffer[i] = (uint16_t)((sample / Reference_Voltage) * (float32_t)ADC_MaxValue);
    phase1 += two_pi * f1 / fs;
    phase2 += two_pi * f2 / fs;
    if (phase1 >= two_pi) phase1 -= two_pi;
    if (phase2 >= two_pi) phase2 -= two_pi;
  }
}
#endif

static float32_t Phase_Get_TIM3_TriggerFreq_Hz(void)
{
  uint32_t timclk_hz;
  if ((TIM3->SMCR & TIM_SMCR_ECE) != 0U)
    timclk_hz = 1024000U;
  else
  {
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    uint32_t apb1_div = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    timclk_hz = pclk1_hz;
    if (apb1_div >= 4U) timclk_hz = pclk1_hz * 2U;
  }
  uint32_t psc = (uint32_t)(htim3.Init.Prescaler + 1U);
  uint32_t arr = (uint32_t)(htim3.Init.Period + 1U);
  return (float32_t)timclk_hz / (float32_t)(psc * arr);
}

float32_t Reference_Voltage = 3.3f;
volatile uint8_t ADC_COMPLETED = 0;
volatile int max_index_watch;
uint16_t ADC_Buffer[FFT_LENGTH] = {0};
volatile uint8_t ADC_Flag = 0;
static float32_t adc_float_buffer[FFT_LENGTH];
float32_t FFT_InputBuf[FFT_LENGTH * 2] = {0.0f};
float32_t FFT_OutputBuf[FFT_LENGTH] = {0.0f};
uint8_t ifftFlag = 0;
uint8_t doBitReverse = 1;
float32_t window;
static const arm_cfft_instance_f32 *CFFT_Instance = NULL;

/**
 * @brief 单通道FFT预处理 + 基波峰值搜索
 */
void Phase_FFT_OneChannel(const uint16_t *snap,
                                 float32_t *out_phase_deg,
                                 float32_t *out_mag_v,
                                 float32_t *out_freq_hz)
{
  uint64_t adc_sum = 0;
  for (uint32_t i = 0; i < FFT_LENGTH; i++) adc_sum += snap[i];
  float32_t adc_mean = (float32_t)adc_sum / (float32_t)FFT_LENGTH;
  for (uint32_t i = 0; i < FFT_LENGTH; i++) {
    float32_t centered = ((float32_t)snap[i] - adc_mean) / (float32_t)ADC_MaxValue;
    adc_float_buffer[i] = centered * Reference_Voltage;
  }
  { static float32_t win_coeff[FFT_LENGTH]; static uint8_t win_init_done = 0;
    if (!win_init_done) { hannWin(FFT_LENGTH, win_coeff); win_init_done = 1; }
    Window_Apply(adc_float_buffer, win_coeff, FFT_LENGTH); }
  arm_cfft_f32_app(adc_float_buffer, CFFT_Instance);
  { const float32_t scale = 2.0f / ((float32_t)FFT_LENGTH * 0.5f);
    for (uint32_t i = 0; i < FFT_LENGTH; i++) FFT_OutputBuf[i] *= scale; }
  float32_t bin_res = FFT_SamplingRate_Hz / (float32_t)FFT_LENGTH;
  uint32_t halfN = FFT_LENGTH / 2U;
  uint32_t best_bin = 2U;
  float32_t best_mag = FFT_OutputBuf[2U];
  for (uint32_t i = 3U; i < halfN; i++)
    if (FFT_OutputBuf[i] > best_mag) { best_mag = FFT_OutputBuf[i]; best_bin = i; }
  { uint32_t lo = (best_bin > HARM_BIN_HALF_WIDTH) ? best_bin - HARM_BIN_HALF_WIDTH : 1U;
    uint32_t hi = best_bin + HARM_BIN_HALF_WIDTH;
    if (hi >= halfN) hi = halfN - 1U;
    for (uint32_t i = lo; i <= hi; i++)
      if (FFT_OutputBuf[i] > best_mag) { best_mag = FFT_OutputBuf[i]; best_bin = i; } }
  float32_t delta = 0.0f;
  if (best_bin > 1U && best_bin < halfN - 1U) {
    float32_t A_prev = FFT_OutputBuf[best_bin - 1U];
    float32_t A_k    = FFT_OutputBuf[best_bin];
    float32_t A_next = FFT_OutputBuf[best_bin + 1U];
    if (A_k > 1e-12f) {
      if (A_next >= A_prev) delta = (2.0f * A_next - A_k) / (A_k + A_next);
      else delta = -(2.0f * A_prev - A_k) / (A_k + A_prev);
      if (delta > 0.5f) delta = 0.5f; else if (delta < -0.5f) delta = -0.5f;
    }
    if (fabsf(delta) > 1e-6f) {
      float32_t sin_d = sinf(PI * delta);
      if (fabsf(sin_d) > 1e-12f) {
        best_mag = A_k * (PI * delta) / sin_d * (1.0f - delta * delta);
        if (best_mag < 0.0f) best_mag = A_k;
      }
    }
  }
  float32_t re = FFT_InputBuf[2U * best_bin];
  float32_t im = FFT_InputBuf[2U * best_bin + 1U];
  *out_phase_deg = atan2f(im, re) * 180.0f / PI;
  *out_mag_v     = best_mag;
  *out_freq_hz   = ((float32_t)best_bin + delta) * bin_res;
}

/**
 * @brief 双通道相位差计算 (Rife-Hanning + DFT精修相位 + 跨功率谱法)
 */
static void Phase_ComputeDiff(void)
{
  static float32_t cplx1[FFT_LENGTH * 2];
  static float32_t cplx2[FFT_LENGTH * 2];
  static float32_t mag1[FFT_LENGTH];
  static float32_t mag2[FFT_LENGTH];
  static float32_t win[FFT_LENGTH];
  static float32_t sig1[FFT_LENGTH];
  static float32_t sig2[FFT_LENGTH];
  static uint8_t   win_ok = 0;
  static uint16_t snap1[FFT_LENGTH];
  static uint16_t snap2[FFT_LENGTH];

  const float32_t bin_res = FFT_SamplingRate_Hz / (float32_t)FFT_LENGTH;
  const uint32_t  halfN   = FFT_LENGTH / 2U;
  const float32_t scale   = 2.0f / ((float32_t)FFT_LENGTH * 0.5f);

  if (!win_ok) { hannWin(FFT_LENGTH, win); win_ok = 1; }

  __disable_irq();
  memcpy(snap1, (const void *)adc_snapshot,  sizeof(snap1));
  memcpy(snap2, (const void *)adc_snapshot2, sizeof(snap2));
  __enable_irq();

  /* CH1 FFT */
  { uint64_t sum = 0; uint32_t i;
    for (i = 0; i < FFT_LENGTH; i++) sum += snap1[i];
    float32_t mean = (float32_t)sum / (float32_t)FFT_LENGTH;
    for (i = 0; i < FFT_LENGTH; i++) {
      float32_t v = ((float32_t)snap1[i] - mean) / (float32_t)ADC_MaxValue * Reference_Voltage;
      float32_t wv = v * win[i];
      sig1[i] = wv;
      cplx1[2U*i] = wv; cplx1[2U*i + 1U] = 0.0f;
    }
    arm_cfft_f32(CFFT_Instance, cplx1, ifftFlag, doBitReverse);
    arm_cmplx_mag_f32(cplx1, mag1, FFT_LENGTH);
    for (i = 0; i < FFT_LENGTH; i++) mag1[i] *= scale; }

  /* CH2 FFT */
  { uint64_t sum = 0; uint32_t i;
    for (i = 0; i < FFT_LENGTH; i++) sum += snap2[i];
    float32_t mean = (float32_t)sum / (float32_t)FFT_LENGTH;
    for (i = 0; i < FFT_LENGTH; i++) {
      float32_t v = ((float32_t)snap2[i] - mean) / (float32_t)ADC_MaxValue * Reference_Voltage;
      float32_t wv = v * win[i];
      sig2[i] = wv;
      cplx2[2U*i] = wv; cplx2[2U*i + 1U] = 0.0f;
    }
    arm_cfft_f32(CFFT_Instance, cplx2, ifftFlag, doBitReverse);
    arm_cmplx_mag_f32(cplx2, mag2, FFT_LENGTH);
    for (i = 0; i < FFT_LENGTH; i++) mag2[i] *= scale; }

  /* 找峰 */
  uint32_t peak_bin = 2U;
  float32_t peak_mag = mag1[2U];
  for (uint32_t i = 3U; i < halfN; i++)
    if (mag1[i] > peak_mag) { peak_mag = mag1[i]; peak_bin = i; }
  { uint32_t lo = (peak_bin > HARM_BIN_HALF_WIDTH) ? peak_bin - HARM_BIN_HALF_WIDTH : 2U;
    uint32_t hi = peak_bin + HARM_BIN_HALF_WIDTH;
    if (hi >= halfN) hi = halfN - 1U;
    for (uint32_t i = lo; i <= hi; i++)
      if (mag1[i] > peak_mag) { peak_mag = mag1[i]; peak_bin = i; } }

  /* Rife插值 */
  float32_t delta = 0.0f;
  if (peak_bin > 1U && peak_bin < halfN - 1U) {
    float32_t A_k   = mag1[peak_bin];
    float32_t A_prev = mag1[peak_bin - 1U];
    float32_t A_next = mag1[peak_bin + 1U];
    if (A_k > 1e-12f) {
      if (A_next >= A_prev) delta = (2.0f * A_next - A_k) / (A_k + A_next);
      else delta = -(2.0f * A_prev - A_k) / (A_k + A_prev);
      if (delta > 0.5f) delta = 0.5f; else if (delta < -0.5f) delta = -0.5f;
    }
    if (delta > 0.5f && peak_bin < halfN - 1U) { peak_bin++; delta -= 1.0f; }
    else if (delta < -0.5f && peak_bin > 1U) { peak_bin--; delta += 1.0f; }
  }
  g_f0_hz = ((float32_t)peak_bin + delta) * bin_res;

  /* Rife幅值修正 CH1 */
  g_mag_ch1_raw = mag1[peak_bin];
  { float32_t d = delta;
    if (fabsf(d) > 1e-6f) { float32_t sin_d = sinf(PI * d);
      if (fabsf(sin_d) > 1e-12f) { g_mag_ch1 = mag1[peak_bin] * (PI * d) / sin_d * (1.0f - d * d);
        if (g_mag_ch1 < 0.0f) g_mag_ch1 = mag1[peak_bin]; }
      else g_mag_ch1 = mag1[peak_bin]; }
    else g_mag_ch1 = mag1[peak_bin]; }

  /* Rife幅值修正 CH2 */
  g_mag_ch2_raw = mag2[peak_bin];
  { if (fabsf(delta) > 1e-6f) { float32_t sin_d = sinf(PI * delta);
      if (fabsf(sin_d) > 1e-12f) { g_mag_ch2 = mag2[peak_bin] * (PI * delta) / sin_d * (1.0f - delta * delta);
        if (g_mag_ch2 < 0.0f) g_mag_ch2 = mag2[peak_bin]; }
      else g_mag_ch2 = mag2[peak_bin]; }
    else g_mag_ch2 = mag2[peak_bin];
    g_mag_ch2 *= CH2_AMP_CAL_SCALE; }

  /* DFT精修相位 + 跨功率谱法 */
  { float32_t omega = 2.0f * PI * g_f0_hz / FFT_SamplingRate_Hz;
    float32_t re1 = 0.0f, im1 = 0.0f, re2 = 0.0f, im2 = 0.0f;
    for (uint32_t n = 0; n < FFT_LENGTH; n++) {
      float32_t s1 = sig1[n]; float32_t s2 = sig2[n];
      float32_t theta = omega * (float32_t)n;
      float32_t c = cosf(theta); float32_t s = sinf(theta);
      re1 += s1 * c; im1 -= s1 * s;
      re2 += s2 * c; im2 -= s2 * s; }
    float32_t cross_real = re1 * re2 + im1 * im2;
    float32_t cross_imag = im1 * re2 - re1 * im2;
    float32_t raw_deg = atan2f(cross_imag, cross_real) * 180.0f / PI;

    /* 中值滤波 */
    { static float32_t buf[5] = {0};
      static uint8_t idx = 0;
      buf[idx] = raw_deg;
      idx = (idx + 1) % 5;
      float32_t sorted[5];
      for (uint8_t k = 0; k < 5; k++) sorted[k] = buf[k];
      for (uint8_t k = 0; k < 4; k++)
        for (uint8_t j = k + 1; j < 5; j++)
          if (sorted[j] < sorted[k]) { float32_t t = sorted[k]; sorted[k] = sorted[j]; sorted[j] = t; }
      raw_deg = sorted[2]; }

    /* 运行时零点校准 */
    if (g_cal_request) {
      g_phase_offset = raw_deg;
      g_cal_request = 0;
      { char msg[32];
        int len = snprintf(msg, sizeof(msg), "CAL:offset=%.2f\r\n", (double)raw_deg);
        if (len > 0) while (UART1_TxEnqueue((const uint8_t *)msg, (uint16_t)len) < 0) {} }
    }

    g_phase_diff_deg = raw_deg - g_phase_offset;
    while (g_phase_diff_deg >  180.0f) g_phase_diff_deg -= 360.0f;
    while (g_phase_diff_deg < -180.0f) g_phase_diff_deg += 360.0f; }
}

uint32_t ADC_Raw_Data[FFT_LENGTH] = {0};
uint16_t ADC_1_Value_DMA[FFT_LENGTH] = {0};
uint16_t ADC_2_Value_DMA[FFT_LENGTH] = {0};
float32_t ADC_1_Real_Value[FFT_LENGTH] = {0.0f};
float32_t ADC_2_Real_Value[FFT_LENGTH] = {0.0f};

void Phase_RequestCalibrate(void) { g_cal_request = 1; }

void FFT_App_Init(void)
{
    #if FFT_LENGTH == 64
        CFFT_Instance = &arm_cfft_sR_f32_len64;
    #elif FFT_LENGTH == 128
        CFFT_Instance = &arm_cfft_sR_f32_len128;
    #elif FFT_LENGTH == 256
        CFFT_Instance = &arm_cfft_sR_f32_len256;
    #elif FFT_LENGTH == 512
        CFFT_Instance = &arm_cfft_sR_f32_len512;
    #elif FFT_LENGTH == 1024
        CFFT_Instance = &arm_cfft_sR_f32_len1024;
    #elif FFT_LENGTH == 2048
        CFFT_Instance = &arm_cfft_sR_f32_len2048;
    #elif FFT_LENGTH == 4096
        CFFT_Instance = &arm_cfft_sR_f32_len4096;
    #else
        #error "Unsupported FFT Length!"
    #endif

#ifdef FFT_TEST_SIMULATION
    FFT_SamplingRate_Hz = SAMPLING_RATE_DEFAULT;
    ADC_Flag = 1;
    return;
#endif

#if PHASE_CLOCK_SOURCE == PHASE_CLK_INTERNAL
    { uint32_t tim_clk = HAL_RCC_GetPCLK1Freq() * 2U;
      uint32_t period = (uint32_t)((float)tim_clk / PHASE_TARGET_FS + 0.5f);
      if (period < 2U) period = 2U;
      htim3.Init.Period = period - 1U;
      TIM3->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_ECE); }
#else
    { uint32_t period = (uint32_t)(1024000.0f / PHASE_TARGET_FS + 0.5f);
      if (period < 2U) period = 2U;
      htim3.Init.Period = period - 1U;
      TIM3->SMCR |= TIM_SMCR_ECE; }
#endif
    __HAL_TIM_SET_AUTORELOAD(&htim3, htim3.Init.Period);
    FFT_SamplingRate_Hz = Phase_Get_TIM3_TriggerFreq_Hz();
    Phase_StartAcq();
}

void FFT_SendSpectrumFrame(void)
{
  if (g_frame_ready == 0U) return;
  g_transmitting = 1U;
  g_frame_ready = 0U;
  float32_t ph, mag, f0;
  Phase_FFT_OneChannel(adc_snapshot, &ph, &mag, &f0);
  {
    /* 计算ADC原始值范围，诊断信号是否到达 */
    uint16_t adc_min = 65535, adc_max = 0;
    for (uint32_t i = 0; i < FFT_LENGTH; i++) {
      if (adc_snapshot[i] < adc_min) adc_min = adc_snapshot[i];
      if (adc_snapshot[i] > adc_max) adc_max = adc_snapshot[i];
    }
    int32_t fs_int = (int32_t)FFT_SamplingRate_Hz;
    uint32_t fs_frac = (uint32_t)((FFT_SamplingRate_Hz - (float)fs_int) * 100.0f + 0.5f);
    int32_t  f0_i = (int32_t)f0;
    uint32_t f0_f = (uint32_t)((f0 - (float32_t)f0_i) * 10.0f + 0.5f);
    if (f0_f >= 10U) { f0_i++; f0_f = 0U; }
    char header[100];
    int header_len = snprintf(header, sizeof(header),
      "FFT_BEGIN,Fs=%ld.%02lu,N=%u,F0=%ld.%01lu,ADC=%u-%u\r\n",
      (long)fs_int, (unsigned long)fs_frac, (unsigned)FFT_LENGTH,
      (long)f0_i, (unsigned long)f0_f,
      (unsigned)adc_min, (unsigned)adc_max);
    if (header_len > 0) while (UART1_TxEnqueue((const uint8_t *)header, (uint16_t)header_len) < 0) {} }
  { char line[64]; uint32_t halfN = FFT_LENGTH / 2U;
    for (uint32_t i = 0; i < halfN; i++) {
      float val = FFT_OutputBuf[i]; if (val < 0.0f) val = 0.0f;
      int32_t int_part = (int32_t)val;
      uint32_t frac_part = (uint32_t)((val - (float)int_part) * 1000000.0f + 0.5f);
      if (frac_part >= 1000000U) { int_part++; frac_part = 0U; }
      int len = snprintf(line, sizeof(line), "%lu,%ld.%06lu\r\n",
                         (unsigned long)i, (long)int_part, (unsigned long)frac_part);
      if (len > 0) while (UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len) < 0) {} } }
  { static const char tail[] = "FFT_END\r\n";
    while (UART1_TxEnqueue((const uint8_t *)tail, (uint16_t)(sizeof(tail) - 1U)) < 0) {} }
  g_transmitting = 0U;
}

void FFT_SendPhaseDiffFrame(void)
{
  if (g_phase_ready == 0U) return;
  g_phase_ready = 0U;
  Phase_ComputeDiff();
  { char line[128];
    int len = snprintf(line, sizeof(line),
      "PDIFF,%.1f,%.2f,%.4f,%.4f,raw1=%.4f,raw2=%.4f\r\n",
      (double)g_f0_hz,(double)g_phase_diff_deg,
      (double)g_mag_ch1,(double)g_mag_ch2,
      (double)g_mag_ch1_raw,(double)g_mag_ch2_raw);
    if(len>0) while(UART1_TxEnqueue((const uint8_t*)line,(uint16_t)len)<0){} }
}

void FFT_App_Process(void)
{
#ifdef FFT_TEST_SIMULATION
    static uint32_t last_sim_tick = 0;
    if (HAL_GetTick() - last_sim_tick < 1000) return;
    last_sim_tick = HAL_GetTick();
    Phase_GenerateSimulatedSignal(adc_float_buffer, FFT_LENGTH);
    arm_cfft_f32_app(adc_float_buffer, CFFT_Instance);
    return;
#endif
    (void)ADC_Flag;
}

void arm_cfft_f32_app(float32_t *rawData, const arm_cfft_instance_f32 *fft_instance)
{
  uint16_t n; uint16_t fftLen = fft_instance->fftLen;
  for (n = 0; n < fftLen; n++) { FFT_InputBuf[2*n] = rawData[n]; FFT_InputBuf[2*n+1] = 0.0f; }
  arm_cfft_f32(fft_instance, FFT_InputBuf, ifftFlag, doBitReverse);
  arm_cmplx_mag_f32(FFT_InputBuf, FFT_OutputBuf, fftLen);
}

void Apply_Hanning_Window(float32_t *signal, uint16_t length)
{ for(uint16_t i = 0; i < length; i++) { window = 0.5f*(1.0f - cosf(2.0f*3.1415926f*i/(length-1))); signal[i] *= window; } }

void Apply_FlatTop_Window(float32_t *signal, uint16_t length)
{ const float32_t a0=0.21557895f,a1=0.41663158f,a2=0.277263158f,a3=0.083578947f;
  float32_t d=(float32_t)(length-1);
  for(uint16_t i=0;i<length;i++){float32_t r=(float32_t)i/d;float32_t t=2.0f*3.1415926f*r;
  window=a0-a1*cosf(t)+a2*cosf(2.0f*t)-a3*cosf(3.0f*t);signal[i]*=window;} }

void PhaseCalculate_ADC_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{ HAL_ADC_Start(hadc2);
  HAL_ADC_Start_DMA(hadc1, (uint32_t *)ADC_1_Value_DMA, FFT_LENGTH);
  HAL_ADC_Start_DMA(hadc2, (uint32_t *)ADC_2_Value_DMA, FFT_LENGTH); }

int Find_nMax(const float *ARR)
{ if(ARR==NULL)return -1; float aMax=ARR[80]; uint32_t nMax=80;
  for(uint32_t i=81;i<FFT_LENGTH/2;i++) if(ARR[i]>aMax){aMax=ARR[i];nMax=i;}
  return nMax; }

float32_t Find_PhaseAngle(float32_t *signal)
{ arm_cfft_f32_app(signal, &arm_cfft_sR_f32_len1024);
  int n_max=Find_nMax(FFT_OutputBuf);
  return atan2f(FFT_InputBuf[2*n_max+1],FFT_InputBuf[2*n_max])*180.0f/PI; }

void Process_ADC_RawData(void)
{ if(ADC_COMPLETED){ADC_COMPLETED=0;
  for(int i=0;i<FFT_LENGTH;i++){
    ADC_1_Real_Value[i]=((float32_t)ADC_1_Value_DMA[i]/(float32_t)ADC_MaxValue)*Reference_Voltage;
    ADC_2_Real_Value[i]=((float32_t)ADC_2_Value_DMA[i]/(float32_t)ADC_MaxValue)*Reference_Voltage;} } }

float32_t Get_PhaseDifference(void)
{ float32_t p1,p2,d; Process_ADC_RawData();
  p1=Find_PhaseAngle(ADC_1_Real_Value); p2=Find_PhaseAngle(ADC_2_Real_Value);
  d=p1-p2; if(d>180.0f)d-=360.0f; else if(d<-180.0f)d+=360.0f; return d; }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) g_adc1_done = 1;
  else if (hadc->Instance == ADC2) g_adc2_done = 1;
  if (g_adc1_done && g_adc2_done) {
    CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN);
    memcpy(adc_snapshot,  ADC_Buffer,  sizeof(adc_snapshot));
    memcpy(adc_snapshot2, ADC_Buffer2, sizeof(adc_snapshot2));
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Buffer,  FFT_LENGTH);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC_Buffer2, FFT_LENGTH);
    SET_BIT(TIM3->CR1, TIM_CR1_CEN);
    g_cb_count++; g_adc1_done = 0; g_adc2_done = 0;
    g_frame_ready = 1; g_phase_ready = 1;
  }
}

uint16_t Get_FFT_Spectrum(float32_t* buffer, uint16_t length)
{ uint16_t cl=(length<FFT_LENGTH)?length:FFT_LENGTH;
  memcpy(buffer,FFT_OutputBuf,cl*sizeof(float32_t)); return cl; }

uint16_t Phase_Get_PeakFreq(Phase_PeakInfo_t *peaks, uint16_t max_peaks)
{
    if (peaks == NULL || max_peaks == 0) return 0;

    /* 搜索范围: bin [1, FFT_LENGTH/2 - 1], 跳过 DC 和 Nyquist */
    const uint16_t half = FFT_LENGTH / 2U;
    const float32_t bin_res = SAMPLING_RATE / (float32_t)FFT_LENGTH;

    /* 临时峰值列表 (栈分配, 最多存 half 个) */
    uint16_t count = 0;
    for (uint16_t i = 1; i < half - 1U; i++) {
        float32_t cur = FFT_OutputBuf[i];
        if (cur > FFT_OutputBuf[i - 1U] && cur > FFT_OutputBuf[i + 1U]) {
            /* 找到峰值, 按幅值降序插入 peaks */
            uint16_t pos = count;
            while (pos > 0 && cur > peaks[pos - 1U].mag) {
                if (pos < max_peaks) peaks[pos] = peaks[pos - 1U];
                pos--;
            }
            if (pos < max_peaks) {
                peaks[pos].freq_hz = (float32_t)i * bin_res;
                peaks[pos].mag     = cur;
                peaks[pos].bin     = i;
            }
            if (count < max_peaks) count++;
        }
    }

    return (count < max_peaks) ? count : max_peaks;
}

void Set_Reference_Voltage(float32_t voltage)
{ if(voltage>0.0f&&voltage<=5.0f) Reference_Voltage=voltage; }

float32_t Get_Reference_Voltage(void) { return Reference_Voltage; }

void ADC_Signal_Collect_To_ADC_Buffer(void) { }

void FFT_IFFT_SendWaveform(void)
{
  static uint32_t last_cb = 0;
  uint32_t cb = g_cb_count;
  if (cb == last_cb) return;
  last_cb = cb;
  uint16_t snap[FFT_LENGTH]; float32_t wf[FFT_LENGTH]; float32_t cb2[FFT_LENGTH*2];
  memcpy(snap, adc_snapshot, sizeof(snap));
  for(uint32_t i=0;i<FFT_LENGTH;i++) wf[i]=(float32_t)snap[i]/(float32_t)ADC_MaxValue*Reference_Voltage;
  for(uint32_t i=0;i<FFT_LENGTH;i++){cb2[2*i]=wf[i];cb2[2*i+1]=0.0f;}
  arm_cfft_f32(CFFT_Instance,cb2,0,1); arm_cfft_f32(CFFT_Instance,cb2,1,1);
  { int32_t fi=(int32_t)FFT_SamplingRate_Hz;
    uint32_t ff=(uint32_t)((FFT_SamplingRate_Hz-(float)fi)*100.0f+0.5f);
    char line[64]; int len;
    len=snprintf(line,sizeof(line),"WAVE_BEGIN,Fs=%ld.%02lu,N=%u\r\n",(long)fi,(unsigned long)ff,(unsigned)FFT_LENGTH);
    if(len>0)while(UART1_TxEnqueue((const uint8_t*)line,(uint16_t)len)<0){}
    for(uint32_t i=0;i<FFT_LENGTH;i+=4){float32_t v=cb2[2*i];int32_t ip=(int32_t)v;
      uint32_t fp=(uint32_t)((v-(float)ip)*1000000.0f+0.5f);if(fp>=1000000U){ip++;fp=0U;}
      len=snprintf(line,sizeof(line),"%lu,%ld.%06lu\r\n",(unsigned long)i,(long)ip,(unsigned long)fp);
      if(len>0)while(UART1_TxEnqueue((const uint8_t*)line,(uint16_t)len)<0){}}
    static const char tail[]="WAVE_END\r\n";
    while(UART1_TxEnqueue((const uint8_t*)tail,(uint16_t)(sizeof(tail)-1U))<0){} }
}
