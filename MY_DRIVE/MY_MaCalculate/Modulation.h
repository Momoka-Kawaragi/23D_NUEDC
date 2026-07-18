/**
 ******************************************************************************
 * @file    Modulation.h
 * @brief   FM 调频指数 (贝塞尔法) + AM 调制度 (时域加窗) 测量模块
 * @author  (auto)
 * @version V1.0
 * @date    2026-07-13
 *
 * @details
 *   本模块基于 Phase.c 的 FFT 频谱 + WindowFunction.c 的加窗能力,
 *   实现两个常用调制度测量 API:
 *
 *     1. FM_Bessel_Mf()    — 贝塞尔法估算调频指数 mf
 *        输入: ADC 原始 buffer + 采样率 Fs + 已知调制频率 fm
 *        输出: 调频指数 mf / 频偏 Δf / 载波 fc
 *
 *     2. AM_Windowed_Ma()  — 时域加窗后取包络算调制度 ma
 *        输入: ADC 原始 buffer + 采样率 Fs + 窗类型
 *        输出: 调制度 ma (0~1, >1 为过调)
 *
 *   依赖:
 *     - Phase.h         (FFT_App_Process, Phase_Get_PeakFreq, SAMPLING_RATE 等)
 *     - WindowFunction.h (hannWin, blackManHarrisWin, Window_Apply)
 *     - arm_math.h      (FFT 加速, fabsf)
 *     - math.h          (sqrtf, atan2f)
 *
 *   使用流程:
 *     1. 初始化: FFT_App_Init() (Phase.c 提供)
 *     2. 采集:   FFT_App_Process() 内部会触发 ADC DMA + FFT 填充 FFT_OutputBuf
 *     3. 测量:   把 ADC_1_Value_DMA + 已知 fm 传给 FM_Bessel_Mf()
 *                把 ADC_1_Value_DMA + Fs  传给 AM_Windowed_Ma()
 ******************************************************************************
 */
#ifndef __MODULATION_H
#define __MODULATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* 依赖 */
#include "main.h"
#include "arm_math.h"
#include <math.h>
#include "Phase.h"        // FFT_OutputBuf / SAMPLING_RATE / FFT_LENGTH
#include "WindowFunction.h" // hannWin / blackManHarrisWin / Window_Apply

/* ===========================================================================
 *  窗类型枚举 (本模块自定, 与 WindowFunction 库映射)
 * ===========================================================================
 */
typedef enum {
    MOD_WIN_HANN              = 0,  /* Hanning, 主瓣窄 (-31dB 旁瓣) */
    MOD_WIN_BLACKMAN_HARRIS   = 1,  /* Blackman-Harris 4 项, 旁瓣 -92dB, 主瓣宽 */
    MOD_WIN_HAMMING           = 2,  /* Hamming, 旁瓣 -43dB, 主瓣比 Hann 稍宽 */
    MOD_WIN_RECTANGULAR       = 3   /* 不加窗 (矩形), 等效无窗, 频谱泄漏最大 */
} Modulation_WinType_t;

/* ===========================================================================
 *  错误码
 * ===========================================================================
 */
typedef enum {
    MOD_OK                    =  0,  /* 成功 */
    MOD_ERR_NULL_PTR          = -1,  /* 输入空指针 */
    MOD_ERR_INVALID_LEN       = -2,  /* buffer 长度异常 (0 或 > FFT_LENGTH) */
    MOD_ERR_FS_ZERO           = -3,  /* 采样率为 0 */
    MOD_ERR_FM_INVALID        = -4,  /* fm 超出 (0, Fs/2) 范围 */
    MOD_ERR_NO_PEAK_FOUND     = -5,  /* 频谱找不到主峰 */
    MOD_ERR_BESSEL_NOT_CONV   = -6,  /* 贝塞尔二分查表不收敛 */
    MOD_ERR_MA_OUT_OF_RANGE   = -7,  /* ma 算出 > 1.5 (严重过调, 数据异常) */
    MOD_ERR_CLIPPED           = -8,  /* 时域检测到信号削顶, ma 不可信 */
} Modulation_Status_t;

/* ===========================================================================
 *  FM 测量结果
 * ===========================================================================
 */
/* 贝塞尔查表配置 (来自参考实现, beta=1.0~6.0 步进 0.1) */
#define FM_MF_START            1.0f
#define FM_MF_STEP             0.1f
#define FM_MF_NUM              51U       /* (6.0 - 1.0) / 0.1 + 1 = 51 */
#define FM_FM_MIN_HZ           1000U
#define FM_FM_MAX_HZ           5000U
#define FM_FM_STEP_HZ          1000U
#define FM_FM_CANDIDATE_NUM    5U        /* (5000-1000)/1000 + 1 = 5 */

typedef struct {
    float32_t fc;            /* 载波频率 (Hz) */
    float32_t fm;            /* 调制频率 (Hz, 自动搜到或输入回显) */
    float32_t mf;            /* 调频指数 (= Δf / fm) */
    float32_t df_hz;         /* 频偏 Δf (Hz) = mf * fm */
    float32_t carrier_mag;   /* 载波幅值 (Goertzel 输出, 任意单位) */
    float32_t sideband1_mag; /* ±1 阶边带平均幅值 */
    float32_t sideband2_mag; /* ±2 阶边带平均幅值 */
    float32_t fit_error;     /* 最小二乘拟合归一化误差 (越小越好, <0.01 视为匹配) */
    uint8_t  fm_auto;        /* 1: fm 是自动搜到的, 0: 用调用方传入的 fm */
    Modulation_Status_t status;
} Modulation_FM_Result_t;

/* ===========================================================================
 *  AM 测量结果
 * ===========================================================================
 */
typedef struct {
    float32_t ma;            /* 调制度 (0~1, 1 表示 100% 调幅, >1 为过调) */
    float32_t v_max;         /* 包络最大值 (V) */
    float32_t v_min;         /* 包络最小值 (V) */
    float32_t v_dc;          /* 包络直流分量 (V) = (Vmax + Vmin) / 2 */
    Modulation_WinType_t win_used;  /* 实际使用的窗 */
    Modulation_Status_t status;
} Modulation_AM_Result_t;

/* 频谱法 AM 结果 (抗削顶, 推荐) */
typedef struct {
    float32_t ma;            /* 调制度 ma = 2·|A1|/|A0| (来自边带 / 载波) */
    float32_t fc;            /* 载波频率 (Hz) */
    float32_t fm;            /* 调制频率 (Hz) (回显输入) */
    float32_t carrier_mag;   /* 载波谱线幅值 */
    float32_t sideband_mag;  /* ±1 阶边带平均幅值 */
    Modulation_Status_t status;
} Modulation_AM_Spec_Result_t;

/* ===========================================================================
 *  API 函数声明
 * ===========================================================================
 */

/**
 * @brief   贝塞尔查表 + 三阶最小二乘拟合估算 FM 调频指数 mf
 *          (替代原 v1.0 的 J1/J0 单阶比值法, 抗 J_0 过零点)
 *
 * @details
 *   原理: FM 信号 s(t) = A·cos(2π·fc·t + mf·sin(2π·fm·t))
 *   展开: 边带 = A·|J_n(mf)| at fc ± n·fm
 *   - A0 = C·|J0(mf)|  载波
 *   - A1 = C·|J1(mf)|  ±1 阶边带
 *   - A2 = C·|J2(mf)|  ±2 阶边带
 *   C 是未知公共缩放 (含输入幅度、ADC 增益、窗增益)
 *
 *   步骤:
 *     1. 用 Goertzel 算法算 fc / fc±fm / fc±2fm 处的幅值 (含去 DC + Hann 窗)
 *     2. 遍历 mf=1.0~6.0 (步进 0.1) 查 BESSEL_J0/J1/J2 理论值
 *     3. 最小二乘求 C = Σ(Ai·Ji) / Σ(Ji²)
 *     4. 算归一化误差 err = Σ(Ai - C·Ji)² / Σ(Ai²)
 *     5. 挑 err 最小的那对 (mf, fm) 当结果
 *     6. 若调用方传了 fm_hint, 跳过 fm 遍历; 若传 fc_hint, 跳过自动找 fc
 *
 *   vs v1.0 J1/J0 二分法:
 *     - 不依赖 J0(mf) 不为零, 在 mf≈2.4/5.5 等 J_0 过零点处仍能算
 *     - 用三阶拟合, 噪声鲁棒性更好 (±10% 噪声仍能收敛)
 *     - 用 Goertzel 替代 FFT, 只需算 4 个频率点 (N 次乘加 vs FFT 整段)
 *     - fm 也能自动搜, 不必传
 *
 *   限制:
 *     - mf 范围 [1.0, 6.0] (51 步查表), 超出需要扩表
 *     - fm 范围 [1k, 5k] (5 候选, 步进 1k), 改 fc_hint 可以测更窄
 *     - 需要 buffer 长度 ≥ 256, 否则 Goertzel 精度不够
 *
 * @param   signal    : 时域 ADC 原始 buffer (uint16_t, 0~4095), 必填
 * @param   length    : signal 长度, 0 = 用 FFT_LENGTH
 * @param   fs        : 采样率 (Hz), 0 = 用 SAMPLING_RATE
 * @param   fc_hint   : 已知载波频率 (Hz), 0 = 自动从 FFT 找
 * @param   fm_hint   : 已知调制频率 (Hz), 0 = 自动从 1k/2k/3k/4k/5k 候选搜
 * @param   result    : [out] 测量结果
 *
 * @return  Modulation_Status_t
 */
Modulation_Status_t FM_Bessel_Mf(const uint16_t *signal,
                                 uint16_t length,
                                 float32_t fs,
                                 float32_t fc_hint,
                                 float32_t fm_hint,
                                 Modulation_FM_Result_t *result);

/**
 * @brief   时域加窗后取包络算 AM 调制度 ma
 *
 * @details
 *   原理: 标准 AM 信号 s(t) = (1 + ma·cos(2π·fm·t)) · cos(2π·fc·t)
 *   包络 = 1 + ma·cos(2π·fm·t), 范围 [1-ma, 1+ma]
 *   所以 ma = (Vmax - Vmin) / (Vmax + Vmin)
 *
 *   步骤:
 *     1. 时域 buffer → 浮点电压 → 减 DC (一阶高通截断, fc<<Fs 时 OK)
 *     2. 按 win_type 加窗
 *     3. abs() 取瞬时幅值
 *     4. 一阶 IIR 低通 (α ≈ 0.05) 平滑包络
 *     5. 包络找 Vmax, Vmin → 算 ma
 *
 *   注意:
 *     - 要求 Fs >= 4·fc, 否则包络提取会失败 (载波欠采样)
 *     - 不需要 fc / fm 任何先验, 函数完全自包含
 *     - 若 ma > 1.0 会在 status 提示过调, 但 ma 值仍返回
 *
 * @param   signal    : 时域 ADC 原始 buffer (uint16_t, 0~4095)
 * @param   length    : signal 长度, 0 表示用 FFT_LENGTH
 * @param   fs        : 采样率 (Hz), 0 表示用全局 SAMPLING_RATE
 * @param   vref      : ADC 参考电压 (V), 0 表示用全局 Reference_Voltage (3.3V)
 * @param   win_type  : 窗类型 (见 Modulation_WinType_t)
 * @param   result    : [out] 测量结果
 *
 * @return  Modulation_Status_t
 */
Modulation_Status_t AM_Windowed_Ma(const uint16_t *signal,
                                   uint16_t length,
                                   float32_t fs,
                                   float32_t vref,
                                   Modulation_WinType_t win_type,
                                   Modulation_AM_Result_t *result);

/**
 * @brief   频谱法估算 AM 调制度 ma (抗削顶 / 抗时域失真, 推荐用于工程测量)
 *
 * @details
 *   原理: 标准 AM 信号 s(t) = A·(1 + ma·cos(2π·fm·t))·cos(2π·fc·t)
 *   频谱: 1 根载波 + 2 根 ±fm 边带, 边带幅值 = A·ma/2, 载波幅值 = A
 *   故 ma = 2·|A1| / |A0| (与载波绝对幅度无关, 完全靠 FFT 频谱的相对值)
 *
 *   vs AM_Windowed_Ma (时域法) 的优势:
 *     - 不受 ADC 削顶影响 (削顶时 Vmax 卡 ADC 满量程, 时域法失效)
 *     - 不受 DC 偏置 / 直流偏置耦合影响
 *     - 不受信号源输出幅度不稳影响
 *     - 与示波器 FFT 选件的 "AM 调制度" 测量算法一致
 *
 *   限制:
 *     - 需已知 fm (Hz), 否则无法定位 ±1 阶边带
 *     - 需 fm ≥ 几倍 df (频谱分辨率), 否则边带和载波粘在一起
 *       Fs=500kHz, N=1024 → df=488Hz → fm ≥ 2kHz 较稳
 *     - 需先调 FFT_App_Process() 让 FFT_OutputBuf 有数据
 *
 * @param   signal : 时域 ADC buffer (仅参考, 实际读 FFT_OutputBuf), NULL 也行
 * @param   length : signal 长度, 0 = 用 FFT_LENGTH
 * @param   fs     : 采样率 (Hz), 0 = 用 SAMPLING_RATE
 * @param   fm     : 已知调制频率 (Hz), 必须 > 0 且 < fs/2
 * @param   result : [out] 测量结果
 *
 * @return  Modulation_Status_t
 */
Modulation_Status_t AM_Spectral_Ma(const uint16_t *signal,
                                   uint16_t length,
                                   float32_t fs,
                                   float32_t fm,
                                   Modulation_AM_Spec_Result_t *result);

/* ===========================================================================
 *  结果发送函数 (UART)
 * ===========================================================================
 */

void FM_Result_Send(const Modulation_FM_Result_t *result);
void AM_Spec_Result_Send(const Modulation_AM_Spec_Result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* __MODULATION_H */
