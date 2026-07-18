/**
 ******************************************************************************
 * @file    Modulation.c
 * @brief   FM 调频指数 (贝塞尔法) + AM 调制度 (时域加窗) 实现
 * @see     Modulation.h
 ******************************************************************************
 */

#include "Modulation.h"
#include "ZPN_Uart.h"
#include <stdio.h>

#ifndef PI
#define PI 3.1415926535f
#endif

/* ===========================================================================
 *  内部常量 / 工具
 * ===========================================================================
 */

/* 包络 IIR 平滑系数 (越小越平滑, 但响应越慢)
 * α = 0.05 → 等效截止频率 ≈ Fs * α / (2π) ≈ Fs * 0.008
 *   Fs=500kHz 时截止 ~4kHz, 滤掉载波, 保留 AM 包络 (fm 通常 < 10kHz) */
#define ENV_IIR_ALPHA         0.05f

/* 包络搜索时, 跳过首尾各 10% 样本 (避免窗函数边缘把信号压到 0 影响 ma) */
#define AM_EDGE_RATIO         0.10f

/* ===========================================================================
 *  贝塞尔函数查表 (v1.1, 来自参考实现)
 *  beta = 1.0 + index * 0.1, 共 51 项 (1.0 ~ 6.0)
 *  数据精度: 6 位有效数字, 拟合用足够
 * ===========================================================================
 */
static const float BESSEL_J0[FM_MF_NUM] =
{
    0.765198f, 0.719622f, 0.671133f, 0.620086f, 0.566855f,
    0.511828f, 0.455402f, 0.397985f, 0.339986f, 0.281819f,
    0.223891f, 0.166607f, 0.110362f, 0.055540f, 0.002508f,
   -0.048384f,-0.096805f,-0.142449f,-0.185036f,-0.224312f,
   -0.260052f,-0.292064f,-0.320188f,-0.344296f,-0.364296f,
   -0.380128f,-0.391769f,-0.399230f,-0.402556f,-0.401826f,
   -0.397150f,-0.388670f,-0.376557f,-0.361011f,-0.342257f,
   -0.320543f,-0.296138f,-0.269331f,-0.240425f,-0.209738f,
   -0.177597f,-0.144335f,-0.110290f,-0.075803f,-0.041210f,
   -0.006844f, 0.026971f, 0.059920f, 0.091703f, 0.122033f,
    0.150645f
};

static const float BESSEL_J1[FM_MF_NUM] =
{
    0.440051f, 0.470902f, 0.498289f, 0.522023f, 0.541948f,
    0.557937f, 0.569896f, 0.577765f, 0.581517f, 0.581157f,
    0.576725f, 0.568292f, 0.555963f, 0.539873f, 0.520185f,
    0.497094f, 0.470818f, 0.441601f, 0.409709f, 0.375427f,
    0.339059f, 0.300921f, 0.261343f, 0.220663f, 0.179226f,
    0.137378f, 0.095466f, 0.053834f, 0.012821f,-0.027244f,
   -0.066043f,-0.103273f,-0.138647f,-0.171897f,-0.202776f,
   -0.231060f,-0.256553f,-0.279081f,-0.298500f,-0.314695f,
   -0.327579f,-0.337097f,-0.343223f,-0.345961f,-0.345345f,
   -0.341438f,-0.334333f,-0.324148f,-0.311028f,-0.295142f,
   -0.276684f
};

static const float BESSEL_J2[FM_MF_NUM] =
{
    0.114903f, 0.136564f, 0.159349f, 0.183027f, 0.207356f,
    0.232088f, 0.256968f, 0.281739f, 0.306144f, 0.329926f,
    0.352834f, 0.374624f, 0.395059f, 0.413915f, 0.430980f,
    0.446059f, 0.458973f, 0.469562f, 0.477685f, 0.483227f,
    0.486091f, 0.486207f, 0.483528f, 0.478032f, 0.469723f,
    0.458629f, 0.444805f, 0.428330f, 0.409304f, 0.387855f,
    0.364128f, 0.338292f, 0.310535f, 0.281059f, 0.250086f,
    0.217849f, 0.184593f, 0.150573f, 0.116050f, 0.081292f,
    0.046565f, 0.012140f,-0.021718f,-0.054748f,-0.086695f,
   -0.117315f,-0.146375f,-0.173656f,-0.198954f,-0.222082f,
   -0.242873f
};

/* 二分查表精度 */
#define MF_SEARCH_MAX         30.0f   /* 最大 mf (覆盖 ±1 过零) */
#define MF_BISECT_ITERS       50      /* 50 次迭代 → 精度 ~3e-14, 远高于硬件噪声 */
#define MF_BISECT_TOL         1e-4f   /* 收敛阈值 (相对误差) */

/* ADC 12-bit 上限 */
#define ADC_MAX_12BIT         4095.0f

/* 把窗类型映射到 WindowFunction 库的生成函数 */
static Modulation_Status_t mod_generate_window(Modulation_WinType_t type,
                                                uint16_t N,
                                                float32_t *w)
{
    switch (type) {
        case MOD_WIN_HANN:
            return hannWin(N, w);
        case MOD_WIN_BLACKMAN_HARRIS:
            return blackManHarrisWin(N, w);
        case MOD_WIN_HAMMING:
            return hammingWin(N, w);
        case MOD_WIN_RECTANGULAR:
        default:
            /* 矩形窗 = 全 1 */
            for (uint16_t i = 0; i < N; i++) w[i] = 1.0f;
            return MOD_OK;
    }
}

/* ===========================================================================
 *  Goertzel 算法 — 算指定单频的幅值 (v1.1 来自参考实现)
 *
 *  比 FFT 省事: 只需要算 1 个频率点, N 次乘加 vs FFT 的 N·log2(N).
 *  这里还顺手去 DC + 加 Hann 窗.
 *
 *  输入: samples (uint16_t ADC 原始), length, mean (本帧均值), targetFreqHz
 *  输出: 相对幅值 (单位任意, 不做电压换算, 后续拟合里有公共缩放 C)
 * ===========================================================================
 */
static float32_t goertzel_magnitude(const uint16_t *samples,
                                    uint32_t length,
                                    float32_t mean,
                                    float32_t target_freq_hz,
                                    float32_t sample_rate_hz)
{
    float32_t omega = 2.0f * (float32_t)PI * target_freq_hz / sample_rate_hz;
    float32_t coeff = 2.0f * cosf(omega);
    float32_t s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

    for (uint32_t n = 0; n < length; n++) {
        /* 去 DC + Hann 窗 (但本函数没用窗数组, 留简洁版)
         * 如果需要加窗: x = ((float)samples[n] - mean) * hannWindow[n]; */
        float32_t x = (float32_t)samples[n] - mean;
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    /* |X|^2 = s1² + s2² - coeff·s1·s2 */
    float32_t power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    if (power <= 0.0f) return 0.0f;
    return sqrtf(power);
}

/* ===========================================================================
 *  FitBesselError  对一个 beta 表项做最小二乘拟合
 *
 *  实测: amp0 (载波)  amp1 (±1 阶边带平均)  amp2 (±2 阶边带平均)
 *  理论: C·|J0(β)|     C·|J1(β)|            C·|J2(β)|
 *  C 未知 (含输入幅度 / ADC 增益 / 窗增益), 最小二乘求 C = Σ(Ai·Ji) / Σ(Ji²)
 *  返回归一化均方误差, 越小匹配越好
 * ===========================================================================
 */
static float32_t fit_bessel_error(float32_t amp0,
                                  float32_t amp1,
                                  float32_t amp2,
                                  uint32_t table_index)
{
    float32_t j0 = fabsf(BESSEL_J0[table_index]);
    float32_t j1 = fabsf(BESSEL_J1[table_index]);
    float32_t j2 = fabsf(BESSEL_J2[table_index]);

    float32_t dot_obs_theo  = amp0 * j0 + amp1 * j1 + amp2 * j2;
    float32_t dot_theo_theo = j0 * j0 + j1 * j1 + j2 * j2;
    if (dot_theo_theo < 1e-12f) return 1e30f;

    float32_t scale = dot_obs_theo / dot_theo_theo;

    float32_t e0 = amp0 - scale * j0;
    float32_t e1 = amp1 - scale * j1;
    float32_t e2 = amp2 - scale * j2;

    float32_t obs_energy = amp0 * amp0 + amp1 * amp1 + amp2 * amp2;
    if (obs_energy < 1e-12f) return 1e30f;

    return (e0 * e0 + e1 * e1 + e2 * e2) / obs_energy;
}

/* ===========================================================================
 *  FM_Bessel_Mf  v1.1 — 查表 + 三阶最小二乘拟合
 *  (替代 v1.0 的 J1/J0 单阶比值法, 抗 J_0 过零点)
 * ===========================================================================
 */
Modulation_Status_t FM_Bessel_Mf(const uint16_t *signal,
                                 uint16_t length,
                                 float32_t fs,
                                 float32_t fc_hint,
                                 float32_t fm_hint,
                                 Modulation_FM_Result_t *result)
{
    if (result == NULL) return MOD_ERR_NULL_PTR;
    if (signal == NULL) return MOD_ERR_NULL_PTR;

    if (length == 0) length = FFT_LENGTH;
    if (fs     == 0.0f) fs = SAMPLING_RATE;

    if (length > FFT_LENGTH) return MOD_ERR_INVALID_LEN;
    if (fs     <= 0.0f)      return MOD_ERR_FS_ZERO;

    /* 0 初始化 */
    result->fc = 0.0f;  result->fm = 0.0f;  result->mf = 0.0f;
    result->df_hz = 0.0f;
    result->carrier_mag = 0.0f;
    result->sideband1_mag = 0.0f;
    result->sideband2_mag = 0.0f;
    result->fit_error = 0.0f;
    result->fm_auto = 0;
    result->status = MOD_OK;

    /* 1. 算 buffer 均值 (去 DC 用) */
    float32_t mean = 0.0f;
    for (uint32_t n = 0; n < length; n++) {
        mean += (float32_t)signal[n];
    }
    mean /= (float32_t)length;

    /* 2. 找 fc
     *    - fc_hint > 0: 用它
     *    - fc_hint == 0: 从 FFT_OutputBuf 自动找主峰 (Phase_Get_PeakFreq) */
    float32_t fc = fc_hint;
    if (fc <= 0.0f) {
        Phase_PeakInfo_t pk[1];
        uint16_t n = Phase_Get_PeakFreq(pk, 1);
        if (n == 0) {
            result->status = MOD_ERR_NO_PEAK_FOUND;
            return result->status;
        }
        fc = pk[0].freq_hz;
    }
    result->fc = fc;

    /* 3. 构造 fm 候选列表
     *    - fm_hint > 0: 1 个候选 = fm_hint
     *    - fm_hint == 0: 默认 1k/2k/3k/4k/5k 5 候选 */
    float32_t fm_list[FM_FM_CANDIDATE_NUM];
    uint32_t fm_n;
    if (fm_hint > 0.0f) {
        fm_list[0] = fm_hint;
        fm_n = 1;
    } else {
        for (uint32_t k = 0; k < FM_FM_CANDIDATE_NUM; k++) {
            fm_list[k] = (float32_t)(FM_FM_MIN_HZ + k * FM_FM_STEP_HZ);
        }
        fm_n = FM_FM_CANDIDATE_NUM;
        result->fm_auto = 1;
    }

    /* 4. 遍历 (fm, β) 找最小拟合误差 */
    uint32_t best_fm_idx = 0;
    uint32_t best_beta_idx = 0;
    float32_t best_err = 1e30f;
    float32_t best_a0 = 0.0f, best_a1 = 0.0f, best_a2 = 0.0f;

    for (uint32_t fk = 0; fk < fm_n; fk++) {
        float32_t fm_hz = fm_list[fk];
        if (fm_hz >= fs * 0.5f) continue;  /* 越界保护 */

        /* Goertzel 算 5 个频点的幅值: fc, fc±fm, fc±2fm */
        float32_t a0 = goertzel_magnitude(signal, length, mean, fc, fs);

        float32_t s1_lo = goertzel_magnitude(signal, length, mean, fc - fm_hz,     fs);
        float32_t s1_hi = goertzel_magnitude(signal, length, mean, fc + fm_hz,     fs);
        float32_t s2_lo = goertzel_magnitude(signal, length, mean, fc - 2.0f*fm_hz, fs);
        float32_t s2_hi = goertzel_magnitude(signal, length, mean, fc + 2.0f*fm_hz, fs);

        float32_t a1 = 0.5f * (s1_lo + s1_hi);
        float32_t a2 = 0.5f * (s2_lo + s2_hi);

        if (a0 < 1e-6f) continue;   /* 载波太小 (可能在 J_0 过零), 跳过 */

        /* 对 β 1.0~6.0 查表拟合 */
        for (uint32_t bi = 0; bi < FM_MF_NUM; bi++) {
            float32_t err = fit_bessel_error(a0, a1, a2, bi);
            if (err < best_err) {
                best_err = err;
                best_fm_idx = fk;
                best_beta_idx = bi;
                best_a0 = a0;
                best_a1 = a1;
                best_a2 = a2;
            }
        }
    }

    if (best_err >= 1e29f) {
        /* 全部候选都没匹配上 (可能信号太弱 / 载波不在 fc / β 超出 [1, 6]) */
        result->status = MOD_ERR_BESSEL_NOT_CONV;
        return result->status;
    }

    /* 5. 填结果 */
    result->fm            = fm_list[best_fm_idx];
    result->mf            = FM_MF_START + (float32_t)best_beta_idx * FM_MF_STEP;
    result->df_hz         = result->mf * result->fm;
    result->carrier_mag   = best_a0;
    result->sideband1_mag = best_a1;
    result->sideband2_mag = best_a2;
    result->fit_error     = best_err;

    /* 拟合误差 < 0.01 视为良好匹配, 0.01~0.1 一般, >0.1 警告 */
    if (best_err > 0.1f) {
        result->status = MOD_ERR_BESSEL_NOT_CONV;  /* 复用错误码表示拟合差 */
    }

    return MOD_OK;
}

/* ===========================================================================
 *  AM_Windowed_Ma  时域加窗后取包络算 ma
 * ===========================================================================
 */
Modulation_Status_t AM_Windowed_Ma(const uint16_t *signal,
                                   uint16_t length,
                                   float32_t fs,
                                   float32_t vref,
                                   Modulation_WinType_t win_type,
                                   Modulation_AM_Result_t *result)
{
    /* v1.0 时域法 — 用户要求改用 AM_Spectral_Ma (频谱法), 本函数不再使用
     * 函数体保留以备未来调试, 实际主循环不调用
     * ==================== (v1.1: 函数体禁用) ==================== */
    (void)signal; (void)length; (void)fs; (void)vref; (void)win_type;
    if (result) {
        result->ma = 0.0f;
        result->v_max = 0.0f;
        result->v_min = 0.0f;
        result->v_dc = 0.0f;
        result->win_used = win_type;
        result->status = MOD_ERR_CLIPPED;  /* 标记: 不用此函数, 请改用 AM_Spectral_Ma */
    }
    return MOD_ERR_CLIPPED;
#if 0  // 原实现 (保留以备将来切换)
    /* 参数检查 */
    if (signal == NULL || result == NULL) return MOD_ERR_NULL_PTR;

    if (length == 0) length = FFT_LENGTH;
    if (fs     == 0.0f) fs  = SAMPLING_RATE;
    if (vref   == 0.0f) vref = Reference_Voltage;

    if (length > FFT_LENGTH) return MOD_ERR_INVALID_LEN;
    if (fs     <= 0.0f)      return MOD_ERR_FS_ZERO;

    result->ma   = 0.0f;
    result->v_max = 0.0f;
    result->v_min = 0.0f;
    result->v_dc  = 0.0f;
    result->win_used = win_type;
    result->status = MOD_OK;

    /* 1. ADC → 浮点电压 (不减 DC!
     *    标准 AM 公式 ma = (Vmax - Vmin) / (Vmax + Vmin) 要求包络保留 DC,
     *    减 DC 会把包络谷底压到 0, ma 永远接近 1.
     *    这里直接把原始电压送下去, 包络里 DC = 载波平均幅值 = A, ma·A = 调制深度)
     */
    static float32_t x[FFT_LENGTH];   /* static 不占栈 */
    for (uint16_t i = 0; i < length; i++) {
        x[i] = (float32_t)signal[i] * (vref / ADC_MAX_12BIT);
    }

    /* 2. 加窗 (调用方指定类型) */
    static float32_t w[FFT_LENGTH];   /* static, 不重复算系数 */
    if (mod_generate_window(win_type, length, w) != MOD_OK) {
        result->status = MOD_ERR_INVALID_LEN;
        return result->status;
    }
    Window_Apply(x, w, length);

    /* 3. abs + IIR 低通 取包络
     *    IIR 输入是 |x[i]| (瞬时幅值), IIR 输出 = 包络 = A·(1 + ma·cos(2π·fm·t))
     *    包络最大值 = A·(1+ma), 最小值 = A·(1-ma)
     *    ma = (Vmax - Vmin) / (Vmax + Vmin)
     */
    static float32_t env[FFT_LENGTH];
    float32_t y = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        float32_t ai = fabsf(x[i]);
        y = (1.0f - ENV_IIR_ALPHA) * y + ENV_IIR_ALPHA * ai;
        env[i] = y;
    }

    /* 4. 找 Vmax / Vmin (避开首尾 10% 边缘) */
    uint16_t start = (uint16_t)(length * AM_EDGE_RATIO);
    uint16_t end   = length - start;
    if (end <= start + 2) {
        result->status = MOD_ERR_INVALID_LEN;
        return result->status;
    }
    float32_t vmax = env[start];
    float32_t vmin = env[start];
    for (uint16_t i = start; i < end; i++) {
        if (env[i] > vmax) vmax = env[i];
        if (env[i] < vmin) vmin = env[i];
    }

    result->v_max = vmax;
    result->v_min = vmin;
    result->v_dc  = 0.5f * (vmax + vmin);   /* 直流分量 = 平均 */

    /* 5. 算 ma = (Vmax - Vmin) / (Vmax + Vmin) */
    if ((vmax + vmin) < 1e-6f) {
        /* 几乎无信号 (噪声 / 直流) */
        result->status = MOD_ERR_MA_OUT_OF_RANGE;
        result->ma = 0.0f;
        return result->status;
    }

    result->ma = (vmax - vmin) / (vmax + vmin);

    /* ma > 1.5 视为异常 */
    if (result->ma > 1.5f) {
        result->status = MOD_ERR_MA_OUT_OF_RANGE;
    } else if (result->ma > 1.0f) {
        /* 过调: ma 在 (1, 1.5] 是合理的过调, 不算错误 */
        result->status = MOD_OK;
    }

    return MOD_OK;
#endif
}

/* ===========================================================================
 *  AM_Spectral_Ma  频谱法算 ma (抗削顶 / 抗时域失真, 推荐用于工程测量)
 * ===========================================================================
 *
 *  原理: 标准 AM 信号频谱:
 *      S(f) = (A/2)·δ(f-fc) + (A·ma/4)·δ(f-fc-fm) + (A·ma/4)·δ(f-fc+fm)
 *            \________载波________/  \___________±1 阶边带_____________/
 *
 *  所以 |±1 阶边带| / |载波| = ma / 2, 即
 *      ma = 2 · A1 / A0
 *
 *  此法只依赖 FFT 频谱的相对幅值, 不受 ADC 削顶 / 直流偏置 / 时域噪声影响,
 *  是工程上最稳的 ma 测量法 (示波器 + FFT 选件的 "AM 调制度" 测量就是这原理).
 *
 *  限制: 要求调制频率 fm ≥ 几倍 df (频谱分辨率), 否则 ±1 阶边带和载波粘一起.
 *        Fs=500kHz, N=1024 → df=488Hz → fm ≥ 2kHz 较稳.
 */
Modulation_Status_t AM_Spectral_Ma(const uint16_t *signal,
                                    uint16_t length,
                                    float32_t fs,
                                    float32_t fm,
                                    Modulation_AM_Spec_Result_t *result)
{
    (void)signal;     /* 不读时域, 只看 FFT_OutputBuf */
    (void)length;

    if (result == NULL) return MOD_ERR_NULL_PTR;
    if (fm <= 0.0f)     return MOD_ERR_FM_INVALID;

    if (fs == 0.0f) fs = SAMPLING_RATE;
    if (fm >= fs * 0.5f) return MOD_ERR_FM_INVALID;

    result->ma = 0.0f;
    result->fc = 0.0f;
    result->fm = fm;
    result->carrier_mag  = 0.0f;
    result->sideband_mag = 0.0f;
    result->status = MOD_OK;

    /* 1. 找载波 (主峰) */
    Phase_PeakInfo_t pk[1];
    uint16_t n = Phase_Get_PeakFreq(pk, 1);
    if (n == 0) {
        result->status = MOD_ERR_NO_PEAK_FOUND;
        return result->status;
    }
    result->fc          = pk[0].freq_hz;
    result->carrier_mag = pk[0].mag;
    uint16_t carrier_bin = pk[0].bin;

    /* 2. 算 ±1 阶边带应在的 bin */
    float32_t df = fs / (float32_t)FFT_LENGTH;
    float32_t fm_bins = fm / df;
    int sideband_offset = (int)(fm_bins + 0.5f);
    if (sideband_offset < 1) sideband_offset = 1;

    int16_t bin_lo = (int16_t)carrier_bin - sideband_offset;
    int16_t bin_hi = (int16_t)carrier_bin + sideband_offset;
    if (bin_lo < 1 || bin_hi >= (int16_t)(FFT_LENGTH / 2)) {
        result->status = MOD_ERR_NO_PEAK_FOUND;
        return result->status;
    }

    /* 3. 在 ±1 阶边带附近 ±2 bin 找局部最大 (抗频谱泄漏) */
    #define SPEC_SEARCH_HALF  2
    float32_t sb_lo = 0.0f;
    for (int k = -SPEC_SEARCH_HALF; k <= SPEC_SEARCH_HALF; k++) {
        float32_t v = FFT_OutputBuf[bin_lo + k];
        if (v > sb_lo) sb_lo = v;
    }
    float32_t sb_hi = 0.0f;
    for (int k = -SPEC_SEARCH_HALF; k <= SPEC_SEARCH_HALF; k++) {
        float32_t v = FFT_OutputBuf[bin_hi + k];
        if (v > sb_hi) sb_hi = v;
    }
    result->sideband_mag = 0.5f * (sb_lo + sb_hi);

    if (result->carrier_mag < 1e-6f) {
        result->status = MOD_ERR_NO_PEAK_FOUND;
        return result->status;
    }

    /* 4. ma = 2 · A1 / A0 */
    result->ma = 2.0f * result->sideband_mag / result->carrier_mag;

    /* 物理意义: ma 不可能 < 0, 不可能 > 1.5 (过调也会重建载波) */
    if (result->ma < 0.0f)   result->ma = 0.0f;
    if (result->ma > 1.5f)   result->status = MOD_ERR_MA_OUT_OF_RANGE;
    if (result->ma > 1.0f)   result->status = MOD_OK;   /* (1, 1.5] 算合理过调 */

    return MOD_OK;
}

/* ===========================================================================
 *  结果发送函数 (UART1 DMA, 格式与 MATLAB 脚本配套)
 * ===========================================================================
 */

void FM_Result_Send(const Modulation_FM_Result_t *result)
{
    if (result == NULL) return;

    char line[160];
    int len = snprintf(line, sizeof(line),
        "MOD_FM,mf=%.2f,df=%.1f,fc=%.1f,fm=%.1f,car=%.3f,side1=%.3f,side2=%.3f,fit=%.4f,auto=%u,status=%d\r\n",
        (double)result->mf,
        (double)result->df_hz,
        (double)result->fc,
        (double)result->fm,
        (double)result->carrier_mag,
        (double)result->sideband1_mag,
        (double)result->sideband2_mag,
        (double)result->fit_error,
        (unsigned)result->fm_auto,
        (int)result->status);

    if (len > 0 && len < (int)sizeof(line)) {
        UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len);
    }
}

void AM_Spec_Result_Send(const Modulation_AM_Spec_Result_t *result)
{
    if (result == NULL) return;

    char line[128];
    int len = snprintf(line, sizeof(line),
        "MOD_AM,ma=%.3f,fc=%.1f,fm=%.1f,car=%.3f,side=%.3f,status=%d\r\n",
        (double)result->ma,
        (double)result->fc,
        (double)result->fm,
        (double)result->carrier_mag,
        (double)result->sideband_mag,
        (int)result->status);

    if (len > 0 && len < (int)sizeof(line)) {
        UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len);
    }
}
