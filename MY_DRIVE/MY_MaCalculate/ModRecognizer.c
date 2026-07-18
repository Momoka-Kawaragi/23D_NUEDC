/**
 ******************************************************************************
 * @file    ModRecognizer.c
 * @brief   调制识别 — 仿 23Df4070010 工程: CMSIS-DSP FFT + 6 类型识别
 * @note    载波 50kHz, Fs≈206kHz(TIM2), N=1024 → bin≈201Hz → 载波bin=249
 ******************************************************************************
 */

#include "ModRecognizer.h"
#include "Phase.h"
#include "ZPN_Uart.h"
#include "ZPN_Hmi.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ==================== 常量 ==================== */
#define MR_N           1024u
#define MR_FS          205882.0f                    /* TIM2: 84MHz/408 */
#define MR_BIN_W       (MR_FS / (float)MR_N)       /* ≈201 Hz */
#define MR_CARRIER_IDX 249u                        /* 50kHz / 201Hz */
#define MR_HALF        (MR_N / 2u)
#define MR_MAX_PEAKS   100u

/* ==================== 贝塞尔表 (mf=1.0~6.0, 步进 0.1, 共 51 项) ==================== */
#define MF_NUM   51u
#define MF_START 1.0f
#define MF_STEP  0.1f

static const float BJ0[MF_NUM] = {
    0.765198f,0.719622f,0.671133f,0.620086f,0.566855f,0.511828f,0.455402f,0.397985f,0.339986f,0.281819f,
    0.223891f,0.166607f,0.110362f,0.055540f,0.002508f,-0.048384f,-0.096805f,-0.142449f,-0.185036f,-0.224312f,
   -0.260052f,-0.292064f,-0.320188f,-0.344296f,-0.364296f,-0.380128f,-0.391769f,-0.399230f,-0.402556f,-0.401826f,
   -0.397150f,-0.388670f,-0.376557f,-0.361011f,-0.342257f,-0.320543f,-0.296138f,-0.269331f,-0.240425f,-0.209738f,
   -0.177597f,-0.144335f,-0.110290f,-0.075803f,-0.041210f,-0.006844f,0.026971f,0.059920f,0.091703f,0.122033f,
    0.150645f
};
static const float BJ1[MF_NUM] = {
    0.440051f,0.470902f,0.498289f,0.522023f,0.541948f,0.557937f,0.569896f,0.577765f,0.581517f,0.581157f,
    0.576725f,0.568292f,0.555963f,0.539873f,0.520185f,0.497094f,0.470818f,0.441601f,0.409709f,0.375427f,
    0.339059f,0.300921f,0.261343f,0.220663f,0.179226f,0.137378f,0.095466f,0.053834f,0.012821f,-0.027244f,
   -0.066043f,-0.103273f,-0.138647f,-0.171897f,-0.202776f,-0.231060f,-0.256553f,-0.279081f,-0.298500f,-0.314695f,
   -0.327579f,-0.337097f,-0.343223f,-0.345961f,-0.345345f,-0.341438f,-0.334333f,-0.324148f,-0.311028f,-0.295142f,
   -0.276684f
};
static const float BJ2[MF_NUM] = {
    0.114903f,0.136564f,0.159349f,0.183027f,0.207356f,0.232088f,0.256968f,0.281739f,0.306144f,0.329926f,
    0.352834f,0.374624f,0.395059f,0.413915f,0.430980f,0.446059f,0.458973f,0.469562f,0.477685f,0.483227f,
    0.486091f,0.486207f,0.483528f,0.478032f,0.469723f,0.458629f,0.444805f,0.428330f,0.409304f,0.387855f,
    0.364128f,0.338292f,0.310535f,0.281059f,0.250086f,0.217849f,0.184593f,0.150573f,0.116050f,0.081292f,
    0.046565f,0.012140f,-0.021718f,-0.054748f,-0.086695f,-0.117315f,-0.146375f,-0.173656f,-0.198954f,-0.222082f,
   -0.242873f
};

/* ==================== 静态变量 ==================== */
static arm_rfft_fast_instance_f32 mr_rfft;
static uint8_t  mr_init_ok = 0;
static uint32_t mr_last_cb = 0;
static float    mr_mag[MR_HALF];       /* 幅值谱 (bin 0 ~ MR_HALF-1) */
static float    mr_fft_out[MR_N];      /* arm_rfft 复数输出缓冲 */
static uint16_t mr_adc_tmp[MR_N];      /* ADC 快照 */
static uint32_t mr_peaks[MR_MAX_PEAKS];

/* ==================== 内部工具 ==================== */

static void mag_normalize(float *mag, uint16_t n)
{
    float mx = mag[10];
    for (uint16_t i = 11; i < n; i++)
        if (mag[i] > mx) mx = mag[i];
    if (mx < 1e-9f) return;
    float inv = 1.0f / mx;
    for (uint16_t i = 0; i < n; i++) mag[i] *= inv;
}

/* ==================== 峰值检测 (仿 23Df4070010) ==================== */

static uint16_t find_peak(const float *mag, uint16_t start, uint16_t end)
{
    float    mx = 0.0f;
    uint16_t mi = start;
    for (uint16_t i = start; i < end; i++) {
        if (mag[i] > mx) { mx = mag[i]; mi = i; }
    }
    return mi;
}

static int peak_count(const float *mag)
{
    float max_val = 0.0f;
    for (uint16_t i = 10; i < MR_HALF; i++)
        if (mag[i] > max_val) max_val = mag[i];

    float thresh = max_val * 0.08f;
    if (thresh < 0.008f) thresh = 0.008f;

    int cnt = 0;
    for (uint16_t i = 10; i < MR_HALF - 1; i++)
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > thresh)
            cnt++;
    return cnt;
}

/* 低阈值全峰检测 — 专用于 2PSK/FM/FSK 细判
 *   对应 23Df4070010 的 peak_detect(threshold=70), 归一化后 ≈ 0.002
 *   必须用固定低阈值而非相对值, 否则 2PSK 的细节旁瓣会被漏掉 */
static int peak_detect(const float *mag, uint32_t *peaks)
{
    int cnt = 0;
    for (uint16_t i = 5; i < MR_HALF - 1; i++) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > 0.0015f) {
            if (cnt < (int)MR_MAX_PEAKS) { peaks[cnt] = i; cnt++; }
        }
    }
    return cnt;
}

/* 时域平顶检测 (ASK 特征: 连续 ADC 值相近 → 0 码时段) */
static int find_000(const uint16_t *arr, uint16_t len)
{
    int cnt = 0, cnt_max = 0;
    for (uint16_t i = 1; i < len; i++) {
        if (abs((int)arr[i] - (int)arr[i-1]) < 20) {
            cnt++;
            if (cnt > cnt_max) cnt_max = cnt;
        } else {
            cnt = 0;
        }
    }
    return cnt_max;
}

/* ==================== 参数计算 ==================== */

static float calc_am_ma(const float *mag, uint16_t c_idx, float *out_fm)
{
    float c_mag = mag[c_idx];
    if (out_fm) *out_fm = 0.0f;
    if (c_mag < 1e-6f) return 0.0f;

    /* 左右各找最强边带峰 (find_peak, 非第一个局部峰) */
    int lo_l = (int)c_idx - 60; if (lo_l < 10) lo_l = 10;
    int hi_r = (int)c_idx + 60; if (hi_r >= (int)MR_HALF - 1) hi_r = (int)MR_HALF - 2;

    /* 左边带: 在 [lo_l, c_idx-4] 找最大峰值 */
    float  l_mag = 0.0f; int l_bin = 0;
    for (int i = lo_l; i <= (int)c_idx - 4; i++) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > l_mag)
            { l_mag = mag[i]; l_bin = i; }
    }
    /* 右边带: 在 [c_idx+4, hi_r] 找最大峰值 */
    float  r_mag = 0.0f; int r_bin = 0;
    for (int i = (int)c_idx + 4; i <= hi_r; i++) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > r_mag)
            { r_mag = mag[i]; r_bin = i; }
    }

    float thresh = c_mag * 0.04f;  /* 边带至少为载波 4% */
    if (l_mag < thresh || r_mag < thresh) return 0.0f;

    if (out_fm) {
        int dist = ((int)c_idx - l_bin) + (r_bin - (int)c_idx);
        *out_fm = (float)(dist / 2) * MR_BIN_W;
    }

    float side_avg = (l_mag + r_mag) * 0.5f;
    return 2.0f * side_avg / c_mag * 100.0f;
}

static float calc_2ask_baud(const float *mag, uint16_t c_idx)
{
    float c_mag = mag[c_idx];
    if (c_mag < 1e-6f) return 0.0f;

    float thresh = c_mag * 0.15f;
    if (thresh < 0.005f) thresh = 0.005f;

    int l_peak = 0, r_peak = 0;
    int lo = (c_idx > 70) ? (int)c_idx - 60 : 11;
    int hi = (c_idx < MR_HALF - 60) ? (int)c_idx + 60 : (int)MR_HALF - 2;

    for (int i = (int)c_idx - 4; i >= lo; i--) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > thresh) { l_peak = i; break; }
    }
    for (int i = (int)c_idx + 4; i <= hi; i++) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > thresh) { r_peak = i; break; }
    }

    if (l_peak && r_peak) {
        int dl = (int)c_idx - l_peak, dr = r_peak - (int)c_idx;
        if (abs(dl - dr) <= 3) return ((float)(dl + dr) * 0.5f) * MR_BIN_W;
        return (mag[l_peak] > mag[r_peak]) ? (float)dl * MR_BIN_W : (float)dr * MR_BIN_W;
    }
    if (l_peak) return (float)((int)c_idx - l_peak) * MR_BIN_W;
    if (r_peak) return (float)(r_peak - (int)c_idx) * MR_BIN_W;
    return 0.0f;
}

static float calc_2psk_baud(const float *mag, uint16_t c_idx)
{
    int lo = ((int)c_idx > 40) ? (int)c_idx - 40 : 11;
    int hi = ((int)c_idx < MR_HALF - 40) ? (int)c_idx + 40 : (int)MR_HALF - 2;

    float  l_max = 0.0f, r_max = 0.0f;
    uint16_t l_peak = 0, r_peak = 0;

    for (int i = lo; i < (int)c_idx; i++) {
        if (mag[i] > l_max) { l_max = mag[i]; l_peak = (uint16_t)i; }
    }
    for (int i = (int)c_idx + 1; i <= hi; i++) {
        if (mag[i] > r_max) { r_max = mag[i]; r_peak = (uint16_t)i; }
    }

    int dl = (int)c_idx - (int)l_peak, dr = (int)r_peak - (int)c_idx;
    float thresh = 0.01f;

    if (l_max > thresh && r_max > thresh && abs(dl - dr) <= 4)
        return ((float)(dl + dr) * 0.5f) * MR_BIN_W;
    if (l_max > thresh && l_max > r_max * 2.0f && dl >= 8 && dl <= 35)
        return (float)dl * MR_BIN_W;
    if (r_max > thresh && r_max > l_max * 2.0f && dr >= 8 && dr <= 35)
        return (float)dr * MR_BIN_W;
    return 0.0f;
}

/* FSK 边带检测 */
static int fsk_find_sideband(const float *mag, uint16_t center)
{
    float c_mag = mag[center];
    float thresh = c_mag * 0.10f;
    if (thresh < 0.004f) thresh = 0.004f;

    int l_dist = 0, r_dist = 0;
    float l_mag_v = 0.0f, r_mag_v = 0.0f;

    for (int i = (int)center - 4; i >= (int)center - 25 && i > 10; i--) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > thresh) {
            l_dist = (int)center - i; l_mag_v = mag[i]; break;
        }
    }
    for (int i = (int)center + 4; i <= (int)center + 25 && i < (int)MR_HALF - 1; i++) {
        if (mag[i] > mag[i-1] && mag[i] > mag[i+1] && mag[i] > thresh) {
            r_dist = i - (int)center; r_mag_v = mag[i]; break;
        }
    }

    if (l_dist > 0 && r_dist > 0) {
        if (abs(l_dist - r_dist) <= 3) return (l_dist + r_dist) / 2;
        return (l_mag_v > r_mag_v) ? l_dist : r_dist;
    }
    if (l_dist > 0) return l_dist;
    if (r_dist > 0) return r_dist;
    return 0;
}

static float calc_2fsk_params(const float *mag, float *out_h,
                              uint16_t *out_f1, uint16_t *out_f2)
{
    uint16_t p1 = find_peak(mag, 10, MR_HALF);

    uint16_t p2 = 0;
    float    p2_mag = 0.0f;
    for (uint16_t i = 10; i < MR_HALF; i++) {
        if (i == p1) continue;
        if (abs((int)i - (int)p1) <= 30) continue;
        if (mag[i] > p2_mag) { p2_mag = mag[i]; p2 = i; }
    }

    if (p2 == 0 || p2_mag < mag[p1] * 0.15f) {
        *out_h = 0.0f; *out_f1 = 0; *out_f2 = 0; return 0.0f;
    }

    if (p1 > p2) { uint16_t t = p1; p1 = p2; p2 = t; }
    *out_f1 = p1; *out_f2 = p2;

    float delta_f = (float)(p2 - p1) * MR_BIN_W;

    int sb1 = fsk_find_sideband(mag, p1);
    int sb2 = fsk_find_sideband(mag, p2);

    int valid = 0, sum_sb = 0;
    if (sb1 > 0) { sum_sb += sb1; valid++; }
    if (sb2 > 0) { sum_sb += sb2; valid++; }

    float baud = (valid > 0) ? (float)sum_sb / (float)valid * MR_BIN_W
                             : delta_f / 8.0f;
    if (baud > 0.0f) *out_h = delta_f / baud;
    else             *out_h = 0.0f;

    return baud;
}

/* ==================== Goertzel + J0/J1/J2 三阶拟合 (替代比值查表) ==================== */

static float mr_hann[MR_N];
static uint8_t mr_hann_ok = 0;

static float goertzel_mag(const uint16_t *samples, float mean, float target_hz)
{
    float omega = 2.0f * 3.14159265f * target_hz / MR_FS;
    float coeff = 2.0f * cosf(omega);
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

    for (uint32_t n = 0; n < MR_N; n++) {
        float x = ((float)samples[n] - mean) * mr_hann[n];
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    if (power <= 0.0f) return 0.0f;
    return sqrtf(power);
}

static float fit_bessel_err(float a0, float a1, float a2, uint32_t idx)
{
    float j0 = fabsf(BJ0[idx]);
    float j1 = fabsf(BJ1[idx]);
    float j2 = fabsf(BJ2[idx]);

    float dot_ot  = a0 * j0 + a1 * j1 + a2 * j2;
    float dot_tt  = j0 * j0 + j1 * j1 + j2 * j2;
    if (dot_tt < 1e-12f) return 1e30f;

    float scale = dot_ot / dot_tt;

    float e0 = a0 - scale * j0;
    float e1 = a1 - scale * j1;
    float e2 = a2 - scale * j2;

    float obs = a0 * a0 + a1 * a1 + a2 * a2;
    if (obs < 1e-12f) return 1e30f;

    return (e0 * e0 + e1 * e1 + e2 * e2) / obs;
}

static void calc_fm_goertzel(const uint16_t *adc, float *out_mf, float *out_fm, float *out_df)
{
    if (!mr_hann_ok) {
        for (uint32_t i = 0; i < MR_N; i++)
            mr_hann[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * (float)i / (float)(MR_N - 1)));
        mr_hann_ok = 1;
    }

    float mean = 0.0f;
    for (uint32_t i = 0; i < MR_N; i++) mean += (float)adc[i];
    mean /= (float)MR_N;

    float fc = (float)MR_CARRIER_IDX * MR_BIN_W;  /* 载波 Hz */

    uint32_t best_fm_hz = 1000;
    uint32_t best_beta_i = 0;
    float    best_err    = 1e30f;

    float a0 = goertzel_mag(adc, mean, fc);

    for (uint32_t fk = 0; fk < 5; fk++) {
        float fm_hz = (float)(1000 + fk * 1000);  /* 1k,2k,3k,4k,5k */
        if (fm_hz >= MR_FS * 0.5f) continue;

        float s1_lo = goertzel_mag(adc, mean, fc - fm_hz);
        float s1_hi = goertzel_mag(adc, mean, fc + fm_hz);
        float s2_lo = goertzel_mag(adc, mean, fc - 2.0f * fm_hz);
        float s2_hi = goertzel_mag(adc, mean, fc + 2.0f * fm_hz);

        float a1 = 0.5f * (s1_lo + s1_hi);
        float a2 = 0.5f * (s2_lo + s2_hi);

        for (uint32_t bi = 0; bi < MF_NUM; bi++) {
            float err = fit_bessel_err(a0, a1, a2, bi);
            if (err < best_err) {
                best_err    = err;
                best_fm_hz  = (uint32_t)fm_hz;
                best_beta_i = bi;
            }
        }
    }

    *out_mf = MF_START + MF_STEP * (float)best_beta_i;
    *out_fm = (float)best_fm_hz;
    *out_df = *out_mf * *out_fm;
}

/* ==================== 调制识别 (仿 23Df4070010 recognize_once) ==================== */

static MR_ModType_t recognize_once(const float *mag, const uint16_t *adc, uint16_t adc_len)
{
    int     cnt = peak_count(mag);
    int     f_max_idx = (int)find_peak(mag, 10, MR_HALF);
    float   max_val   = mag[f_max_idx];


    /* ======== 第1层: CW (仅1峰) ======== */
    if (cnt == 1) return MR_TYPE_CW;

    /* ======== 第2层: AM (恰好3峰) ======== */
    if (cnt == 3) return MR_TYPE_AM;

    /* ======== 第3层: 2FSK 预判 (双峰间距>12, 仅2个高峰, 各有边带) ======== */
    {
        int p1 = f_max_idx;
        int p2 = 0; float p2_mag = 0.0f;
        for (int i = 10; i < (int)MR_HALF; i++) {
            if (abs(i - p1) <= 12) continue;
            if (mag[i] > p2_mag) { p2_mag = mag[i]; p2 = i; }
        }
        if (p2 > 0 && p2_mag > mag[p1] * 0.15f && abs(p1 - p2) > 12) {
            /* 统计同高度峰数: >2 → FM (Bessel多边带), =2 → 2FSK */
            int high_peaks = 0;
            for (int i = 10; i < (int)MR_HALF; i++)
                if (mag[i] > mag[p1] * 0.5f) high_peaks++;
            if (high_peaks > 2) goto not_2fsk;

            /* 大间距直判 (>50bin≈10kHz, 边带可能超出搜索窗) */
            if (abs(p1 - p2) > 50)
                return MR_TYPE_2FSK;

            /* 确认: 两边各有自己的边带 */
            int sb1 = fsk_find_sideband(mag, (uint16_t)p1);
            int sb2 = fsk_find_sideband(mag, (uint16_t)p2);
            if (sb1 > 0 || sb2 > 0)
                return MR_TYPE_2FSK;
        }
    }
    not_2fsk:

    /* ======== 第4层: 2PSK (对称双峰 + 中心凹陷 + 无二阶边带) ======== */
    {
        int lo = (int)MR_CARRIER_IDX - 40; if (lo < 10) lo = 10;
        int hi = (int)MR_CARRIER_IDX + 40; if (hi >= (int)MR_HALF - 1) hi = (int)MR_HALF - 2;
        int l_pk = (int)find_peak(mag, (uint16_t)lo, MR_CARRIER_IDX - 5U);
        int r_pk = (int)find_peak(mag, MR_CARRIER_IDX + 5U, (uint16_t)hi);
        int dl = (int)MR_CARRIER_IDX - l_pk;
        int dr = r_pk - (int)MR_CARRIER_IDX;
        float c_v = mag[MR_CARRIER_IDX];

        if (mag[l_pk] > 0.05f && mag[r_pk] > 0.05f
            && abs(dl - dr) <= 5 && dl >= 5 && dr >= 5
            && c_v < mag[l_pk] * 0.2f && c_v < mag[r_pk] * 0.2f) {
            /* 排除 FM (J0过零): FM有高阶边带, 2PSK没有 */
            int l2 = (int)MR_CARRIER_IDX - 2 * dl;
            int r2 = (int)MR_CARRIER_IDX + 2 * dr;
            if (l2 > 10 && r2 < (int)MR_HALF - 1) {
                float pk2_l = mag[l2];
                float pk2_r = mag[r2];
                if (pk2_l > mag[l2-1] && pk2_l > mag[l2+1]) pk2_l = pk2_l; else pk2_l = 0;
                if (pk2_r > mag[r2-1] && pk2_r > mag[r2+1]) pk2_r = pk2_r; else pk2_r = 0;
                if (pk2_l > 0.03f || pk2_r > 0.03f) {
                    /* 存在二阶边带 → FM (Bessel分布), 非2PSK */
                    goto not_2psk;
                }
            }
            return MR_TYPE_2PSK;
        }
        not_2psk:;
    }

    /* ======== 第5层: 2ASK (时域平顶) ======== */
    if (find_000(adc, adc_len) > 15) return MR_TYPE_2ASK;

    /* ======== 第6层: 2PSK (载波深度抑制 + 不规则边带) ======== */
    if (mag[MR_CARRIER_IDX] < max_val * 0.10f
        && mag[MR_CARRIER_IDX] < 0.015f
        && abs(f_max_idx - (int)MR_CARRIER_IDX) < 30)
    {
        int pn = peak_detect(mag, mr_peaks);

        /* 对称双峰直判: DSB-SC 类信号 (载波两侧各一个强峰) */
        if (pn >= 2) {
            int sym_pairs = 0;
            for (int i = 0; i < pn; i++) {
                if (mr_peaks[i] == MR_CARRIER_IDX) continue;
                int mirror = 2 * (int)MR_CARRIER_IDX - (int)mr_peaks[i];
                if (mirror >= 5 && mirror < (int)MR_HALF - 5) {
                    for (int j = 0; j < pn; j++) {
                        if (abs((int)mr_peaks[j] - mirror) <= 3) {
                            if (mag[mr_peaks[i]] > 0.05f && mag[mr_peaks[j]] > 0.05f)
                                sym_pairs++;
                            break;
                        }
                    }
                }
            }
            /* 至少一对强对称峰 → 2PSK */
            if (sym_pairs >= 1) return MR_TYPE_2PSK;
        }

        /* 多峰不规则 → 2PSK (sinc²旁瓣); 规则 → FM */
        int min_iv = (int)MR_HALF, max_iv = 0;
        for (int i = 1; i < pn; i++) {
            int iv = (int)mr_peaks[i] - (int)mr_peaks[i-1];
            if (iv < min_iv) min_iv = iv;
            if (iv > max_iv) max_iv = iv;
        }
        if (pn > 5 && (max_iv - min_iv) > 5) return MR_TYPE_2PSK;
        return MR_TYPE_FM;
    }

    /* ======== 第7层: 2FSK vs FM (对称性分析) ======== */
    {
        int pn = peak_detect(mag, mr_peaks);
        int sym_cnt = 0, check_cnt = 0;

        /* 对称性: 每个峰在 CARRIER_IDX 镜像位置是否有对应峰 */
        for (int i = 0; i < pn; i++) {
            if (mr_peaks[i] == MR_CARRIER_IDX) continue;
            int mirror = 2 * (int)MR_CARRIER_IDX - (int)mr_peaks[i];
            if (mirror >= 5 && mirror < (int)MR_HALF - 5) {
                check_cnt++;
                for (int j = 0; j < pn; j++) {
                    if (abs((int)mr_peaks[j] - mirror) <= 3) {
                        float r = mag[mr_peaks[i]] / mag[mr_peaks[j]];
                        if (r > 0.4f && r < 2.5f) sym_cnt++;
                        break;
                    }
                }
            }
        }

        /* 两个主峰间距及中间能量 */
        uint32_t peak1 = (uint32_t)f_max_idx;
        uint32_t peak2 = 0;
        float    peak2_mag = 0.0f;
        for (int i = 0; i < pn; i++) {
            if (mr_peaks[i] != peak1 && mag[mr_peaks[i]] > peak2_mag) {
                peak2_mag = mag[mr_peaks[i]];
                peak2 = mr_peaks[i];
            }
        }

        int   gap = 0;
        float mid_avg = 0.0f;
        if (peak2 > 0) {
            gap = abs((int)peak1 - (int)peak2);
            uint32_t left  = (peak1 < peak2) ? peak1 : peak2;
            uint32_t right = (peak1 < peak2) ? peak2 : peak1;
            float sum = 0.0f; int n_mid = 0;
            for (uint32_t k = left + 3; k + 3 < right && k < MR_HALF; k++) {
                sum += mag[k]; n_mid++;
            }
            if (n_mid > 0) mid_avg = sum / (float)n_mid;
        }

        float sym_ratio = (check_cnt > 0) ? (float)sym_cnt / (float)check_cnt : 0.0f;

        int is_fm = (sym_ratio > 0.55f)
                 || (pn >= 5 && sym_ratio > 0.40f && mag[MR_CARRIER_IDX] > 0.03f);
        int is_2fsk = (sym_ratio < 0.45f && gap > 12)
                   || (gap > 35 && mid_avg < mag[peak1] * 0.12f && sym_ratio < 0.35f);

        /* FM 优先: 同时满足时判 FM */
        if (is_2fsk && !is_fm) return MR_TYPE_2FSK;
        return MR_TYPE_FM;
    }
}

/* ==================== 对外 API ==================== */

void MR_Init(void)
{
    arm_rfft_fast_init_f32(&mr_rfft, MR_N);
    mr_last_cb = 0;
    mr_init_ok = 1;
}

MR_Result_t MR_Process(void)
{
    MR_Result_t r;
    memset(&r, 0, sizeof(r));
    r.type = MR_TYPE_NONE;

    if (!mr_init_ok) return r;
    if (g_cb_count == mr_last_cb) return r;   /* 无新数据 */
    mr_last_cb = g_cb_count;

    /* ---- 1. 拷贝 ADC 数据 ---- */
    __disable_irq();
    memcpy(mr_adc_tmp, ADC_Buffer, MR_N * sizeof(uint16_t));
    __enable_irq();

    /* ---- 2. 预处理: 去 DC + Hann 窗 + FFT ---- */
    {
        static float fft_in[MR_N];
        static float hann_win[MR_N];
        static uint8_t win_ok = 0;

        if (!win_ok) {
            for (uint16_t i = 0; i < MR_N; i++)
                hann_win[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * (float)i / (float)(MR_N - 1)));
            win_ok = 1;
        }

        float mean = 0.0f;
        for (uint16_t i = 0; i < MR_N; i++) mean += (float)mr_adc_tmp[i];
        mean /= (float)MR_N;

        for (uint16_t i = 0; i < MR_N; i++) {
            float x = ((float)mr_adc_tmp[i] - mean) * (1.0f / 2048.0f);  /* → [-1, 1] */
            fft_in[i] = x * hann_win[i];
        }

        arm_rfft_fast_f32(&mr_rfft, fft_in, mr_fft_out, 0);
    }

    /* ---- 3. 计算幅值谱 (packed → mag[0..511], 2/N 还原) ---- */
    mr_mag[0] = fabsf(mr_fft_out[0]) * 2.0f / (float)MR_N;
    for (uint16_t i = 1; i < MR_HALF; i++) {
        float re = mr_fft_out[2 * i];
        float im = mr_fft_out[2 * i + 1];
        mr_mag[i] = sqrtf(re * re + im * im) * 2.0f / (float)MR_N;
    }

    /* ---- 4. 归一化 (便于阈值比较) ---- */
    mag_normalize(mr_mag, MR_HALF);

    /* ---- 5. 调制识别 ---- */
    r.type = recognize_once(mr_mag, mr_adc_tmp, MR_N);

    /* ---- 6. 参数计算 ---- */
    /* FM/2PSK 载波可能被抑制, 用固定 CARRIER_IDX; 其余用 find_peak */
    if (r.type == MR_TYPE_FM || r.type == MR_TYPE_2PSK) {
        r.carrier_bin  = MR_CARRIER_IDX;
    } else {
        r.carrier_bin  = find_peak(mr_mag, (uint16_t)(MR_CARRIER_IDX - 30),
                                              (uint16_t)(MR_CARRIER_IDX + 30));
    }
    r.carrier_mag  = mr_mag[r.carrier_bin];
    r.carrier_freq = (float)r.carrier_bin * MR_BIN_W;
    r.peak_count   = (uint8_t)peak_count(mr_mag);

    switch (r.type) {
    case MR_TYPE_AM:
        r.ma = calc_am_ma(mr_mag, r.carrier_bin, &r.fm);
        break;
    case MR_TYPE_2ASK:
        r.baud = calc_2ask_baud(mr_mag, r.carrier_bin);
        r.ma   = calc_am_ma(mr_mag, r.carrier_bin, &r.fm);
        break;
    case MR_TYPE_2PSK:
        r.baud = calc_2psk_baud(mr_mag, r.carrier_bin);
        break;
    case MR_TYPE_FM: {
        calc_fm_goertzel(mr_adc_tmp, &r.mf, &r.fm, &r.df);

        static float mf_hist[7] = {0};
        static uint8_t mf_hist_idx = 0, mf_hist_fill = 0;

        mf_hist[mf_hist_idx] = r.mf;
        mf_hist_idx = (mf_hist_idx + 1) % 7;
        if (mf_hist_fill < 7) mf_hist_fill++;

        /* 取中值: 复制+排序 */
        float sorted[7];
        for (uint8_t i = 0; i < mf_hist_fill; i++) sorted[i] = mf_hist[i];
        for (uint8_t i = 0; i < mf_hist_fill - 1; i++)
            for (uint8_t j = i + 1; j < mf_hist_fill; j++)
                if (sorted[i] > sorted[j]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }

        r.mf = sorted[mf_hist_fill / 2];
        r.df = r.mf * r.fm;
        break;
    }
    case MR_TYPE_2FSK:
        r.baud = calc_2fsk_params(mr_mag, &r.h, &r.f1, &r.f2);
        break;
    default:
        break;
    }

    return r;
}

/* ==================== UART1 结果发送 ==================== */

static const char *mr_type_names[] = {
    "CW", "AM", "2ASK", "2PSK", "FM", "2FSK"
};

/* 浮点转 int+1位小数 (nano-specs 不支持 %f, 手动转换) */
#define FMT1(f)  (int)(f), (int)(((f) - (int)(f)) * 10.0f + 0.5f)
#define FMT2(f)  (int)(f), (int)(((f) - (int)(f)) * 100.0f + 0.5f)
#define FMT0(f)  (int)((f) + 0.5f)

void MR_SendResult(const MR_Result_t *r)
{
    if (r == NULL || r->type > MR_TYPE_2FSK) return;

    char line[128];
    int len = 0;

    switch (r->type) {
    case MR_TYPE_CW:
        len = snprintf(line, sizeof(line),
            "MR:%s,fc=%d\r\n",
            mr_type_names[r->type], FMT0(r->carrier_freq));
        break;
    case MR_TYPE_AM:
        len = snprintf(line, sizeof(line),
            "MR:%s,ma=%d.%d,fm=%dk\r\n",
            mr_type_names[r->type], FMT1(r->ma),
            (int)(r->fm / 1000.0f + 0.5f));
        break;
    case MR_TYPE_2ASK:
        len = snprintf(line, sizeof(line),
            "MR:%s,baud=%d,fm=%dk,ma=%d.%d\r\n",
            mr_type_names[r->type], FMT0(r->baud),
            (int)(r->fm / 1000.0f + 0.5f), FMT1(r->ma));
        break;
    case MR_TYPE_2PSK:
        len = snprintf(line, sizeof(line),
            "MR:%s,baud=%dk\r\n",
            mr_type_names[r->type],
            (int)(r->baud / 1000.0f + 0.5f));
        break;
    case MR_TYPE_FM:
        len = snprintf(line, sizeof(line),
            "MR:%s,mf=%d,fm=%dk,df=%dk\r\n",
            mr_type_names[r->type],
            (int)(r->mf * 10.0f + 0.5f),
            (int)(r->fm / 1000.0f + 0.5f),
            (int)(r->df / 1000.0f + 0.5f));
        break;
    case MR_TYPE_2FSK: {
        float fsk_df = (float)(r->f2 - r->f1) * MR_BIN_W * 0.5f;
        len = snprintf(line, sizeof(line),
            "MR:%s,baud=%dk,h=%d.%02d,df=%dk\r\n",
            mr_type_names[r->type],
            (int)(r->baud / 1000.0f + 0.5f), FMT2(r->h * 10.0f),
            (int)(fsk_df / 1000.0f + 0.5f));
        HMI_SetValue("x3", (int)(r->h * 1000.0f + 0.5f));
        break;
    }
    default:
        return;
    }

    if (len > 0 && len < (int)sizeof(line))
        UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len);
}
