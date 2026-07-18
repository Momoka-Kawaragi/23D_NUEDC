#include "DFT.h"
#include "Phase.h"
#include "ZPN_Uart.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define DFT_FS_HZ      512000.0f /* 采样率(Hz), 与PHASE_TARGET_FS一致 */
#define THD_MAX_HARM   10U
#define THD_WIN_FRAMES 10U  /* 最大值窗口帧数 (THD≥10%时使用) */
#define THD_WIN_LO     10U  /* 窗口帧数 (THD<10%时取均值) */
#define THD_NOISE_MV   5.0f  /* 噪声门限(mV), 低于此值的谐波视为噪声置零 */

////////////////////////AM/ASK//////////////////////////////
#define AM_WIN_FRAMES 10U     /* AM ma滤波窗口帧数 (取最大) */
#define AM_MIN_FRAMES 1U     /* ma窗口后取最小的帧数 (去尖峰) */
////////////////////////FM//////////////////////////////////
#define FM_WIN_FRAMES 6U     /* FM 频偏滤波窗口帧数 (取最大) */
#define FM_MIN_FRAMES 6U     /* FM 频偏窗口后取最小的帧数 (去尖峰) */
#define FM_THRESHOLD  0.005f /* FM判别谱线幅值阈值 (V) */
#define FM_MIN_LINES  5U     /* FM判别最少谱线数 */
#define TYPE_HOLD_FRAMES 3U  /* TYPE切换需持续帧数 */
#define TYPE_THD_THRESH  0.03f /* THD阈值: <此值→SINE, ≥此值→SQUARE */

static volatile uint32_t g_last_cb = 0;
extern volatile uint32_t g_cb_count;
static float32_t g_f0     = 0.0f;
static float32_t g_thd    = 0.0f;

static float32_t g_harms[THD_MAX_HARM + 1];
static volatile uint8_t g_ready = 0;
static volatile uint8_t g_flush_filter = 0;

void DFT_Process(void)
{
    if (g_cb_count == g_last_cb) return;
    g_last_cb = g_cb_count;

    static uint16_t snap[FFT_LENGTH];
    __disable_irq();
    memcpy(snap, ADC_Buffer, sizeof(snap));
    __enable_irq();

    float32_t ph, mag;
    Phase_FFT_OneChannel(snap, &ph, &mag, &g_f0);
    (void)ph; (void)mag;
    if (g_f0 < 1000.0f) g_f0 = 1000.0f;  /* 低于1kHz钳位 */

    /* F0突变>20% → 清滤波状态 */
    {
        static float32_t last_f0 = 0.0f;
        if (last_f0 > 0.0f) {
            float32_t ratio = (g_f0 > last_f0) ? g_f0 / last_f0 : last_f0 / g_f0;
            if (ratio > 1.2f) g_flush_filter = 1;
        }
        last_f0 = g_f0;
    }

    /* ── 单频DFT精算谐波 (三角递推, 一次遍历 H1~H10) ── */
    {
        static float32_t win[FFT_LENGTH];
        static uint8_t win_ok = 0;
        if (!win_ok) {
            for (uint32_t i = 0; i < FFT_LENGTH; i++)
                win[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * (float32_t)i / (float32_t)(FFT_LENGTH - 1)));
            win_ok = 1;
        }

        uint64_t sum = 0;
        for (uint32_t i = 0; i < FFT_LENGTH; i++) sum += snap[i];
        float32_t mean   = (float32_t)sum / (float32_t)FFT_LENGTH;
        float32_t inv_fs = 1.0f / 4095.0f;

        float32_t re[THD_MAX_HARM + 1] = {0};
        float32_t im[THD_MAX_HARM + 1] = {0};
        float32_t omega = 2.0f * 3.14159265f * g_f0 / DFT_FS_HZ;

        for (uint32_t n = 0; n < FFT_LENGTH; n++) {
            float32_t sig  = ((float32_t)snap[n] - mean) * inv_fs * 3.3f * win[n];
            float32_t theta = omega * (float32_t)n;
            float32_t c = cosf(theta);
            float32_t s = sinf(theta);
            float32_t ch = 1.0f, sh = 0.0f;
            for (uint32_t h = 1; h <= THD_MAX_HARM; h++) {
                float32_t ch_n = ch * c - sh * s;
                float32_t sh_n = sh * c + ch * s;
                ch = ch_n; sh = sh_n;
                re[h] += sig * ch;
                im[h] -= sig * sh;
            }
        }

        for (uint32_t h = 1; h <= THD_MAX_HARM; h++)
            g_harms[h] = sqrtf(re[h]*re[h] + im[h]*im[h]) * 4.0f / (float32_t)FFT_LENGTH;

        float32_t harm_pwr = 0.0f;
        for (uint32_t h = 2; h <= THD_MAX_HARM; h++)
            harm_pwr += g_harms[h] * g_harms[h];
        float32_t raw_thd = (g_harms[1] > 1e-9f) ? sqrtf(harm_pwr) / g_harms[1] : 0.0f;

        if (raw_thd >= 0.10f) {
            for (uint32_t h = 2; h <= THD_MAX_HARM; h++) {
                if (g_harms[h] * 1000.0f < THD_NOISE_MV) g_harms[h] = 0.0f;
            }
            harm_pwr = 0.0f;
            for (uint32_t h = 2; h <= THD_MAX_HARM; h++)
                harm_pwr += g_harms[h] * g_harms[h];
            g_thd = (g_harms[1] > 1e-9f) ? sqrtf(harm_pwr) / g_harms[1] : 0.0f;
            if (g_thd < 0.01f) g_thd = 0.0f;
        } else {
            g_thd = raw_thd;
        }
    }

    /* 窗口滤波: THD<10%取均值, ≥10%取最大 */
    {
        static uint8_t buf[THD_WIN_FRAMES] = {0};
        static uint8_t idx = 0, fill = 0;
        uint8_t low_thd = (g_thd < 0.10f);
        uint8_t win = low_thd ? THD_WIN_LO : THD_WIN_FRAMES;

        if (g_flush_filter) { idx = 0; fill = 0; }

        buf[idx] = (uint8_t)(g_thd * 100.0f + 0.5f);
        if (buf[idx] > 100) buf[idx] = 100;
        idx = (idx + 1) % win;
        if (fill < win) fill++;

        if (low_thd) {
            uint16_t sum = 0;
            for (uint8_t i = 0; i < fill; i++) sum += buf[i];
            g_thd = (float32_t)sum / (float32_t)fill / 100.0f;
        } else {
            uint8_t max_val = 0;
            for (uint8_t i = 0; i < fill; i++)
                if (buf[i] > max_val) max_val = buf[i];
            g_thd = (float32_t)max_val / 100.0f;
        }
        g_flush_filter = 0;
    }

    g_ready = 1;
}

/* ==================== 调制识别 (50kHz 载波) ==================== */

/* 在指定区间找前 max_n 个局部峰值, 按幅值降序返回 */
static uint8_t find_top_peaks(uint32_t lo, uint32_t hi, uint32_t exclude_lo,
                              uint32_t exclude_hi, uint8_t max_n,
                              float32_t *out_mags, uint32_t *out_bins)
{
    uint8_t found = 0;
    for (uint32_t i = lo; i <= hi; i++) {
        if (i >= exclude_lo && i <= exclude_hi) continue;
        if (FFT_OutputBuf[i] <= FFT_OutputBuf[i - 1U] ||
            FFT_OutputBuf[i] <= FFT_OutputBuf[i + 1U]) continue;

        float32_t mag = FFT_OutputBuf[i];
        uint8_t pos = found;
        while (pos > 0 && mag > out_mags[pos - 1U]) {
            if (pos < max_n) { out_mags[pos] = out_mags[pos - 1U]; out_bins[pos] = out_bins[pos - 1U]; }
            pos--;
        }
        if (pos < max_n) { out_mags[pos] = mag; out_bins[pos] = i; }
        if (found < max_n) found++;
    }
    return found;
}

/**
 * @brief  50kHz 载波调制识别 + 参数测量
 * @note   读取全局 FFT_OutputBuf, 内部带滤波状态
 * @return DFT_ModResult_t
 */
DFT_ModResult_t DFT_DetectModulation(void)
{
    DFT_ModResult_t r;
    memset(&r, 0, sizeof(r));
    r.type = DFT_MOD_AM;

    float32_t bin_res = DFT_FS_HZ / (float32_t)FFT_LENGTH;
    uint32_t c_bin = (uint32_t)(50000.0f / bin_res + 0.5f);
    uint32_t half  = FFT_LENGTH / 2U;
    uint32_t lo = (c_bin > 120) ? c_bin - 120 : 0U;
    uint32_t hi = c_bin + 120;
    if (hi >= half) hi = half - 1U;

    /* 找最高的 3 个峰 (按幅值降序) */
    float32_t top_mags[3] = {0};
    uint32_t top_bins[3] = {0};
    uint8_t top_n = find_top_peaks(lo, hi, 0, 0, 3, top_mags, top_bins);

    if (top_n < 3) return r;  /* 峰不够, 默认 AM (ma=0) */

    /* 按 bin 位置排序 → [左峰, 中间峰(载波), 右峰] */
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (top_bins[i] > top_bins[j]) {
                float32_t tm = top_mags[i]; top_mags[i] = top_mags[j]; top_mags[j] = tm;
                uint32_t tb = top_bins[i]; top_bins[i] = top_bins[j]; top_bins[j] = tb;
            }
        }
    }

    float32_t left_mag    = top_mags[0];
    float32_t carrier_mag = top_mags[1];
    float32_t right_mag   = top_mags[2];
    uint32_t left_bin     = top_bins[0];
    uint32_t carrier_bin  = top_bins[1];
    uint32_t right_bin    = top_bins[2];

    uint32_t dist_left  = carrier_bin - left_bin;
    uint32_t dist_right = right_bin - carrier_bin;

    /* AM 判别:
     *   1. 左右边带到载波的距离对称 (差 ≤ 3 bin)
     *   2. (左边带 + 右边带) / 2 ≈ 载波幅值
     *   对于标准 AM: (USB+LSB)/2 = ma·A/2, 载波 = A
     *   比值 = ma/2, 典型 AM (ma: 0.1~1.6) 比值范围 0.05~0.8 */
    float32_t side_avg = (left_mag + right_mag) * 0.5f;
    float32_t ratio = (carrier_mag > 1e-9f) ? side_avg / carrier_mag : 0.0f;

    int dist_sym = abs((int)dist_left - (int)dist_right);
    if (dist_sym <= 3 && ratio > 0.05f && ratio < 0.8f) {
        r.type = DFT_MOD_AM;
    }

    /* AM ma 计算 (边带/载波比值法) */
    {
        float32_t raw_depth = 2.0f * ratio * 100.0f;  /* ma (%) */
        r.am_raw = raw_depth;

        static float32_t max_buf[AM_WIN_FRAMES] = {0};
        static uint8_t max_idx = 0;
        max_buf[max_idx] = raw_depth;
        max_idx = (max_idx + 1) % AM_WIN_FRAMES;

        float32_t max_val = max_buf[0];
        for (uint8_t i = 1; i < AM_WIN_FRAMES; i++)
            if (max_buf[i] > max_val) max_val = max_buf[i];
        r.am_peak = max_val;

        static float32_t min_buf[AM_MIN_FRAMES] = {0};
        static uint8_t min_idx = 0, min_fill = 0;
        min_buf[min_idx] = max_val;
        min_idx = (min_idx + 1) % AM_MIN_FRAMES;
        if (min_fill < AM_MIN_FRAMES) min_fill++;

        float32_t min_val = min_buf[0];
        for (uint8_t i = 1; i < min_fill; i++)
            if (min_buf[i] < min_val) min_val = min_buf[i];
        r.am_depth = min_val;
    }

    return r;
}

void DFT_SendSpectrumFrame(void)
{
    if (!g_ready) return;
    g_ready = 0;

    /* header: Fs + N + F0 */
    {
        int32_t  fs_i = (int32_t)512195;
        uint32_t fs_f = 12;
        int32_t  f0_i = (int32_t)g_f0;
        uint32_t f0_f = (uint32_t)((g_f0 - (float32_t)f0_i) * 10.0f + 0.5f);
        if (f0_f >= 10U) { f0_i++; f0_f = 0U; }

        char hdr[80];
        int len = snprintf(hdr, sizeof(hdr),
            "FFT_BEGIN,Fs=%ld.%02lu,N=%u,F0=%ld.%01lu\r\n",
            (long)fs_i, (unsigned long)fs_f,
            (unsigned)FFT_LENGTH, (long)f0_i, (unsigned long)f0_f);
        if (len > 0) {
            while (UART1_TxEnqueue((const uint8_t *)hdr, (uint16_t)len) < 0) {}
        }
    }

    /* spectrum */
    {
        char line[64];
        uint32_t half = FFT_LENGTH / 2U;
        for (uint32_t i = 0; i < half; i++) {
            float32_t val = FFT_OutputBuf[i];
            if (val < 0.0f) val = 0.0f;
            int32_t  vi = (int32_t)val;
            uint32_t vf = (uint32_t)((val - (float32_t)vi) * 1000000.0f + 0.5f);
            if (vf >= 1000000U) { vi++; vf = 0U; }
            int len = snprintf(line, sizeof(line),
                "%lu,%ld.%06lu\r\n",
                (unsigned long)i, (long)vi, (unsigned long)vf);
            if (len > 0) {
                while (UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len) < 0) {}
            }
        }
    }

    /* tail */
    {
        static const char tail[] = "FFT_END\r\n";
        while (UART1_TxEnqueue((const uint8_t *)tail, (uint16_t)(sizeof(tail) - 1U)) < 0) {}
    }

    /* THD */
    {
        char line[32];
        float32_t thd_pct = g_thd * 100.0f;
        int32_t  ti = (int32_t)thd_pct;
        uint32_t tf = (uint32_t)((thd_pct - (float32_t)ti) * 100.0f + 0.5f);
        if (tf >= 100U) { ti++; tf = 0U; }
        int len = snprintf(line, sizeof(line),
            "THD:%ld.%02lu%%\r\n", (long)ti, (unsigned long)tf);
        if (len > 0) {
            while (UART1_TxEnqueue((const uint8_t *)line, (uint16_t)len) < 0) {}
        }
    }

    /* H1~H10 占比 + 波形类型 */
    {
        char line[200]; uint32_t pos = 0;
        pos += (uint32_t)snprintf(line + pos, sizeof(line) - pos, "HARM:");

        for (uint32_t h = 1; h <= THD_MAX_HARM; h++) {
            float32_t pct = (g_harms[1] > 1e-9f)
                ? g_harms[h] / g_harms[1] * 100.0f : 0.0f;
            int32_t  pi = (int32_t)pct;
            uint32_t pf = (uint32_t)((pct - (float32_t)pi) * 10.0f + 0.5f);
            if (pf >= 10U) { pi++; pf = 0U; }
            pos += (uint32_t)snprintf(line + pos, sizeof(line) - pos,
                "%s%ld.%01lu", (h == 1) ? "" : ",", (long)pi, (unsigned long)pf);
        }
        pos += (uint32_t)snprintf(line + pos, sizeof(line) - pos, "\r\n");
        while (UART1_TxEnqueue((const uint8_t *)line, (uint16_t)pos) < 0) {}

        /* 直接通过 THD 判断: <阈值→SINE, ≥阈值→SQUARE */
        const char *type = (g_thd < TYPE_THD_THRESH) ? "SINE" : "SQUARE";

        /* TYPE 去抖: 同类型持续 TYPE_HOLD_FRAMES 帧才切换 */
        {
            static char   out_type[12] = "SINE";
            static char   cand_type[12] = "SINE";
            static uint8_t hold_cnt = 0;

            if (strcmp(type, cand_type) == 0) {
                hold_cnt++;
            } else {
                strncpy(cand_type, type, sizeof(cand_type) - 1);
                hold_cnt = 1;
            }
            if (hold_cnt >= TYPE_HOLD_FRAMES)
                strncpy(out_type, cand_type, sizeof(out_type) - 1);

            type = out_type;
        }

        char tline[24];
        int tlen = snprintf(tline, sizeof(tline), "TYPE:%s\r\n", type);
        while (UART1_TxEnqueue((const uint8_t *)tline, (uint16_t)tlen) < 0) {}
    }

    /* 调制识别 */
    DFT_ModResult_t mod = DFT_DetectModulation();
    char mline[96];
    int mlen;
    if (mod.type == DFT_MOD_FM) {
        int32_t di = (int32_t)mod.fm_df;
        uint32_t df = (uint32_t)((mod.fm_df - (float32_t)di) * 10.0f + 0.5f);
        int32_t fi = (int32_t)mod.fm_fm;
        uint32_t ff = (uint32_t)((mod.fm_fm - (float32_t)fi) * 10.0f + 0.5f);
        if (df >= 10U) { di++; df = 0U; }
        if (ff >= 10U) { fi++; ff = 0U; }
        mlen = snprintf(mline, sizeof(mline), "MOD:FM,%ld.%01lu,%ld.%01lu,%lu,%lu\r\n",
            (long)di, (unsigned long)df,
            (long)fi, (unsigned long)ff,
            (unsigned long)mod.fm_left_bin, (unsigned long)mod.fm_right_bin);
    } else if (mod.am_depth < 0.01f) {
        const char *name = (mod.type == DFT_MOD_ASK) ? "ASK" : "AM";
        mlen = snprintf(mline, sizeof(mline), "MOD:%s\r\n", name);
    } else {
        const char *name = (mod.type == DFT_MOD_ASK) ? "ASK" : "AM";
        int32_t  di = (int32_t)mod.am_depth;
        uint32_t df = (uint32_t)((mod.am_depth - (float32_t)di) * 10.0f + 0.5f);
        int32_t  pi = (int32_t)mod.am_peak;
        uint32_t pf = (uint32_t)((mod.am_peak  - (float32_t)pi) * 10.0f + 0.5f);
        int32_t  ri = (int32_t)mod.am_raw;
        uint32_t rf = (uint32_t)((mod.am_raw   - (float32_t)ri) * 10.0f + 0.5f);
        if (df >= 10U) { di++; df = 0U; }
        if (pf >= 10U) { pi++; pf = 0U; }
        if (rf >= 10U) { ri++; rf = 0U; }
        mlen = snprintf(mline, sizeof(mline), "MOD:%s,%ld.%01lu,%ld.%01lu,%ld.%01lu\r\n",
            name, (long)di, (unsigned long)df, (long)pi, (unsigned long)pf, (long)ri, (unsigned long)rf);
    }
    while (UART1_TxEnqueue((const uint8_t *)mline, (uint16_t)mlen) < 0) {}

    /* ASK 详情 */
    if (mod.type == DFT_MOD_ASK) {
        char aline[96];
        int32_t fi = (int32_t)mod.ask_fm;
        int32_t h1 = (int32_t)(mod.ask_h1 * 1000.0f + 0.5f);
        int32_t h3 = (int32_t)(mod.ask_h3 * 1000.0f + 0.5f);
        int32_t h5 = (int32_t)(mod.ask_h5 * 1000.0f + 0.5f);
        int alen = snprintf(aline, sizeof(aline),
            "ASK:fm=%ld,H1=%ldmV,H3=%ldmV,H5=%ldmV\r\n",
            (long)fi, (long)h1, (long)h3, (long)h5);
        while (UART1_TxEnqueue((const uint8_t *)aline, (uint16_t)alen) < 0) {}
    }

}
