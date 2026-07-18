/**
 ******************************************************************************
 * @file    ModRecognizer.h
 * @brief   调制识别模块 — 仿 23Df4070010 工程, CMSIS-DSP FFT + 6类型识别
 ******************************************************************************
 */

#ifndef __MOD_RECOGNIZER_H
#define __MOD_RECOGNIZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"

/* ==================== 调制类型 ==================== */
typedef enum {
    MR_TYPE_CW    = 0,
    MR_TYPE_AM    = 1,
    MR_TYPE_2ASK  = 2,
    MR_TYPE_2PSK  = 3,
    MR_TYPE_FM    = 4,
    MR_TYPE_2FSK  = 5,
    MR_TYPE_NONE  = 0xFF  /* 无新数据 */
} MR_ModType_t;

/* ==================== 识别结果 ==================== */
typedef struct {
    MR_ModType_t type;       /* 调制类型 */
    /* 通用 */
    float     carrier_freq;  /* 载波频率 (Hz) */
    uint16_t  carrier_bin;   /* 载波 bin */
    float     carrier_mag;   /* 载波幅值 */
    uint8_t   peak_count;    /* 总峰数 */
    /* AM */
    float     ma;            /* 调制度 (%) = 2*边带/载波*100 */
    /* FM */
    float     mf;            /* 调频指数 */
    float     fm;            /* 调制频率 (Hz) */
    float     df;            /* 频偏 (Hz) = mf * fm */
    /* ASK / PSK */
    float     baud;          /* 波特率 (Hz) */
    /* FSK */
    float     h;             /* 调制指数 = Δf / baud */
    uint16_t  f1;            /* 低频点 bin */
    uint16_t  f2;            /* 高频点 bin */
    /* 拟合 */
    float     fit_error;     /* 贝塞尔拟合误差 (仅FM) */
} MR_Result_t;

/* ==================== API ==================== */

void        MR_Init(void);
MR_Result_t MR_Process(void);
void        MR_SendResult(const MR_Result_t *r);  /* UART1 发送识别结果 */

#ifdef __cplusplus
}
#endif

#endif /* __MOD_RECOGNIZER_H */
