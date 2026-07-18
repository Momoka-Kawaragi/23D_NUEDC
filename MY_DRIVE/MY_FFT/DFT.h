#ifndef __DFT_H
#define __DFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"

/* ==================== 调制识别结果 ==================== */
typedef enum {
    DFT_MOD_FM  = 0,
    DFT_MOD_ASK = 1,
    DFT_MOD_AM  = 2,
} DFT_ModType_t;

typedef struct {
    DFT_ModType_t type;
    /* FM */
    float32_t fm_df;        /* 频偏 (Hz) */
    float32_t fm_fm;        /* 调制频率 (Hz) */
    uint32_t  fm_left_bin;  /* 最左峰值 bin */
    uint32_t  fm_right_bin; /* 最右峰值 bin */
    /* AM (含 ASK) */
    float32_t am_depth;     /* ma 最终值 (%) */
    float32_t am_peak;      /* ma 第一级 max (%) */
    float32_t am_raw;       /* ma 当前帧 (%) */
    /* ASK */
    float32_t ask_fm;       /* 基带频率 (Hz) */
    float32_t ask_h1;       /* H1 幅值 (V) */
    float32_t ask_h3;       /* H3 幅值 (V) */
    float32_t ask_h5;       /* H5 幅值 (V) */
} DFT_ModResult_t;

/* ==================== API ==================== */
void DFT_Process(void);
void DFT_SendSpectrumFrame(void);
DFT_ModResult_t DFT_DetectModulation(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFT_H */
