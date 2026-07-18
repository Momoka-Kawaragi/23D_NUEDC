/**
 * @file     WindowFunction.h
 * @author   Vincent Cui (Adapted for STM32 DSP)
 * @version  0.3
 * @date     2026-01-29
 * @brief    各种窗函数的C语言实现头文件
 */

#ifndef __WINDOWFUNCTION_H
#define __WINDOWFUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h" // 包含 float32_t 定义
#include <stdint.h>

/* 类型定义 */
typedef uint16_t dspUint_16;
typedef double   dspDouble; // 内部计算使用双精度

typedef enum {
    DSP_SUCESS = 0,
    DSP_ERROR = 1
} dspErrorStatus;

/* 常量定义 */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 窗函数参数宏 (Blackman-Harris) */
#define BLACKMANHARRIS_A0 0.35875
#define BLACKMANHARRIS_A1 0.48829
#define BLACKMANHARRIS_A2 0.14128
#define BLACKMANHARRIS_A3 0.01168

/* 窗函数参数宏 (Flat Top) */
#define FLATTOPWIN_A0 0.21557895
#define FLATTOPWIN_A1 0.41663158
#define FLATTOPWIN_A2 0.277263158
#define FLATTOPWIN_A3 0.083578947
#define FLATTOPWIN_A4 0.006947368

/* 窗函数参数宏 (Nuttall) */
#define NUTTALL_A0 0.3635819
#define NUTTALL_A1 0.4891775
#define NUTTALL_A2 0.1365995
#define NUTTALL_A3 0.0106411

/* 函数声明 */
/* 注意：输出数组 w 的类型改为 float32_t 以适配 ARM DSP 库 */

/**
 * @brief  计算三角窗 (Triangular Window)
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus triangularWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Bartlett 窗
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus bartlettWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Bartlett-Hann 窗
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus bartLettHannWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Blackman 窗 (标准)
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus blackManWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Blackman-Harris 窗 (4项)
 * @note   旁瓣极低，适合高动态范围测量
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus blackManHarrisWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Bohman 窗
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus bohmanWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Chebyshev 窗
 * @param  N: 窗长度
 * @param  r: 旁瓣衰减 (dB), 例如 100 表示 -100dB
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus chebyshevWin(uint16_t N, double r, float32_t w[]);

/**
 * @brief  计算 Flat Top 窗 (平顶窗)
 * @note   幅值测量精度最高，但频率分辨率较差
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus flatTopWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Gaussian 窗 (高斯窗)
 * @param  N: 窗长度
 * @param  alpha: 形状参数 (通常 2.5)
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus gaussianWin(uint16_t N, double alpha, float32_t w[]);

/**
 * @brief  计算 Hamming 窗 (汉明窗)
 * @note   旁瓣抑制优于 Hanning，但不归零
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus hammingWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Hanning 窗 (汉宁窗/Hann)
 * @note   通用性强，频谱泄漏低，适合大多数连续信号
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus hannWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Nuttall 窗
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus nuttalWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Parzen 窗
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus parzenWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Rectangular 窗 (矩形窗)
 * @note   相当于不加窗，频率分辨率最高，但泄漏严重
 * @param  N: 窗长度
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus rectangularWin(uint16_t N, float32_t w[]);

/**
 * @brief  计算 Taylor 窗 (泰勒窗)
 * @note   雷达领域常用，可控制旁瓣电平
 * @param  N: 窗长度
 * @param  nbar: 旁瓣数量参数 (例如 4)
 * @param  sll: 旁瓣电平 (dB), 正数表示衰减量 (例如 40 表示 -40dB)
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus taylorWin(uint16_t N, uint16_t nbar, double sll, float32_t w[]);

/**
 * @brief  计算 Tukey 窗
 * @param  N: 窗长度
 * @param  r: 余弦比例参数 (0=矩形窗, 1=汉宁窗)
 * @param  w: 输出系数数组
 * @return dspErrorStatus
 */
dspErrorStatus tukeyWin(uint16_t N, double r, float32_t w[]);

/**
 * @brief  应用窗函数到信号
 * @param  signal: 信号数组 (将被修改: signal[i] *= window[i])
 * @param  window: 窗函数系数数组
 * @param  length: 数组长度
 */
void Window_Apply(float32_t *signal, const float32_t *window, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __WINDOWFUNCTION_H */
