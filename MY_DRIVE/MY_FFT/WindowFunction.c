/**
 * @file     WindowFunction.c
 * @author   Vincent Cui (Adapted for STM32 DSP)
 * @version  0.3
 * @date     2026-01-29
 * @brief    各种窗函数的C语言实现
 */

#include "WindowFunction.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 内部辅助宏 */
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/* 辅助函数：连乘 */
static double prod(double start, double step, double end) {
    double res = 1.0;
    for (double i = start; i <= end; i += step) {
        res *= i;
    }
    return res;
}

/* 辅助函数：应用窗函数 */
void Window_Apply(float32_t *signal, const float32_t *window, uint16_t length)
{
    if (signal == NULL || window == NULL) return;
    for (uint16_t i = 0; i < length; i++) {
        signal[i] *= window[i];
    }
}

/* ------------------- 窗函数实现 ------------------- */

dspErrorStatus triangularWin(uint16_t N, float32_t w[])
{
    uint16_t i;
    /* 阶数为奇 */
    if ((N % 2) == 1) {
        for (i = 0; i < ((N - 1) / 2); i++) {
            w[i] = (float32_t)(2 * (double)(i + 1) / (N + 1));
        }
        for (i = ((N - 1) / 2); i < N; i++) {
            w[i] = (float32_t)(2 * (double)(N - i) / (N + 1));
        }
    }
    /* 阶数为偶 */
    else {
        for (i = 0; i < (N / 2); i++) {
            w[i] = (float32_t)((i + i + 1) * (double)1 / N);
        }
        for (i = (N / 2); i < N; i++) {
            w[i] = w[N - 1 - i];
        }
    }
    return DSP_SUCESS;
}

dspErrorStatus bartlettWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < (N - 1) / 2; n++) {
        w[n] = (float32_t)(2 * (double)n / (N - 1));
    }
    for (n = (N - 1) / 2; n < N; n++) {
        w[n] = (float32_t)(2 - 2 * (double)n / (N - 1));
    }
    return DSP_SUCESS;
}

dspErrorStatus bartLettHannWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(0.62 - 0.48 * fabs(((double)n / (N - 1)) - 0.5) + 0.38 * cos(2 * PI * (((double)n / (N - 1)) - 0.5)));
    }
    return DSP_SUCESS;
}

dspErrorStatus blackManWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(0.42 - 0.5 * cos(2 * PI * (double)n / (N - 1)) + 0.08 * cos(4 * PI * (double)n / (N - 1)));
    }
    return DSP_SUCESS;
}

dspErrorStatus blackManHarrisWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(BLACKMANHARRIS_A0 - BLACKMANHARRIS_A1 * cos(2 * PI * (double)n / (N)) + \
               BLACKMANHARRIS_A2 * cos(4 * PI * (double)n / (N)) - \
               BLACKMANHARRIS_A3 * cos(6 * PI * (double)n / (N)));
    }
    return DSP_SUCESS;
}

dspErrorStatus bohmanWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    double x;
    for (n = 0; n < N; n++) {
        x = -1 + n * (double)2 / (N - 1);
        x = x >= 0 ? x : (x * (-1));
        w[n] = (float32_t)((1 - x) * cos(PI * x) + (double)(1 / PI) * sin(PI * x));
    }
    return DSP_SUCESS;
}

dspErrorStatus flatTopWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(FLATTOPWIN_A0 - FLATTOPWIN_A1 * cos(2 * PI * (double)n / (N - 1)) + \
               FLATTOPWIN_A2 * cos(4 * PI * (double)n / (N - 1)) - \
               FLATTOPWIN_A3 * cos(6 * PI * (double)n / (N - 1)) + \
               FLATTOPWIN_A4 * cos(8 * PI * (double)n / (N - 1)));
    }
    return DSP_SUCESS;
}

dspErrorStatus gaussianWin(uint16_t N, double alpha, float32_t w[])
{
    uint16_t n;
    double k, beta, theta;
    for (n = 0; n < N; n++) {
        if ((N % 2) == 1) {
            k = n - (N - 1) / 2.0;
        } else {
            k = n - N / 2.0;
        }
        beta = 2 * alpha * k / (N - 1);
        theta = pow(beta, 2);
        w[n] = (float32_t)exp((-1) * theta / 2);
    }
    return DSP_SUCESS;
}

dspErrorStatus hammingWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(0.54 - 0.46 * cos(2 * PI * (double)n / (N - 1)));
    }
    return DSP_SUCESS;
}

dspErrorStatus hannWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(0.5 * (1 - cos(2 * PI * (double)n / (N - 1))));
    }
    return DSP_SUCESS;
}

dspErrorStatus nuttalWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = (float32_t)(NUTTALL_A0 - NUTTALL_A1 * cos(2 * PI * (double)n / (N - 1)) + \
               NUTTALL_A2 * cos(4 * PI * (double)n / (N - 1)) - \
               NUTTALL_A3 * cos(6 * PI * (double)n / (N - 1)));
    }
    return DSP_SUCESS;
}

dspErrorStatus parzenWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    double alpha, k;
    for (n = 0; n < N; n++) {
        k = n - (N - 1) / 2.0;
        alpha = 2 * (double)fabs(k) / N;
        if (fabs(k) <= (N - 1) / 4.0) {
            w[n] = (float32_t)(1 - 6 * pow(alpha, 2) + 6 * pow(alpha, 3));
        } else {
            w[n] = (float32_t)(2 * pow((1 - alpha), 3));
        }
    }
    return DSP_SUCESS;
}

dspErrorStatus rectangularWin(uint16_t N, float32_t w[])
{
    uint16_t n;
    for (n = 0; n < N; n++) {
        w[n] = 1.0f;
    }
    return DSP_SUCESS;
}

dspErrorStatus chebyshevWin(uint16_t N, double r, float32_t w[])
{
    uint16_t n, index;
    double x, alpha, beta, theta, gama;
    
    // 使用临时double数组以保证精度，最后再转float，或者直接计算。
    // 为简化，这里直接计算，但Chebyshev涉及递归乘法，精度敏感。
    // 考虑到嵌入式资源，我们尽量不使用malloc。
    // 原算法使用动态规划方式计算 w[n]。
    // 这里采用简化版本，或者直接在 w[] (float) 上操作，可能损失精度。
    // 为保证精度，最好使用 double buffer。
    // 如果N=1024，double buffer 需要 8KB，可能栈溢出。
    // 建议直接在 w[] 上操作，虽然 w 是 float。
    
    // 10^(r/20)
    theta = pow(10.0, fabs(r) / 20.0);
    beta = pow(cosh(acosh(theta) / (N - 1)), 2);
    alpha = 1 - 1.0 / beta;

    /* 计算一半的区间 */
    // 注意：原算法 w 是 double，这里 w 是 float。
    // 我们先用 float 计算。
    
    w[0] = 1.0f; // placeholder, actually set later
    
    // 由于 chebyshev 算法依赖 w[n] 的累积，使用 float 可能误差较大。
    // 但暂且如此实现。
    
    // 实际上原代码逻辑比较复杂，这里直接搬运并适配类型。
    // 为避免栈溢出，不分配大数组。
    
    // 重新审视原代码：
    // for (n = 1; n < (N + 1) / 2; n++) { ... w[n] = ... }
    // 它是独立计算每个 w[n] 的，依赖内层循环 index。
    // 所以不需要额外的 buffer！可以直接写入 w[n]。
    
    for (n = 1; n < (N + 1) / 2; n++)
    {
        gama = 1;
        for (index = 1; index < n; index++)
        {
            x = index * (double)(N - 1 - 2 * n + index) / ((n - index) * (n + 1 - index));
            gama = gama * alpha * x + 1;
        }
        w[n] = (float32_t)((N - 1) * alpha * gama);
    }

    if ((N % 2) == 1) {
        theta = w[(N - 1) / 2];
        w[0] = 1.0f;
        for (n = 0; n < (N + 1) / 2; n++) {
            w[n] = (float32_t)(w[n] / theta);
        }
        for (; n < N; n++) {
            w[n] = w[N - n - 1];
        }
    } else {
        theta = w[(N / 2) - 1];
        w[0] = 1.0f;
        for (n = 0; n < (N + 1) / 2; n++) {
            w[n] = (float32_t)(w[n] / theta);
        }
        for (; n < N; n++) {
            w[n] = w[N - n - 1];
        }
    }

    return DSP_SUCESS;
}

dspErrorStatus taylorWin(uint16_t N, uint16_t nbar, double sll, float32_t w[])
{
    double A;
    double sf[64]; // 静态分配，假设 nbar < 64
    double alpha, beta, theta;
    uint16_t i, j;
    
    if (nbar >= 64) return DSP_ERROR;

    /* A = R   cosh(PI, A) = R */
    A = acosh(pow(10.0, sll / 20.0)) / PI;
    A = A * A;

    alpha = prod(1, 1, (nbar - 1));
    alpha *= alpha;
    beta = (double)nbar / sqrt(A + pow((nbar - 0.5), 2));
    
    for (i = 1; i <= (nbar - 1); i++)
    {
        sf[i - 1] = prod(1, 1, (nbar - 1 + i)) * prod(1, 1, (nbar - 1 - i));
        theta = 1;
        for (j = 1; j <= (nbar - 1); j++)
        {
            theta *= 1 - (double)(i * i) / (beta * beta * (A + (j - 0.5) * (j - 0.5)));
        }
        sf[i - 1] = alpha * theta / sf[i - 1];
    }

    /* 奇数阶 */
    if ((N % 2) == 1)
    {
        for (i = 0; i < N; i++)
        {
            alpha = 0;
            for (j = 1; j <= (nbar - 1); j++)
            {
                alpha += sf[j - 1] * cos(2 * PI * j * (double)(i - ((N - 1) / 2.0)) / N);
            }
            w[i] = (float32_t)(1 + 2 * alpha);
        }
    }
    /* 偶数阶 */
    else
    {
        for (i = 0; i < N; i++)
        {
            alpha = 0;
            for (j = 1; j <= (nbar - 1); j++)
            {
                alpha += sf[j - 1] * cos(PI * j * (double)(2 * (i - (N / 2)) + 1) / N);
            }
            w[i] = (float32_t)(1 + 2 * alpha);
        }
    }
    return DSP_SUCESS;
}

dspErrorStatus tukeyWin(uint16_t N, double r, float32_t w[])
{
    uint16_t index;
    double alpha;

    /* r <= 0 就是矩形窗 */
    if (r <= 0) {
        return rectangularWin(N, w);
    }
    /* r >= 1 就是汉宁窗 */
    else if (r >= 1) {
        return hannWin(N, w);
    }
    else {
        for (index = 0; index < N; index++) {
            // linSpace logic: x = index / (N-1)
            alpha = (double)index / (N - 1);
            
            if (alpha < (r / 2)) {
                w[index] = (float32_t)((1 + cos(2 * PI * (alpha - r / 2) / r)) / 2);
            }
            else if ((alpha >= (r / 2)) && (alpha < (1 - r / 2))) {
                w[index] = 1.0f;
            }
            else {
                w[index] = (float32_t)((1 + cos(2 * PI * (alpha - 1 + r / 2) / r)) / 2);
            }
        }
    }
    return DSP_SUCESS;
}
