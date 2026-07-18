#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

void Delay_Init(void);          // 初始化 DWT 或定时器
void Delay_us(uint32_t us);     // 微秒延时
void Delay_ns(uint32_t ns);     // 纳秒级近似延时（基于循环，精度受系统频率影响）

#endif // __DELAY_H
