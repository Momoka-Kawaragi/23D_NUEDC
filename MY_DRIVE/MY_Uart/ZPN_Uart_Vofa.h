#ifndef __ZPN_UART_VOFA_H__
#define __ZPN_UART_VOFA_H__

#include "ZPN_Uart.h"

/* ==================== 核心接口 ==================== */
int VOFA_SendSamples(const float* data, uint8_t channel_num);
int VOFA_SendSamplesWithLabel(const char* label, const float* data, uint8_t channel_num);
int VOFA_SendImageHeader(uint8_t img_id, uint32_t img_size, uint16_t width, uint16_t height, uint8_t format);
int VOFA_SendImageData(const uint8_t* data, uint32_t length);
int VOFA_PrintText(const char* fmt, ...);

#endif
