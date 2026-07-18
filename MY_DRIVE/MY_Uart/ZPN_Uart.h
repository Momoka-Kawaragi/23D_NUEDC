#ifndef __ZPN_UART_H__
#define __ZPN_UART_H__

#include "usart.h"
#include "ZPN_Hmi_Pack.h"
#ifndef __MAIN_H
   #error "Please include main.h before ZPN_Uart.h"
#endif

#include "ZPN_Uart_Config.h"

/* ==================== UART1 (DMA模式) ==================== */
int UART1_TxEnqueue(const uint8_t *data, uint16_t length);
uint8_t UART1_TxIsIdle(void);
void UART1_RxEnsureRunning(void);
int UART1_DMAPrintf(const char *fmt, ...);
int UART1_DMASendData(const uint8_t *data, uint16_t length);
uint8_t UART1_IsDataReady(void);
uint16_t UART1_GetReceivedData(uint8_t *buf, uint16_t bufSize);

/* ==================== UART2 (环形缓冲模式) ==================== */
int UART2_TxEnqueue(const uint8_t *data, uint16_t length);
uint8_t UART2_TxIsIdle(void);
int UART2_SendPrintf(const char *fmt, ...);
int UART2_SendData(const uint8_t *data, uint16_t length);
uint16_t UART2_GetRxBufferLength(void);
uint16_t UART2_GetReceivedData(uint8_t *buf, uint16_t bufSize);

/* ==================== UART3 (阻塞模式) ==================== */
int UART3_Printf(const char *fmt, ...);
int UART3_SendData(const uint8_t *data, uint16_t length);
/* ==================== 通用初始化 ==================== */
void ZPN_UART_Init(void);

#endif
