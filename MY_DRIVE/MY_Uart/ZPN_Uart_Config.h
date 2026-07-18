#ifndef __ZPN_UART_CONFIG_H__
#define __ZPN_UART_CONFIG_H__

/* ==================== 软件配置参数 ==================== */

// UART1 DMA缓冲区配置（仅发送）
#define UART1_DMA_TX_BUFFER_SIZE    512
#define UART1_DMA_RX_BUFFER_SIZE    512

// UART2 环形缓冲区配置（串口屏双向通信）
#define UART2_RING_BUFFER_SIZE      1024    // 建议增大到1024以应对突发数据
#define UART2_TX_BUFFER_SIZE        1024

// UART3 阻塞发送配置
#define UART3_TX_BUFFER_SIZE        256

/* ==================== UART1模式选择（二选一） ==================== */
/**
 * @def UART1_MODE
 * @brief UART1工作模式选择
 * @details 0: 通用DMA发送模式（仅提供发送接口）
 *          1: VOFA+协议模式（启用ZPN_Uart_Vofa.c）
 */
#define UART1_MODE                  1       // 默认通用模式，如需VOFA+则改为1

/* ==================== UART2模式选择 ==================== */
/**
 * @def UART2_MODE
 * @brief UART2工作模式选择
 * @details 0: 通用环形缓冲模式（仅提供底层收发接口）
 *          1: HMI串口屏模式（启用ZPN_Hmi.c + Pack自动解析）
 * @warning 启用Pack解析必须设置UART2_MODE=1
 */
#define UART2_MODE                  1       // 必须设为1以启用串口屏和Pack解析

/* ==================== UART3调试开关 ==================== */
#define UART3_MODE_DEBUG            1       // 1: 启用UART3阻塞调试函数

/* ==================== 串口屏协议配置 ==================== */
#if UART2_MODE == 1
#define HMI_CMD_TAIL                "\xFF\xFF\xFF"
#define HMI_CMD_TAIL_LEN            3
#endif

/* ==================== VOFA+协议配置 ==================== */
#if UART1_MODE == 1
#define VOFA_MAX_CHANNELS           16
#define VOFA_LABEL_MAX_LEN          32
#define VOFA_SEND_TIMEOUT_MS        50
#endif

/* ==================== 全局初始化开关 ==================== */
#define ZPN_UART_GLOBAL_ENABLE      1

#endif
