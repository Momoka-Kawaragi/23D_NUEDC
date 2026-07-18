#ifndef __ZPN_HMI_PACK_H
#define __ZPN_HMI_PACK_H

#include <stdint.h>

// --- 配置常量 (请根据您的实际串口屏协议设置) ---
#define PACK_FRAME_HEADER   0x55        // 协议帧头 (例如：0x55)
#define PACK_FRAME_TAIL     0xFF        // 协议帧尾 (例如：0xFF, 这里需要根据实际协议确定)
#define PACK_MIN_FRAME_SIZE 4           // 最小帧长：帧头(1) + 数据(1) + 校验和(1) + 帧尾(1)
#define PACK_BUFFER_SIZE    256         // 主循环解析静态缓冲区的大小，必须大于可能的最长帧。
#define PACK_MAX_VARIABLES  32          // 最大支持的变量模板数量

// 变量类型枚举
typedef enum {
    PACK_TYPE_BYTE = 1,
    PACK_TYPE_SHORT,    // 2 字节
    PACK_TYPE_INT,      // 4 字节 (包括 uint32_t, int32_t, float)
    PACK_TYPE_FLOAT = PACK_TYPE_INT // 浮点数也占用 4 字节
} PackDataType_t;

// 变量模板结构体
typedef struct {
    PackDataType_t type;    // 变量数据类型
    void *variable;         // 变量地址指针
} PackVariable_t;


// --- 外部函数声明 ---

/**
 * @brief 设置数据包的变量模板。
 * @param vars: 变量模板数组的指针。
 * @param count: 模板数组中元素的数量。
 * @note 数据包的结构定义
 */
void PACK_SetTemplate(PackVariable_t *vars, uint8_t count);

/**
 * @brief 解析一个完整的Pack帧（内部使用）。
 * @param buffer: 数据缓冲区指针。
 * @param length: 帧数据的长度。
 */
void PACK_ParseFrame(uint8_t *buffer, uint16_t length);

/**
 * @brief 从 UART2 环形缓冲区查找并解析完整的串行数据包。
 * @note 应在主循环中周期性调用。
 */
void PACK_ParseFromRingBuffer(void);

void PACK_ResetParser(void);

#endif // __ZPN_HMI_PACK_H
