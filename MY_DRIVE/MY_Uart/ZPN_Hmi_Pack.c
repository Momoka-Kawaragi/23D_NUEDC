#include "ZPN_Hmi_Pack.h"
#include "ZPN_Uart.h"
#include "main.h"
#include <string.h>
#include <stddef.h>

// 静态变量
static PackVariable_t packVars[PACK_MAX_VARIABLES];
static uint8_t packVarCount = 0;

// 静态解析缓冲区和当前长度
static uint8_t parse_buffer[PACK_BUFFER_SIZE]; // 用于主循环中拼接和解析帧的缓冲区
static uint16_t current_len = 0;               // 当前解析缓冲区中的数据长度

/* 设置 Pack 变量模板 (保持不变) */
void PACK_SetTemplate(PackVariable_t *vars, uint8_t count)
{
    if (vars == NULL || count == 0)
        return;
    if (count > PACK_MAX_VARIABLES)
        count = PACK_MAX_VARIABLES;
    memcpy(packVars, vars, count * sizeof(PackVariable_t));
    packVarCount = count;
}

/* 解析 Pack 帧 (保持不变，但请确保校验和逻辑与您的串口屏协议一致) */
void PACK_ParseFrame(uint8_t *buffer, uint16_t length)
{
    // 1. 基本校验：长度和帧头帧尾
    if (buffer == NULL || length < PACK_MIN_FRAME_SIZE)
        return;
    if (buffer[0] != PACK_FRAME_HEADER || buffer[length - 1] != PACK_FRAME_TAIL)
        return;

    // 兼容 4 字节控制帧：55 [cmd] [value] FF（无校验）
    // 例如：55 04 01 FF / 55 04 00 FF / 55 05 05 FF / 55 06 00 FF
    if (length == 4)
    {
        if (packVarCount >= 1 && packVars[0].variable != NULL)
        {
            *(uint8_t *)packVars[0].variable = buffer[1];
        }
        if (packVarCount >= 2 && packVars[1].variable != NULL)
        {
            // 对于 4 字节帧，我们将第三个字节作为 value 存入 global 变量
            *(uint32_t *)packVars[1].variable = (uint32_t)buffer[2];
        }
        return;
    }

    // 2. 校验和检查 (假设校验和在倒数第二字节，且是包体所有字节的简单累加)
    uint8_t checksum = 0;
    // 从索引 1 (第一个数据字节) 开始，到倒数第三个字节 (数据结束)
    for (uint16_t i = 1; i < length - 2; i++)
    {
        checksum += buffer[i];
    }
    if (checksum != buffer[length - 2])
    {
        // 校验和错误，可能为数据损坏，直接丢弃该帧。
        return;
    }

    // 3. 数据解析并赋值给模板变量
    uint16_t index = 1; // 跳过帧头
    for (uint8_t i = 0; i < packVarCount; i++)
    {
        // 检查是否还有足够的字节来解析下一个变量
        if (index >= length - 2)
            break; // 索引已到达校验和或帧尾，提前退出

        switch (packVars[i].type)
        {
        case PACK_TYPE_BYTE:
            // packVars[i].variable 是一个 void* 指针，它存储着全局变量的地址。
            // 1. (uint8_t*)packVars[i].variable：将这个 void* 强制转换为 uint8_t 类型的指针。
            // 2. *(...)：进行解引用操作，即访问该指针所指向的内存位置。
            // 3. = buffer[index++];：将从串口数据包中取出的字节赋值给该内存位置。
            *(uint8_t *)packVars[i].variable = buffer[index++];
            break;

        case PACK_TYPE_SHORT:
            if (index + 2 > length - 2)
                goto parse_done; // 检查边界
            // 高位在前 (Big Endian) 转换为小端序系统
            *(uint16_t *)packVars[i].variable = (buffer[index] << 8) | buffer[index + 1];
            index += 2;
            break;
        case PACK_TYPE_INT:
            // 注：PACK_TYPE_FLOAT == PACK_TYPE_INT，所以此分支也处理浮点数情况
            if (index + 4 > length - 2)
                goto parse_done; // 检查边界
            // ✓ 正确的大端序转换：高位在前的串口数据 → 小端序MCU内存
            // 数据包中：[高字节][次高][次低][低字节]
            // 转换为MCU小端序：(高字节<<24) | (次高<<16) | (次低<<8) | (低字节)
            *(uint32_t *)packVars[i].variable = 
                ((uint32_t)buffer[index] << 24) |
                ((uint32_t)buffer[index+1] << 16) |
                ((uint32_t)buffer[index+2] << 8) |
                ((uint32_t)buffer[index+3]);
            index += 4;
            break;

        // 默认情况下跳过或打印错误
        default:
            break;
        }
    }

parse_done:
    // 解析完成，变量已经被修改。可以在这里添加一个成功处理的日志或标志。
    return;
}

/* 从 UART2 环形缓冲区非阻塞解析 (核心修改) */
void PACK_ParseFromRingBuffer(void)
{
    // 1. 从环形缓冲区读取数据并追加到静态解析缓冲区
    uint16_t available_len = UART2_GetRxBufferLength();
    if (available_len == 0)
    {
        return;
    }

    // 计算本次最多能读取的字节数，以填充 parse_buffer
    uint16_t read_max = PACK_BUFFER_SIZE - current_len;
    uint16_t read_len = (available_len < read_max) ? available_len : read_max;

    if (read_len > 0)
    {
        // 读取数据并追加到静态解析缓冲区末尾
        uint16_t actual_read = UART2_GetReceivedData(parse_buffer + current_len, read_len);
        current_len += actual_read;
    }

    // 2. 循环查找完整的帧
    // 至少需要 PACK_MIN_FRAME_SIZE 字节才可能包含一个完整的帧
    while (current_len >= PACK_MIN_FRAME_SIZE)
    {
        uint16_t start_index = 0;
        uint16_t end_index = 0;

        // 查找帧头 (0x55)
        for (uint16_t i = 0; i < current_len; i++)
        {
            if (parse_buffer[i] == PACK_FRAME_HEADER)
            {
                start_index = i;
                break;
            }
        }

        // a. 处理帧头之前的垃圾数据
        if (start_index > 0)
        {
            // 移动后续数据到缓冲区开头，丢弃垃圾数据
            memmove(parse_buffer, parse_buffer + start_index, current_len - start_index);
            current_len -= start_index;
            // 重新检查剩余长度
            if (current_len < PACK_MIN_FRAME_SIZE)
            {
                return; // 不够一个最小帧的长度，等待下次接收
            }
        }

        // 此时，parse_buffer[0] 已经是帧头 PACK_FRAME_HEADER

        // b. 查找帧尾 (0xFF)
        // 从最小帧尾可能出现的位置开始查找，假设帧尾不会出现在数据中。
        for (uint16_t i = PACK_MIN_FRAME_SIZE - 1; i < current_len; i++)
        {
            if (parse_buffer[i] == PACK_FRAME_TAIL)
            {
                end_index = i;
                break; // 找到帧尾
            }
        }

        // 未找到完整的帧尾
        if (end_index == 0)
        {
            // 如果缓冲区已满但仍未找到帧尾，说明帧可能太长，丢弃当前帧头，等待下一个。
            if (current_len == PACK_BUFFER_SIZE)
            {
                memmove(parse_buffer, parse_buffer + 1, current_len - 1);
                current_len -= 1;
                continue; // 重新从下一个字节开始查找帧头
            }
            return; // 等待更多数据
        }

        // c. 找到了完整的帧
        uint16_t frame_len = end_index + 1;

        // 3. 解析完整的帧
        PACK_ParseFrame(parse_buffer, frame_len);

        // 4. 移除已解析的帧，移动剩余数据
        if (current_len >= frame_len)
        {
            // 移动剩余数据到缓冲区头部
            memmove(parse_buffer, parse_buffer + frame_len, current_len - frame_len);
            current_len -= frame_len;
        }
        else
        {
            // 逻辑上不应该发生，但以防万一
            current_len = 0;
            break;
        }

        // 继续循环，检查剩余数据中是否包含下一个完整的帧
    }
}

void PACK_ResetParser(void)
{
    current_len = 0;
}
