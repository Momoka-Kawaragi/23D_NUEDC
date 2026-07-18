/**
 * @file ZPN_Hmi.h
 * @brief 串口屏控制协议接口头文件
 * @details 提供陶晶驰串口屏(TJC/HMI)的常用控制函数声明，基于UART2非阻塞发送。
 *          所有函数自动添加协议尾(0xFF 0xFF 0xFF)，调用后需检查返回值判断发送状态。
 *
 * @note 模块特性：
 *       1. 纯接口头文件，不包含任何实现代码
 *       2. 所有函数声明前都有详细的Doxygen注释，IDE可智能提示
 *       3. 依赖ZPN_Uart.h提供的UART2_SendPrintf底层接口
 *       4. 通过HMI_MAX_TEXT_LEN宏限制文本长度，防止缓冲区溢出
 *
 * @author 唐韩宇（基于左岚架构重构）
 * @date 2025-11-21
 * @version 2.0
 * @copyright 版权所有
 */

#ifndef __ZPN_HMI_H__
#define __ZPN_HMI_H__

/*============================== 头文件包含 ==============================*/
#include "ZPN_Uart_Config.h"  // 依赖配置宏UART2_TX_BUFFER_SIZE等
#include "ZPN_Uart.h"         // 依赖UART2_SendPrintf声明
#include <stdint.h>           // 提供uint8_t等类型定义

/*============================== 宏定义 ==============================*/

/**
 * @def HMI_MAX_TEXT_LEN
 * @brief 串口屏文本最大长度限制
 * @details 计算方式：UART2_TX_BUFFER_SIZE - 指令头开销 - 协议尾 - 安全余量
 *          超过此长度的文本会被截断，导致显示不完整
 */
#define HMI_MAX_TEXT_LEN  (UART2_TX_BUFFER_SIZE - 20)

/*============================== 函数声明 ==============================*/

/**
 * @brief 发送原始字符串到串口屏（自动添加协议尾）
 */
int HMI_SendString(const char* str);

/**
 * @brief 设置文本控件的显示内容
 * @param objname: 控件名称，格式"页面名.控件名"，如"page1.t0"
 * @param text: 要显示的字符串（自动转义双引号）
 * @retval >=0: 发送字节数（不含协议尾），-1: 发送失败
 */
int HMI_SetText(const char* objname, const char* text);

/**
 * @brief 设置数值控件的整数值
 * @param objname: 控件名称，如"page1.n0"
 * @param value: 整数值（-2147483648~2147483647）
 * @retval >=0: 发送字节数，-1: 发送失败
 */
int HMI_SetValue(const char* objname, int value);

/**
 * @brief 设置数值控件的浮点数值（自动缩放为整数）
 * @param objname: 控件名称，如"page1.n1"
 * @param value: 浮点数值
 * @param decimal: 小数点后保留位数（0~6）
 * @retval >=0: 发送字节数，-1: 发送失败
 */
int HMI_SetFloat(const char* objname, float value, int decimal);

/**
 * @brief 清除波形控件指定通道的数据
 * @param objname: 波形控件名称，如"page1.w0"
 * @param channel: 通道编号（0~3）或0xFF清除所有通道
 * @retval >=0: 发送字节数，-1: 发送失败
 */
int HMI_ClearWave(const char* objname, int channel);

/**
 * @brief 向波形控件添加单点数据（实时模式，适合<100Hz）
 * @param objname: 波形控件名称
 * @param channel: 通道编号（0~3）
 * @param value: 数据值（0~255，需预归一化）
 * @retval >=0: 发送字节数，-1: 发送失败
 */
int HMI_AddWavePoint(const char* objname, int channel, int value);

/**
 * @brief 快速批量发送波形数据（高速模式，可达20kHz）
 * @param objname: 波形控件名称
 * @param channel: 通道编号（0~3）
 * @param len: 数据长度（1~1024）
 * @param values: 数据数组指针（每个元素0~255）
 * @retval >=0: 发送数据点数，-1: 参数无效
 * @warning 调用后需等待约10ms再发送下一条指令
 */
int HMI_FastWaveSend(const char* objname, int channel, int len, const uint8_t* values);

#endif /* __ZPN_HMI_H__ */
