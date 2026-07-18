/**
 * @file ZPN_Hmi.c
 * @brief 串口屏控制协议实现（基于UART2）
 * @details 本文件封装了陶晶驰串口屏的常用控制指令，所有发送操作通过UART2_SendPrintf()完成，
 *          自动添加协议尾(0xFF 0xFF 0xFF)，采用非阻塞中断方式发送。
 * @note 模块特性：
 *       1. 零耦合：不直接访问硬件，通过ZPN_Uart.h接口通信
 *       2. 非阻塞：调用后立刻返回，不等待发送完成
 *       3. 线程安全：通过uart2_tx_busy标志保护，失败返回-1
 *       4. 自动协议尾：无需手动添加HMI_CMD_TAIL
 *
 * @author 唐韩宇（基于左岚架构重构）
 * @date 2025-11-21
 * @version 2.0
 */

#include "main.h"
#include "ZPN_Hmi.h"
#include "ZPN_Uart.h"
#include "string.h"

/*============================== 私有宏定义 ==============================*/

/**
 * @def HMI_MAX_TEXT_LEN
 * @brief 串口屏文本最大长度限制
 * @note 需小于UART2_TX_BUFFER_SIZE - 协议尾 - 指令开销
 */
#define HMI_MAX_TEXT_LEN  (UART2_TX_BUFFER_SIZE - 20)

/*============================== 对外接口实现 ==============================*/

/**
 * @brief 发送原始字符串到串口屏（自动添加协议尾）
 * @details 适用于发送任意预组装的HMI指令字符串
 * @param str: 指令字符串，如"page 1"或"cle 0,1"
 * @retval >=0: 发送的用户数据字节数（不含协议尾）
 * @retval -1: UART2忙，发送失败
 * 
 * @code
 * // 切换页面
 * HMI_SendString("page 1");
 * // 清屏所有波形
 * HMI_SendString("cle 0,0xff");
 * @endcode
 */
int HMI_SendString(const char* str)
{
    return UART2_SendPrintf("%s", str);
}

/**
 * @brief 设置文本控件的显示内容
 * @details 发送格式：objname.txt="text"\xFF\xFF\xFF
 * @param objname: 控件名称，格式"页面名.控件名"，如"page1.t0"
 * @param text: 要显示的字符串（自动转义双引号）
 * @retval >=0: 发送的用户数据字节数
 * @retval -1: UART2忙或text长度超过HMI_MAX_TEXT_LEN
 * @warning text中不要包含双引号，否则需调用者手动转义
 * 
 * @code
 * // 设置文本框t0显示"Temperature: 25.5°C"
 * HMI_SetText("page1.t0", "Temperature: 25.5C");
 * 
 * // 动态更新带变量的文本
 * char buf[64];
 * snprintf(buf, sizeof(buf), "Temp: %.1fC Hum: %d%%", temp, hum);
 * HMI_SetText("page1.t0", buf); // 需确保buf中无引号
 * @endcode
 */
int HMI_SetText(const char* objname, const char* text)
{
    if(objname == NULL || text == NULL) return -1;
    if(strlen(text) > HMI_MAX_TEXT_LEN) return -1; // 长度保护
    
    return UART2_SendPrintf("%s.txt=\"%s\"", objname, text);
}

/**
 * @brief 设置数值控件的整数值
 * @details 发送格式：objname.val=1234\xFF\xFF\xFF
 * @param objname: 控件名称，如"page1.n0"
 * @param value: 整数值（范围-2147483648~2147483647）
 * @retval >=0: 发送的用户数据字节数
 * @retval -1: UART2忙
 * 
 * @code
 * // 更新数值控件n0的值为512
 * HMI_SetValue("page1.n0", 512);
 * 
 * // 动态显示ADC值
 * HMI_SetValue("page1.n0", HAL_ADC_GetValue(&hadc1) * 3.3f / 4096);
 * @endcode
 */
int HMI_SetValue(const char* objname, int value)
{
    if(objname == NULL) return -1;
    return UART2_SendPrintf("%s.val=%d", objname, value);
}

/**
 * @brief 设置数值控件的浮点数值（自动缩放为整数）
 * @details 原理：value * 10^decimal后转换为整数发送
 *          例如：25.678保留2位小数 → 发送2567
 * @param objname: 控件名称，如"page1.n1"
 * @param value: 浮点数值
 * @param decimal: 小数点后保留位数（0~6）
 * @retval >=0: 发送的用户数据字节数
 * @retval -1: UART2忙或decimal参数无效
 * @note 控件端需设置对应的小数点显示位数
 * 
 * @code
 * // 显示温度值，保留1位小数（如25.5）
 * HMI_SetFloat("page1.n1", temperature, 1);
 * 
 * // 显示百分比，保留2位小数（如99.99）
 * HMI_SetFloat("page1.n2", humidity, 2);
 * 
 * // 错误用法：decimal=8导致整数溢出
 * HMI_SetFloat("page1.n3", value, 8); // ❌ 结果不可预测
 * @endcode
 */
int HMI_SetFloat(const char* objname, float value, int decimal)
{
    if(objname == NULL || decimal < 0 || decimal > 6) return -1;

    static const int pow10[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    int temp = (int)(value * pow10[decimal]);
    return UART2_SendPrintf("%s.val=%d", objname, temp);
}

/**
 * @brief 清除波形控件指定通道的数据
 * @details 发送格式：cle objname,ch\xFF\xFF\xFF
 * @param objname: 波形控件名称，如"page1.w0"
 * @param channel: 通道编号（0~3，或0xFF清除所有通道）
 * @retval >=0: 发送的用户数据字节数
 * @retval -1: UART2忙
 * 
 * @code
 * // 清除波形控件w0的第0通道
 * HMI_ClearWave("page1.w0", 0);
 * 
 * // 清除所有通道
 * HMI_ClearWave("page1.w0", 0xFF);
 * @endcode
 */
int HMI_ClearWave(const char* objname, int channel)
{
    if(objname == NULL) return -1;
    return UART2_SendPrintf("cle %s.id,%d", objname, channel);
}

/**
 * @brief 向波形控件添加单点数据（实时模式）
 * @details 发送格式：add objname.id,ch,val\xFF\xFF\xFF
 *          适合低速数据更新（<100Hz），每次调用发送一个点
 * @param objname: 波形控件名称，如"page1.w0"
 * @param channel: 通道编号（0~3）
 * @param value: 数据值（0~255，需预先归一化）
 * @retval >=0: 发送的用户数据字节数
 * @retval -1: UART2忙
 * @note 波形控件id属性需在串口屏设计中预先设置
 * 
 * @code
 * // ADC值归一化后添加到波形
 * uint16_t adc = HAL_ADC_GetValue(&hadc1);
 * uint8_t wave_val = adc * 255 / 4096; // 归一化到0-255
 * HMI_AddWavePoint("page1.w0", 0, wave_val);
 * 
 * // 每个通道独立更新
 * HMI_AddWavePoint("page1.w0", 0, val0);
 * HMI_AddWavePoint("page1.w0", 1, val1);
 * @endcode
 */
int HMI_AddWavePoint(const char* objname, int channel, int value)
{
    if(objname == NULL || channel < 0 || channel > 3) return -1;
    if(value < 0 || value > 255) return -1; // 值域检查
    
    return UART2_SendPrintf("add %s.id,%d,%d", objname, channel, value);
}

/**
 * @brief 快速批量发送波形数据（高速模式，可达20kHz）
 * @details **重要说明**：虽然UART2使用中断发送，但批量数据部分采用阻塞发送以确保时序精度。
 *          串口屏协议要求addt指令头与数据字节之间不能有延迟，故使用HAL_UART_Transmit()死等每个字节发送完成。
 * @param objname: 波形控件名称
 * @param channel: 通道编号（0~3）
 * @param len: 数据长度（1~1024）
 * @param values: 数据数组指针（每个元素0~255）
 * @retval >=0: 发送数据点数，-1: 参数无效
 * @warning 调用期间会阻塞CPU约 len*87μs（@115200波特率），不得在实时任务中调用！
 * 
 * @code
 * // 在1ms任务中发送128点数据（阻塞11ms，不可行）
 * void Task_1ms(void) {
 *     HMI_FastWaveSend("w0", 0, 128, data); // ❌ 破坏实时性
 * }
 * 
 * // 正确用法：在后台任务或低优先级循环中调用
 * void Task_Background(void) {
 *     if(!uart2_tx_busy) { // 等上包发送完成
 *         HMI_FastWaveSend("w0", 0, 128, data);
 *     }
 * }
 * @endcode
 */
int HMI_FastWaveSend(const char* objname, int channel, int len, const uint8_t* values)
{
    if(objname == NULL || values == NULL) return -1;
    if(channel < 0 || channel > 3) return -1;
    if(len <= 0 || len > 1024) return -1;

    // 等上一包发完再入队，避免超时后指令头残留
    uint32_t t0 = HAL_GetTick();
    while (!UART2_TxIsIdle()) {
        if ((HAL_GetTick() - t0) > 200U) return -1;
    }

    int ret = UART2_SendPrintf("addt %s.id,%d,%d", objname, channel, len);
    if (ret < 0) return ret;

    HAL_Delay(10);

    if (UART2_TxEnqueue(values, (uint16_t)len) < 0) return -1;

    uint8_t end_cmd[1] = {0x01};
    if (UART2_TxEnqueue(end_cmd, 1U) < 0) return -1;

    return len;
}
