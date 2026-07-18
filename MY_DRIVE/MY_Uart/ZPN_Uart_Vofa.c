/**
 * @file ZPN_Uart_Vofa.c
 * @brief VOFA+ FireWater协议实现（基于UART1 DMA）
 * @details 本文件实现了VOFA+上位机FireWater协议的数据发送功能，包括：
 *          - 多通道采样数据发送（CSV格式）
 *          - 带标签的采样数据发送
 *          - 图片数据发送（前导帧+二进制流）
 *          - 文本调试信息打印
 *          所有发送操作通过UART1_DMAPrintf()和UART1_DMASendData()完成，
 *          具备非阻塞、高吞吐、低CPU占用的特点。
 *
 * @author 唐韩宇（基于zuolan架构重构）
 * @date 2025-11-21
 * @version 2.0
 * @copyright 版权所有
 */

#include "ZPN_Uart_Vofa.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "ZPN_Uart_Vofa.h"

// ✅ 关键：条件编译
#if UART1_MODE == 1
/*============================== 内部常量 ==============================*/

/**
 * @brief 图片前导帧格式模板
 * @details 格式：image:IMG_ID,IMG_SIZE,IMG_WIDTH,IMG_HEIGHT,IMG_FORMAT\n
 * @note 与VOFA+协议手册完全兼容
 */
#define IMAGE_HEADER_FMT "image:%d,%lu,%d,%d,%d\n"

/*============================== 对外接口实现 ==============================*/

/**
 * @brief 发送多通道采样数据（无标签）
 * @details 将浮点数组格式化为CSV字符串，通过DMA发送。
 *          格式："ch0,ch1,ch2,...,chN\n"
 * @param data: 通道数据数组指针，不能为空
 * @param channel_num: 通道数量（1~VOFA_MAX_CHANNELS）
 * @retval >0: 成功启动DMA，返回发送字节数
 * @retval -1: 参数错误（data为空或channel_num越界）
 * @retval -2: UART1忙，上次发送未完成
 * @retval 0: 格式化失败（缓冲区不足）
 * 
 * @note 精度控制：浮点数保留6位小数，满足VOFA+显示需求
 * @warning 调用后需检查uart1_tx_busy状态，防止连续调用失败
 * 
 * @code
 * // 示例1：发送4通道传感器数据
 * float sensors[4] = {1.234f, 5.678f, 9.012f, 3.456f};
 * if(VOFA_SendSamples(sensors, 4) < 0) {
 *     // 处理发送失败
 * }
 * 
 * // 示例2：发送单通道数据
 * float temp = 25.5f;
 * VOFA_SendSamples(&temp, 1); // 发送"25.500000\n"
 * @endcode
 */
int VOFA_SendSamples(const float* data, uint8_t channel_num)
{
    if(data == NULL || channel_num == 0 || channel_num > VOFA_MAX_CHANNELS) {
        return -1; // 参数错误
    }
    
    // 计算所需缓冲区：每个浮点数最多12字符 + 逗号 + 换行
    char buffer[VOFA_MAX_CHANNELS * 12 + 20];
    int offset = 0;
    
    // 构建CSV格式
    for(uint8_t i = 0; i < channel_num; i++) {
        offset += snprintf(&buffer[offset], sizeof(buffer) - offset, 
                          "%s%.6f", (i == 0) ? "" : ",", data[i]);
        if(offset >= sizeof(buffer)) {
            return -1; // 缓冲区溢出
        }
    }
    
    // 添加换行符
    offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "\n");
    
    // 通过DMA发送
    return UART1_DMAPrintf("%s", buffer);
}

/**
 * @brief 发送带标签的多通道采样数据
 * @details 格式："label:ch0,ch1,ch2,...,chN\n"
 * @param label: 标签字符串，可为空或NULL（退化为无标签）
 * @param data: 通道数据数组指针，不能为空
 * @param channel_num: 通道数量（1~VOFA_MAX_CHANNELS）
 * @retval >0: 发送字节数
 * @retval -1: 参数错误（data为空或channel_num越界，label超长）
 * @retval -2: UART1忙
 * @retval 0: 格式化失败
 * 
 * @note 标签长度受VOFA_LABEL_MAX_LEN限制
 * @warning 标签中不要包含冒号':'或换行符'\n'
 * 
 * @code
 * // 示例1：带传感器名称标签
 * float imu[3] = {0.1f, 0.2f, 0.3f};
 * VOFA_SendSamplesWithLabel("Accelerometer", imu, 3);
 * // 发送："Accelerometer:0.100000,0.200000,0.300000\n"
 * 
 * // 示例2：空标签（等同VOFA_SendSamples）
 * VOFA_SendSamplesWithLabel(NULL, data, 4);
 * 
 * // 示例3：带时间戳标签
 * char label[32];
 * snprintf(label, sizeof(label), "T%d", HAL_GetTick());
 * VOFA_SendSamplesWithLabel(label, data, 4);
 * @endcode
 */
int VOFA_SendSamplesWithLabel(const char* label, const float* data, uint8_t channel_num)
{
    if(data == NULL || channel_num == 0 || channel_num > VOFA_MAX_CHANNELS) {
        return -1;
    }
    
    char buffer[VOFA_LABEL_MAX_LEN + VOFA_MAX_CHANNELS * 12 + 20];
    int offset = 0;
    
    // 添加标签
    if(label != NULL && label[0] != '\0') {
        if(strlen(label) >= VOFA_LABEL_MAX_LEN) {
            return -1; // 标签超长
        }
        offset = snprintf(buffer, sizeof(buffer), "%s:", label);
        if(offset >= sizeof(buffer)) return -1;
    }
    
    // 添加数据
    for(uint8_t i = 0; i < channel_num; i++) {
        offset += snprintf(&buffer[offset], sizeof(buffer) - offset, 
                          "%s%.6f", (i == 0 && (label == NULL || label[0] == '\0')) ? "" : ",", data[i]);
        if(offset >= sizeof(buffer)) return -1;
    }
    
    // 添加换行符
    offset += snprintf(&buffer[offset], sizeof(buffer) - offset, "\n");
    
    return UART1_DMAPrintf("%s", buffer);
}

/**
 * @brief 发送图片前导帧（协议头）
 * @details 前导帧格式：image:IMG_ID,IMG_SIZE,IMG_WIDTH,IMG_HEIGHT,IMG_FORMAT\n
 *          必须在发送图片二进制数据前调用此函数
 * @param img_id: 图片通道ID（0~255，对应VOFA+的image控件）
 * @param img_size: 图片数据总大小（字节数）
 * @param width: 图片宽度（像素）
 * @param height: 图片高度（像素）
 * @param format: 图片格式（IMAGE_FORMAT_GRAY8或IMAGE_FORMAT_RGB565）
 * @retval >0: 发送字节数
 * @retval -1: 参数错误（img_size/width/height为0）
 * @retval -2: UART1忙
 * @retval 0: 格式化失败
 * 
 * @note 图片数据必须在成功发送前导帧后立即通过VOFA_SendImageData()发送
 * @warning 如果前导帧发送失败，后续图片数据将被VOFA+解析为普通文本导致乱码
 * 
 * @code
 * // 示例1：发送128x64灰度图
 * uint32_t img_size = 128 * 64; // 8192字节
 * VOFA_SendImageHeader(0, img_size, 128, 64, IMAGE_FORMAT_GRAY8);
 * VOFA_SendImageData(gray_buffer, img_size); // 立即发送数据
 * 
 * // 示例2：发送320x240彩色图（RGB565）
 * uint32_t img_size = 320 * 240 * 2; // 153600字节
 * VOFA_SendImageHeader(1, img_size, 320, 240, IMAGE_FORMAT_RGB565);
 * // 需分包发送153600字节数据...
 * @endcode
 */
int VOFA_SendImageHeader(uint8_t img_id, uint32_t img_size, uint16_t width, uint16_t height, uint8_t format)
{
    if(img_size == 0 || width == 0 || height == 0) {
        return -1; // 参数错误
    }
    
    return UART1_DMAPrintf(IMAGE_HEADER_FMT, img_id, (unsigned long)img_size, width, height, format);
}

/**
 * @brief 发送图片二进制数据（原始流）
 * @details 在VOFA_SendImageHeader()成功后立即调用此函数发送图片像素数据。
 *          数据必须为原始像素流，不做任何封装或转义。
 * @param data: 图片数据缓冲区指针，不能为空
 * @param length: 数据长度（字节数），必须大于0
 * @retval 0: DMA启动成功，数据正在后台发送
 * @retval -1: 参数错误（data为NULL或length为0）
 * @retval -2: UART1忙，前一次DMA发送未完成
 * @retval -3: DMA启动失败
 * 
 * @attention 发送完成前不要修改data缓冲区内容，否则DMA会发送错误数据
 * @warning 大数据量（>512KB）需分包多次调用，并等待uart1_tx_busy清零
 * 
 * @code
 * // 示例1：发送小图片（一次性）
 * uint8_t small_logo[512];
 * load_logo(small_logo);
 * VOFA_SendImageData(small_logo, 512);
 * // 等待uart1_tx_busy=0后再发送下一包
 * 
 * // 示例2：发送大图片（分包发送）
 * uint8_t big_image[153600]; // 320x240 RGB565
 * #define CHUNK_SIZE UART1_DMA_TX_BUFFER_SIZE
 * uint32_t sent = 0;
 * while(sent < 153600) {
 *     if(!uart1_tx_busy) { // DMA空闲
 *         uint16_t chunk = (153600 - sent) > CHUNK_SIZE ? CHUNK_SIZE : (153600 - sent);
 *         VOFA_SendImageData(&big_image[sent], chunk);
 *         sent += chunk;
 *     }
 *     HAL_Delay(1); // 避免忙等
 * }
 * @endcode
 */
int VOFA_SendImageData(const uint8_t* data, uint32_t length)
{
    if(data == NULL || length == 0) {
        return -1; // 参数错误
    }
    
    // 大数据量需循环分包发送
    if(length > UART1_DMA_TX_BUFFER_SIZE) {
        return -2; // 单次发送超限制
    }
    
    return UART1_DMASendData(data, length);
}

/**
 * @brief 打印调试文本到VOFA+文本窗口（自动换行）
 * @details 格式化字符串并自动添加换行符，便于在VOFA+文本控件中查看日志。
 *          相当于printf的DMA非阻塞版本。
 * @param fmt: printf风格格式字符串
 * @param ...: 可变参数
 * @retval >0: 发送字节数
 * @retval -2: UART1忙
 * @retval 0: 格式化失败
 * 
 * @note 自动追加'\n'换行符，如果字符串已以'\n'结尾则不会重复添加
 * @warning 不要在DMA发送完成中断中调用，会导致递归
 * 
 * @code
 * // 示例1：简单日志
 * VOFA_PrintText("System started");
 * // 发送："System started\n"
 * 
 * // 示例2：格式化调试信息
 * VOFA_PrintText("ADC=%d, Temp=%.2f", adc_value, temperature);
 * // 发送："ADC=2048, Temp=25.50\n"
 * 
 * // 示例3：带时间戳
 * VOFA_PrintText("[%lu] Status OK", HAL_GetTick());
 * 
 * // 示例4：错误处理
 * if(VOFA_PrintText("Error: %d", err) < 0) {
 *     // UART1忙，可改为阻塞发送或丢弃
 * }
 * @endcode
 */
int VOFA_PrintText(const char* fmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer) - 2, fmt, args);
    va_end(args);
    
    if(len > 0) {
        // 自动添加换行符（如果用户没加）
        if(len < sizeof(buffer) - 1 && buffer[len-1] != '\n') {
            buffer[len++] = '\n';
            buffer[len] = '\0';
        }
        return UART1_DMAPrintf("%s", buffer);
    }
    return len;
}
#endif
