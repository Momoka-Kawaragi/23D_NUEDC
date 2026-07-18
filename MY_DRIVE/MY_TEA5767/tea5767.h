/*********************************************************************
 * @file    tea5767.h
 * @brief   TEA5767 FM radio driver (软件 bit-bang I2C, DriverIO 抽象)
 *
 * CubeMX 引脚: 设 2 个 GPIO_Output (开漏) + User Label
 *   - TEA5767_SCL
 *   - TEA5767_SDA
 * driver 通过 DriverIO.h 的 WRT()/GET() 操作引脚对象,
 * 改板子只需要改 CubeMX 里的 User Label (无需改 .c / .h).
 *********************************************************************/

#ifndef _TEA5767_H_
#define _TEA5767_H_

#include <stdint.h>
#include "main.h"
#include "DriverIO.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C 地址 */
#define TEA5767_I2C_ADDR_7BIT   0x60u
#define TEA5767_I2C_ADDRESS     (TEA5767_I2C_ADDR_7BIT << 1)

/* 频段 / 默认值 (可用 -DTEA5767_DEFAULT_KHZ=xxx 覆盖) */
#ifndef TEA5767_MIN_KHZ
#define TEA5767_MIN_KHZ         76000
#endif

#ifndef TEA5767_MAX_KHZ
#define TEA5767_MAX_KHZ         108000
#endif

#ifndef TEA5767_DEFAULT_KHZ
#define TEA5767_DEFAULT_KHZ     88000
#endif

/* 模式常量 */
#define TEA5767_MUTE_ON         1u
#define TEA5767_MUTE_OFF        0u
#define TEA5767_AUDIO_MONO      1u
#define TEA5767_AUDIO_STEREO    0u
#define TEA5767_SEARCH_UP       1u
#define TEA5767_SEARCH_DOWN     0u

/* 当前调谐频率 (kHz) */
extern uint32_t g_frequency;

/* 状态快照 (GetStatus 一次性填好) */
typedef struct {
    uint32_t frequency_khz;
    uint8_t  rf;
    uint8_t  stereo;
    uint8_t  if_counter;
    uint8_t  level_adc;
    uint8_t  ready;
} TEA5767_Status_t;

/* API */
void     TEA5767_Init(void);
void     TEA5767_Write(void);
int      TEA5767_Read(void);
void     TEA5767_GetPLL(void);
void     TEA5767_SetFrequency(uint32_t frequency_khz);
uint32_t TEA5767_GetFrequency(void);
void     TEA5767_Search(uint8_t mode);
void     TEA5767_AutoSearch(uint8_t mode);
void     TEA5767_Mute(uint8_t mode);
void     TEA5767_SetAudioMode(uint8_t mode);

void     TEA5767_GetStatus(TEA5767_Status_t *out);
uint8_t  TEA5767_IsLocked(void);
uint8_t  TEA5767_IsStereo(void);
uint8_t  TEA5767_GetRSSI(void);
uint8_t  TEA5767_GetIFCounter(void);

#ifdef __cplusplus
}
#endif

#endif /* _TEA5767_H_ */
