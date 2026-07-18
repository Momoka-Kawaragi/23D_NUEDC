/*********************************************************************
 * @file    tea5767.c
 * @brief   TEA5767 FM radio driver (软件 bit-bang I2C, DriverIO 抽象)
 *
 * 引脚用 CubeMX User Label:
 *   - TEA5767_SCL (开漏)
 *   - TEA5767_SDA (开漏)
 *
 * 跟参考 board_tea5767.c 的差异:
 *   - I2C: HAL_I2C1  →  软件 bit-bang (本工程未启用 I2C1)
 *   - 引脚: 硬编码   →  CubeMX User Label (任意引脚, 改板子只改 .ioc)
 *   - 新增 GetStatus / IsLocked / IsStereo / GetRSSI / GetIFCounter
 *********************************************************************/

#include <stdio.h>
#include "stm32f4xx_hal.h"

#include "tea5767.h"
#include "DriverIO.h"

/* ============================================================
 *  引脚对象 (CubeMX User Label 自动展开)
 *  注意: 必须在 CubeMX 把这两个 GPIO 配成 GPIO_OUTPUT_OD (开漏)
 *  且板子上有 4.7 kΩ 上拉电阻, 否则 I2C 不会响应
 * ============================================================ */
driverIO TEA5767_SCL = {TEA5767_SCL_GPIO_Port, TEA5767_SCL_Pin};
driverIO TEA5767_SDA = {TEA5767_SDA_GPIO_Port, TEA5767_SDA_Pin};

/* ============================================================
 *  Driver 内部状态 (与参考 driver 一致)
 * ============================================================ */
uint32_t g_frequency = TEA5767_DEFAULT_KHZ;

static uint8_t  s_radioWriteData[5] = {0x31, 0xA0, 0x20, 0x11, 0x00};
static uint8_t  s_radioReadData[5]  = {0};
static uint32_t s_pll = 0;

/* ============================================================
 *  软件 bit-bang I2C
 * ============================================================ */

/* 短延时 ~5 us @168 MHz (空循环不需要精准) */
static inline void tea5767_delay(void)
{
    volatile uint32_t n = 168 / 4;
    while (n--) __NOP();
}

static inline void tea5767_scl_high(void) { WRT(TEA5767_SCL, 1); }
static inline void tea5767_scl_low (void) { WRT(TEA5767_SCL, 0); }
static inline void tea5767_sda_high(void) { WRT(TEA5767_SDA, 1); }
static inline void tea5767_sda_low (void) { WRT(TEA5767_SDA, 0); }
static inline uint8_t tea5767_sda_read(void) { return GET(TEA5767_SDA); }

/* I2C START: SCL=H, SDA 由 H → L */
static void tea5767_i2c_start(void)
{
    tea5767_sda_high();
    tea5767_scl_high();
    tea5767_delay();
    tea5767_sda_low();
    tea5767_delay();
    tea5767_scl_low();
    tea5767_delay();
}

/* I2C STOP: SCL=H, SDA 由 L → H */
static void tea5767_i2c_stop(void)
{
    tea5767_sda_low();
    tea5767_scl_high();
    tea5767_delay();
    tea5767_sda_high();
    tea5767_delay();
}

/* 写一字节, 返回 ACK (0=ACK, 1=NACK) */
static uint8_t tea5767_i2c_write_byte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) tea5767_sda_high();
        else             tea5767_sda_low();
        tea5767_delay();
        tea5767_scl_high();
        tea5767_delay();
        tea5767_scl_low();
        byte <<= 1;
    }
    /* 第 9 个 clock: 释放 SDA, 读 ACK */
    tea5767_sda_high();
    tea5767_delay();
    tea5767_scl_high();
    tea5767_delay();
    uint8_t ack = tea5767_sda_read();
    tea5767_scl_low();
    return ack;
}

/* 读一字节, send_ack=1 表示主发 ACK (继续), 0 表示 NACK + STOP */
static uint8_t tea5767_i2c_read_byte(uint8_t send_ack)
{
    uint8_t i, byte = 0;
    tea5767_sda_high();
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        tea5767_scl_high();
        tea5767_delay();
        if (tea5767_sda_read()) byte |= 0x01;
        tea5767_scl_low();
        tea5767_delay();
    }
    if (send_ack) tea5767_sda_low();
    else          tea5767_sda_high();
    tea5767_delay();
    tea5767_scl_high();
    tea5767_delay();
    tea5767_scl_low();
    tea5767_sda_high();
    return byte;
}

/* ============================================================
 *  I2C 写 5 字节
 * ============================================================ */
void TEA5767_Write(void)
{
    tea5767_i2c_start();
    uint8_t ack = tea5767_i2c_write_byte((uint8_t)TEA5767_I2C_ADDRESS | 0x00);
    if (ack != 0) {
        printf("TEA5767: write NACK (addr)\r\n");
        tea5767_i2c_stop();
        return;
    }
    for (int i = 0; i < 5; i++) {
        ack = tea5767_i2c_write_byte(s_radioWriteData[i]);
        if (ack != 0) {
            printf("TEA5767: write NACK (byte%d=0x%02X)\r\n", i, s_radioWriteData[i]);
            tea5767_i2c_stop();
            return;
        }
    }
    tea5767_i2c_stop();
}

/* ============================================================
 *  I2C 读 5 字节
 * ============================================================ */
int TEA5767_Read(void)
{
    tea5767_i2c_start();
    uint8_t ack = tea5767_i2c_write_byte((uint8_t)TEA5767_I2C_ADDRESS | 0x01);
    if (ack != 0) {
        printf("TEA5767: read NACK (addr)\r\n");
        tea5767_i2c_stop();
        for (int i = 0; i < 5; i++) s_radioReadData[i] = 0;
        return -1;
    }
    for (int i = 0; i < 5; i++) {
        s_radioReadData[i] = tea5767_i2c_read_byte(i == 4 ? 0 : 1);
    }
    tea5767_i2c_stop();

    s_pll = (uint32_t)(s_radioReadData[0] & 0x3F) * 256u + (uint32_t)s_radioReadData[1];
    return 0;
}

static void TEA5767_ReadRetry(void)
{
    HAL_Delay(30);
    if (TEA5767_Read() != 0) {
        HAL_Delay(20);
        TEA5767_Read();
    }
}

/* ============================================================
 *  PLL 计算 (与参考 driver 一致)
 * ============================================================ */
void TEA5767_GetPLL(void)
{
    uint8_t hlsi = s_radioWriteData[2] & 0x10;
    if (hlsi) {
        s_pll = (uint32_t)((float)((g_frequency + 225) * 4) / (float)32.768);
    } else {
        s_pll = (uint32_t)((float)((g_frequency - 225) * 4) / (float)32.768);
    }
}

/* ============================================================
 *  Init (CubeMX 已经把 GPIO 配成开漏, 这里只发一帧默认频点)
 * ============================================================ */
void TEA5767_Init(void)
{
    g_frequency = TEA5767_DEFAULT_KHZ;
    TEA5767_GetPLL();
    s_radioWriteData[0] = (uint8_t)(s_pll / 256u);
    s_radioWriteData[1] = (uint8_t)(s_pll % 256u);
    s_radioWriteData[2] = 0x20;
    s_radioWriteData[3] = 0x11;
    s_radioWriteData[4] = 0x00;
    TEA5767_Write();
    HAL_Delay(30);
    (void)TEA5767_Read();

    printf("[TEA5767] Init OK, default f=%u kHz (PLL=0x%04X)\r\n",
           (unsigned)g_frequency, (unsigned)s_pll);
}

/* ============================================================
 *  设频 (kHz)
 * ============================================================ */
void TEA5767_SetFrequency(uint32_t frequency_khz)
{
    if (frequency_khz < TEA5767_MIN_KHZ) frequency_khz = TEA5767_MIN_KHZ;
    if (frequency_khz > TEA5767_MAX_KHZ) frequency_khz = TEA5767_MAX_KHZ;

    g_frequency = frequency_khz;
    TEA5767_GetPLL();
    s_radioWriteData[0] = (uint8_t)(s_pll / 256u);
    s_radioWriteData[1] = (uint8_t)(s_pll % 256u);
    s_radioWriteData[2] = 0x20;
    s_radioWriteData[3] = 0x11;
    s_radioWriteData[4] = 0x00;
    TEA5767_Write();
}

/* ============================================================
 *  取频 (kHz)
 * ============================================================ */
uint32_t TEA5767_GetFrequency(void)
{
    uint8_t hlsi = s_radioWriteData[2] & 0x10;
    if (hlsi) {
        g_frequency = (uint32_t)((float)s_pll * 8.192f - 225.0f);
    } else {
        g_frequency = (uint32_t)((float)s_pll * 8.192f + 225.0f);
    }
    return g_frequency;
}

/* ============================================================
 *  手动步进 100 kHz
 * ============================================================ */
void TEA5767_Search(uint8_t mode)
{
    TEA5767_ReadRetry();
    if (mode == TEA5767_SEARCH_UP) {
        g_frequency += 100u;
        if (g_frequency > TEA5767_MAX_KHZ) g_frequency = TEA5767_MIN_KHZ;
    } else {
        g_frequency -= 100u;
        if (g_frequency < TEA5767_MIN_KHZ) g_frequency = TEA5767_MAX_KHZ;
    }
    TEA5767_GetPLL();
    s_radioWriteData[0] = (uint8_t)(s_pll / 256u);
    s_radioWriteData[1] = (uint8_t)(s_pll % 256u);
    s_radioWriteData[2] = 0x20;
    s_radioWriteData[3] = 0x11;
    s_radioWriteData[4] = 0x00;
    TEA5767_Write();
    TEA5767_ReadRetry();
    if (s_radioReadData[0] & 0x80) {
        printf("[TEA5767] search -> freq=%u kHz LOCKED\r\n", (unsigned)g_frequency);
    } else {
        printf("[TEA5767] search -> freq=%u kHz (no RF)\r\n", (unsigned)g_frequency);
    }
}

/* ============================================================
 *  自动搜台 (步进 100 kHz, RF=1 且 0x31<IF<0x3E 锁定)
 * ============================================================ */
void TEA5767_AutoSearch(uint8_t mode)
{
    uint8_t  radioRf  = 0;
    uint8_t  radioIf  = 0;
    uint8_t  radioLev = 0;
    uint32_t steps    = 0;
    const uint32_t maxSteps = (TEA5767_MAX_KHZ - TEA5767_MIN_KHZ) / 100u + 2u;

    while (((radioRf == 0) || (radioIf <= 0x31) || (radioIf >= 0x3E)) && (steps < maxSteps)) {
        steps++;
        if (mode == TEA5767_SEARCH_UP) {
            s_radioWriteData[2] = 0xC0;
            g_frequency += 200u;
            if (g_frequency > TEA5767_MAX_KHZ) g_frequency = TEA5767_MIN_KHZ;
        } else {
            s_radioWriteData[2] = 0x40;
            g_frequency -= 200u;
            if (g_frequency < TEA5767_MIN_KHZ) g_frequency = TEA5767_MAX_KHZ;
        }
        TEA5767_GetPLL();
        s_radioWriteData[0] = (uint8_t)(s_pll / 256u) | 0xC0;
        s_radioWriteData[1] = (uint8_t)(s_pll % 256u);
        s_radioWriteData[3] = 0x11;
        s_radioWriteData[4] = 0x00;
        TEA5767_Write();
        HAL_Delay(30);
        if (TEA5767_Read() != 0) {
            printf("[TEA5767] Read failed during AutoSearch\r\n");
            continue;
        }
        radioRf  = s_radioReadData[0] & 0x80;
        radioIf  = s_radioReadData[2] & 0x7F;
        radioLev = s_radioReadData[3] >> 4;
        printf("[TEA5767] RF=%d IF=0x%02X LEV=%d f=%u kHz\r\n",
               radioRf ? 1 : 0, radioIf, radioLev, (unsigned)g_frequency);
        if ((radioRf == 0) && (radioIf == 0)) {
            printf("         ST=%02X %02X %02X %02X %02X\r\n",
                   s_radioReadData[0], s_radioReadData[1],
                   s_radioReadData[2], s_radioReadData[3], s_radioReadData[4]);
        }
    }

    if (radioRf == 0) {
        s_radioWriteData[0] &= 0x3F;
        s_radioWriteData[3]  = 0x11;
        s_radioWriteData[4]  = 0x00;
        TEA5767_Write();
        printf("[TEA5767] AutoSearch: no station (check antenna/signal).\r\n");
        return;
    }
    TEA5767_GetPLL();
    s_radioWriteData[0] = (uint8_t)(s_pll / 256u);
    s_radioWriteData[1] = (uint8_t)(s_pll % 256u);
    s_radioWriteData[3] = 0x11;
    s_radioWriteData[4] = 0x00;
    TEA5767_Write();
    TEA5767_Read();
    printf("[TEA5767] AutoSearch OK -> freq=%u kHz\r\n", (unsigned)g_frequency);
}

/* ============================================================
 *  静音
 * ============================================================ */
void TEA5767_Mute(uint8_t mode)
{
    if (mode == TEA5767_MUTE_ON) s_radioWriteData[0] |= 0x80;
    else                         s_radioWriteData[0] &= 0x7F;
    TEA5767_Write();
}

/* ============================================================
 *  音频模式
 * ============================================================ */
void TEA5767_SetAudioMode(uint8_t mode)
{
    if (mode == TEA5767_AUDIO_MONO) {
        s_radioWriteData[3] |= 0x08;
        printf("[TEA5767] Audio mode: MONO\r\n");
    } else {
        s_radioWriteData[3] &= 0xF7;
        printf("[TEA5767] Audio mode: STEREO\r\n");
    }
    TEA5767_Write();
}

/* ============================================================
 *  状态快照
 * ============================================================ */
void TEA5767_GetStatus(TEA5767_Status_t *out)
{
    if (!out) return;
    out->ready         = (TEA5767_Read() == 0) ? 1u : 0u;
    out->frequency_khz = TEA5767_GetFrequency();
    out->rf            = (s_radioReadData[0] & 0x80u) ? 1u : 0u;
    out->stereo        = (s_radioReadData[2] & 0x80u) ? 1u : 0u;
    out->if_counter    = (uint8_t)(s_radioReadData[2] & 0x7Fu);
    out->level_adc     = (uint8_t)((s_radioReadData[3] >> 4) & 0x0Fu);
}

uint8_t TEA5767_IsLocked(void)    { return (s_radioReadData[0] & 0x80u) ? 1u : 0u; }
uint8_t TEA5767_IsStereo(void)    { return (s_radioReadData[2] & 0x80u) ? 1u : 0u; }
uint8_t TEA5767_GetRSSI(void)     { return (uint8_t)((s_radioReadData[3] >> 4) & 0x0Fu); }
uint8_t TEA5767_GetIFCounter(void) { return (uint8_t)(s_radioReadData[2] & 0x7Fu); }

/****************************************************END OF FILE****************************************************/
