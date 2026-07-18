#ifndef __AD9959_H
#define __AD9959_H

#include "main.h"

/**
 * @brief AD9959 寄存器地址定义
 * @note  地址来自 AD9959 数据手册，写入长度以字节为单位
 */
#define CSR_ADD 0x00   // CSR：通道选择寄存器                         1 字节
#define FR1_ADD 0x01   // FR1：功能寄存器1                            3 字节
#define FR2_ADD 0x02   // FR2：功能寄存器2                            2 字节
#define CFR_ADD 0x03   // CFR：通道功能寄存器                         3 字节
#define CFTW0_ADD 0x04 // CFTW0：通道频率调谐字寄存器0                 4 字节
#define CPOW0_ADD 0x05 // CPOW0：通道相位偏移字寄存器0                 2 字节
#define ACR_ADD 0x06   // ACR：幅度控制寄存器                          3 字节
#define LSRR_ADD 0x07  // LSRR：线性扫频斜率寄存器                     2 字节
#define RDW_ADD 0x08   // RDW：线性向上扫频步进字                      4 字节
#define FDW_ADD 0x09   // FDW：线性向下扫频步进字                      4 字节
#define CW1 0x0A       // CW1：通道字寄存器1                           4 字节
#define CW2 0x0B       // CW2：通道字寄存器2                           4 字节
#define CW3 0x0C       // CW3：通道字寄存器3                           4 字节
#define CW4 0x0D       // CW4：通道字寄存器4                           4 字节

typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint32_t Pin;
} driverIO;

extern driverIO SDIO0;
extern driverIO SDIO1;
extern driverIO SDIO2;
extern driverIO SDIO3;
extern driverIO PDC;
extern driverIO RST;
extern driverIO SCLK;
extern driverIO CS;
extern driverIO UPDATE;
extern driverIO PS0;
extern driverIO PS1;
extern driverIO PS2;
extern driverIO PS3;

/**
 * @brief AD9959 IO 操作宏
 * @note  IO 必须为 driverIO 结构体；VAL 取 0/1
 */
#define WRT(IO, VAL) HAL_GPIO_WritePin(IO.GPIOx, IO.Pin, (VAL) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define GET(IO) HAL_GPIO_ReadPin(IO.GPIOx, IO.Pin)

/**
 * @brief 初始化 AD9959（GPIO 已初始化的前提下调用）
 */
void Init_AD9959(void);

/**
 * @brief AD9959 软件延时（空循环）
 * @param length 延时循环次数（与主频有关，仅用于满足时序）
 */
void delay_9959(uint32_t length);

/**
 * @brief 初始化 AD9959 相关 IO 引脚默认电平
 */
void InitIO_9959(void);

/**
 * @brief 复位 AD9959（产生 RST 脉冲）
 */
void InitReset(void);

/**
 * @brief 产生 IO UPDATE 脉冲，使寄存器新配置生效
 */
void AD9959_IO_Update(void);

/**
 * @brief 向 AD9959 写寄存器（仅写入，不自动 UPDATE）
 * @param RegisterAddress 寄存器地址
 * @param NumberofRegisters 写入字节数
 * @param RegisterData 待写入数据（高字节在前）
 */
void WriteData_AD9959(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData);

/**
 * @brief 从 AD9959 读寄存器
 * @param RegisterAddress 寄存器地址
 * @param NumberofRegisters 读取字节数
 * @param RegisterData 读出数据缓冲区（高字节在前）
 */
void ReadData_AD9959(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData);

/**
 * @brief 设置指定通道频率（写入 CFTW0，不自动 UPDATE）
 * @param Channel 通道号（0~3）
 * @param Freq 频率（Hz，1~500000000）
 */
void Write_Frequence(uint8_t Channel, uint32_t Freq);

/**
 * @brief 设置指定通道幅度（写入 ACR，不自动 UPDATE）
 * @param Channel 通道号（0~3）
 * @param Ampli 幅度（0~1023）
 */
void Write_Amplitude(uint8_t Channel, uint16_t Ampli);

/**
 * @brief 设置指定通道相位（写入 CPOW0，不自动 UPDATE）
 * @param Channel 通道号（0~3）
 * @param Phase 相位（度，0~359）
 */
void Write_Phase(uint8_t Channel, uint16_t Phase);

/**
 * @brief 一键配置通道：频率/幅度/相位（不自动 UPDATE）
 * @note  调用后需再执行 AD9959_IO_Update() 才会生效
 * @param Channel 通道号（0~3）
 * @param Ampli 幅度（0~1023）
 * @param Phase 相位（度，0~359）
 * @param Freq 频率（Hz）
 */
void AD9959_SetChannel(uint8_t Channel, uint16_t Ampli, uint16_t Phase, uint32_t Freq);

/**
 * @brief 读取当前通道频率（读取 CFTW0 并换算为 Hz）
 * @return 频率（Hz）
 */
uint32_t Get_Freq(void);

/**
 * @brief 读取当前通道幅度（读取 ACR）
 * @return 幅度（0~1023）
 */
uint8_t Get_Amp(void);

/**
 * @brief 线性扫频（使用 AD9959 内部扫频功能）
 * @param Channel 通道号（0~3）
 * @param Start_Freq 起始频率（Hz）
 * @param Stop_Freq 终止频率（Hz）
 * @param Step 步进频率（Hz）
 * @param time 每步持续时间（微秒，1~2048）
 * @param NO_DWELL 是否开启 no-dwell（1 开启 / 0 关闭）
 */
void Sweep_Frequency(uint8_t Channel, uint32_t Start_Freq, uint32_t Stop_Freq, uint32_t Step, uint32_t time, uint8_t NO_DWELL);

/**
 * @brief 选择通道（写 CSR）
 * @param Channel 通道号（0~3）
 */
void Channel_Select(uint8_t Channel);

/**
 * @brief 停止 AD9959 输出（进入掉电/停止模式）
 */
void Stop_AD9959(void);

/**
 * @brief 错误处理回调（可按项目需要实现）
 */
void AD9959_error(void);

/**
 * @brief 2FSK 配置
 * @param Channel 通道号（0~3）
 * @param f_start Profile=0 时频率（Hz）
 * @param f_stop Profile=1 时频率（Hz）
 */
void SET_2FSK(uint8_t Channel, double f_start, double f_stop);

/**
 * @brief 2ASK 配置
 * @param Channel 通道号（0~3）
 * @param f 载波频率（Hz）
 * @param A_start Profile=0 幅度（0~1023）
 * @param A_stop Profile=1 幅度（0~1023）
 */
void SET_2ASK(uint8_t Channel, double f, uint16_t A_start, uint16_t A_stop);

/**
 * @brief 频率换算为调谐字（CFTW0）
 * @param f 频率（Hz）
 * @param fWord 4字节调谐字输出（高字节在前）
 */
void Freq2Word(double f, uint8_t *fWord);

/**
 * @brief 幅度换算为 ACR 写入字
 * @param A 幅度（0~1023）
 * @param AWord 3字节输出（高字节在前）
 */
void Amp2Word(uint16_t A, uint8_t *AWord);

/**
 * @brief 相位换算为 CPOW0 写入字
 * @param Phase 相位（度，0~359）
 * @param PWord 2字节输出（高字节在前）
 */
void Phase2Word(uint16_t Phase, uint8_t *PWord);
#endif
