#ifndef __MCP41xx_h
#define __MCP41xx_h
#include "stm32h7xx_hal.h"
#include "main.h"

#define uint unsigned int
#define uchar unsigned char

#define DATA_CD 0x11

// Define GPIO pins for MCP41xx control
// Using PC8, PC9, PC12, PC13
#define MCP41xx_CS1_Pin   GPIO_PIN_8
#define MCP41xx_CS1_GPIO  GPIOC
#define MCP41xx_CS2_Pin   GPIO_PIN_9
#define MCP41xx_CS2_GPIO  GPIOC
#define MCP41xx_CLK_Pin   GPIO_PIN_13
#define MCP41xx_CLK_GPIO  GPIOC
#define MCP41xx_DAT_Pin   GPIO_PIN_12
#define MCP41xx_DAT_GPIO  GPIOC

// SPI control macros using HAL
#define MCP41xx_SPI_CLK_H()   HAL_GPIO_WritePin(MCP41xx_CLK_GPIO, MCP41xx_CLK_Pin, GPIO_PIN_SET)
#define MCP41xx_SPI_CLK_L()   HAL_GPIO_WritePin(MCP41xx_CLK_GPIO, MCP41xx_CLK_Pin, GPIO_PIN_RESET)

#define MCP41xx_SPI_DAT_H()   HAL_GPIO_WritePin(MCP41xx_DAT_GPIO, MCP41xx_DAT_Pin, GPIO_PIN_SET)
#define MCP41xx_SPI_DAT_L()   HAL_GPIO_WritePin(MCP41xx_DAT_GPIO, MCP41xx_DAT_Pin, GPIO_PIN_RESET)

#define MCP41xx_SPI_CS1_H()   HAL_GPIO_WritePin(MCP41xx_CS1_GPIO, MCP41xx_CS1_Pin, GPIO_PIN_SET)
#define MCP41xx_SPI_CS1_L()   HAL_GPIO_WritePin(MCP41xx_CS1_GPIO, MCP41xx_CS1_Pin, GPIO_PIN_RESET)

#define MCP41xx_SPI_CS2_H()   HAL_GPIO_WritePin(MCP41xx_CS2_GPIO, MCP41xx_CS2_Pin, GPIO_PIN_SET)
#define MCP41xx_SPI_CS2_L()   HAL_GPIO_WritePin(MCP41xx_CS2_GPIO, MCP41xx_CS2_Pin, GPIO_PIN_RESET)

void MCP410XXInit(void);
void mcp_delay(uint n);
void MCP41xx_1writedata(uchar dat1);
void MCP41xx_2writedata(uchar dat2);

// 程控放大器封装函数
// 设置放大倍数,输入倍数值
// Rf和R1可根据硬件修改(默认Rf=1k, R1=1k, 增益范围0~2倍)
void SetAmplifierGain(float gain);

#endif
