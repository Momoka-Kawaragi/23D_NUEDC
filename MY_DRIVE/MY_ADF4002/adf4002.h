#ifndef _adf4002_h_
#define _adf4002_h_

#include "main.h"

// 硬件引脚配置区 - 方便移植修改
// Hardware Pin Configuration
#define ADF_GPIO_PORT       GPIOD
#define ADF_CLK_ENABLE()    __HAL_RCC_GPIOD_CLK_ENABLE()

#define ADF_SCK_PIN         GPIO_PIN_2
#define ADF_SDI_PIN         GPIO_PIN_1
#define ADF_LE_PIN          GPIO_PIN_0

// 引脚操作宏
// Pin Operation Macros
#define PLL_SCK_0 HAL_GPIO_WritePin(ADF_GPIO_PORT, ADF_SCK_PIN, GPIO_PIN_RESET)
#define PLL_SCK_1 HAL_GPIO_WritePin(ADF_GPIO_PORT, ADF_SCK_PIN, GPIO_PIN_SET)

#define PLL_SDI_0 HAL_GPIO_WritePin(ADF_GPIO_PORT, ADF_SDI_PIN, GPIO_PIN_RESET)
#define PLL_SDI_1 HAL_GPIO_WritePin(ADF_GPIO_PORT, ADF_SDI_PIN, GPIO_PIN_SET)

#define PLL_SEN_0 HAL_GPIO_WritePin(ADF_GPIO_PORT, ADF_LE_PIN, GPIO_PIN_RESET)
#define PLL_SEN_1 HAL_GPIO_WritePin(ADF_GPIO_PORT, ADF_LE_PIN, GPIO_PIN_SET)


#define R_Address 0X000000
#define N_Address 0X000001
#define F_Address 0X000002
#define I_Address 0X000003
#define Pre_R 0X000000          //X000,0000,DAT(14),00
#define Pre_N 0X000000          //XX0,DAT(13),XXXXXX,01

                                                                                                                                                                                                                                                                                                                        

void InitADF4002(void);
void Delay(unsigned int z);
void DelayMs(void);
void SendDataPll(unsigned long int Data);

void RDivideTest(uint16_t RData);
void NDivideTest(uint16_t NData);

extern long int Reg0x02_LEDON;
extern long int Reg0x02_LEDOFF;

#endif 
