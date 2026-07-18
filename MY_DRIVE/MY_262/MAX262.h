#ifndef _MAX262_H_
#define _MAX262_H_
#include "stm32h7xx_hal.h"

/* MAX262 GPIO 引脚定义（STM32H743） */
/* 地址线 A0-A3 */
#define A0_Pin        GPIO_PIN_5
#define A0_GPIO_Port  GPIOE
#define A1_Pin        GPIO_PIN_6
#define A1_GPIO_Port  GPIOE
#define A2_Pin        GPIO_PIN_8
#define A2_GPIO_Port  GPIOI
#define A3_Pin        GPIO_PIN_13
#define A3_GPIO_Port  GPIOC
/* 数据线 D0-D1 */
#define D0_Pin        GPIO_PIN_11
#define D0_GPIO_Port  GPIOI
#define D1_Pin        GPIO_PIN_9
#define D1_GPIO_Port  GPIOI
/* 写使能 LE/WR */
#define LE_Pin        GPIO_PIN_10
#define LE_GPIO_Port  GPIOI
#define WR_Pin        GPIO_PIN_10
#define WR_GPIO_Port  GPIOI
/* 时钟输入 CLK */
#define CLK_Pin       GPIO_PIN_4
#define CLK_GPIO_Port GPIOB

#define setWr      HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET)
#define resetWr    HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_RESET)

#ifndef PI
#define PI 3.141592653
#endif
 
extern enum {MODE_1=0,MODE_2,MODE_3,MODE_4} workMode;
extern enum {CH_A=0,CH_B} channel; 


void MAX262_Init(void);//IO�ڳ�ʼ��
void MAX262_Write(uint8_t add,uint8_t dat2bit); //д�����ص����ݵ�ĳ��ַ 
void Set_Af(uint8_t datF); //����Aͨ��Fֵ      
void Set_AQ(uint8_t datQ); //����Aͨ��Qֵ   
void Set_Bf(uint8_t datF);  // ����Bͨ��Fֵ  
void Set_BQ(uint8_t datQ); // ����Bͨ��Qֵ 
  

float lhp_WorkFclk(float f0,float Q,uint8_t workMode,uint8_t channel);//���õ�ͨ�͸�ͨ�˲���������ȡ��ʱ��Ƶ��fclk ���л�ģ���ϵ�����ñ���ɸ��ĵ�ͨ���ͨ
float bp_WorkFclk(float fh,float fl,uint8_t workMode,uint8_t channel); //���ô�ͨ�˲���������ȡ��ʱ��Ƶ��fclk 
float bs_WorkFclk(float f0,float Q,uint8_t workMode,uint8_t channel);   //�����ݲ��˲���������ȡ��ʱ��Ƶ��fclk 
float ap_WorkFclk(float f0,float Q,uint8_t channel);//����ȫͨ�˲���������ȡ��ʱ��Ƶ��fclk 


#endif



