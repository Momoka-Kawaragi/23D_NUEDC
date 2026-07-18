/**********************************************************
                       康威电子
										 
功能：stm32f103rbt6控制，MCP41xx写入值(0-255  ->  0-R)
			参数输入0-255，具体电阻值可根据R总计算
			显示：12864cog
接口：控制接口请参照MCP41xx.h  按键接口请参照key.h
时间：2015/11/10
版本：1.0
作者：康威电子
其他：

更多电子需求，请到淘宝店，康威电子竭诚为您服务 ^_^
店铺：kvdz.taobao.com

**********************************************************/

#include "MCP41xx.h"

void mcp_delay(uint n)
{
	n=n*110;
	while(n--);
}

void MCP410XXInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	// Enable GPIOC clock - HAL uses __HAL_RCC_GPIOC_CLK_ENABLE() macro
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	// Configure all pins as GPIO output
	GPIO_InitStructure.Pin = (MCP41xx_CS1_Pin | MCP41xx_CS2_Pin | MCP41xx_DAT_Pin | MCP41xx_CLK_Pin);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MCP41xx_CS1_GPIO, &GPIO_InitStructure);
	
	// Set all pins high initially
	HAL_GPIO_WritePin(MCP41xx_CS1_GPIO, MCP41xx_CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCP41xx_CS2_GPIO, MCP41xx_CS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCP41xx_DAT_GPIO, MCP41xx_DAT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCP41xx_CLK_GPIO, MCP41xx_CLK_Pin, GPIO_PIN_SET);
}

void MCP41xx_1writedata(uchar dat1)		//调整数字电位器1
{
	uchar i,command=0x11;
	MCP41xx_SPI_CS2_H();//cs=1; 
	MCP41xx_SPI_CS1_H();//cs=1; 
	MCP41xx_SPI_CLK_L();//sck=1;
	MCP41xx_SPI_CS1_L();//cs=0;

	mcp_delay(10);

	for(i=0;i<8;i++)          //写命令
	{ 
		if(command & 0x80)
		{
			MCP41xx_SPI_DAT_H();//si=1;	
		}
		else
		{
			MCP41xx_SPI_DAT_L();//si=0;
		}
		mcp_delay(10);
		MCP41xx_SPI_CLK_L();//sck=1;//sck=1;
		mcp_delay(10);
		MCP41xx_SPI_CLK_H();//sck=0;sck=0;
		mcp_delay(10); 

		command=command<<1;
	}

     
	for(i=0;i<8;i++)          //写数据
	{ 
		
		if(dat1 & 0x80)
		{
			MCP41xx_SPI_DAT_H();//si=1;	
		}
		else
		{
			MCP41xx_SPI_DAT_L();//si=0;
		}
		mcp_delay(10);
		MCP41xx_SPI_CLK_L();//sck=1;//sck=1;
		mcp_delay(10);
		MCP41xx_SPI_CLK_H();//sck=0;sck=0;
		mcp_delay(10);
		dat1=dat1<<1;
	}

	MCP41xx_SPI_CS1_H();//cs=1; //cs=1;

	mcp_delay(10);//_nop_();
}

void MCP41xx_2writedata(uchar dat2)		//调整数字电位器2
{
	uchar i,command=0x11;
	
	MCP41xx_SPI_CS1_H();//cs=1; 
	MCP41xx_SPI_CS2_H();//cs=1; 
	MCP41xx_SPI_CLK_L();//sck=1;
	MCP41xx_SPI_CS2_L();//cs=0;

	mcp_delay(10);

	for(i=0;i<8;i++)          //写命令
	{ 
		if(command & 0x80)
		{
			MCP41xx_SPI_DAT_H();//si=1;	
		}
		else
		{
			MCP41xx_SPI_DAT_L();//si=0;
		}
		mcp_delay(10);
		MCP41xx_SPI_CLK_L();//sck=1;//sck=1;
		mcp_delay(10);
		MCP41xx_SPI_CLK_H();//sck=0;sck=0;
		mcp_delay(10); 

		command=command<<1;
	}

     
	for(i=0;i<8;i++)          //写数据
	{ 
		
		if(dat2 & 0x80)
		{
			MCP41xx_SPI_DAT_H();//si=1;	
		}
		else
		{
			MCP41xx_SPI_DAT_L();//si=0;
		}
		mcp_delay(10);
		MCP41xx_SPI_CLK_L();//sck=1;//sck=1;
		mcp_delay(10);
		MCP41xx_SPI_CLK_H();//sck=0;sck=0;
		mcp_delay(10);
		dat2=dat2<<1;
	}

	MCP41xx_SPI_CS2_H();//cs=1; //cs=1;

	mcp_delay(10);//_nop_();
}

// ============================================================
// 程控放大器封装函数
// 功能: 根据输入的放大倍数设置数字电位器
// 参数: gain - 放大倍数(0.0表示静音, 1.0表示原幅值, 2.0表示2倍)
// 说明: 总增益 = (1 + Rf/R1) * dat/255
//       当Rf=R1=1k时, 最大增益 = 2倍
// ============================================================

// 可根据实际硬件修改这两个参数
#define AMPLIFIER_Rf  1000.0f   // 反馈电阻,单位:欧姆
#define AMPLIFIER_R1  1000.0f   // 输入电阻,单位:欧姆

void SetAmplifierGain(float gain)
{
    // 计算基础增益(由电阻比例决定)
    float base_gain = 1.0f + (AMPLIFIER_Rf / AMPLIFIER_R1);
    
    // 计算最大可能增益(dat=255时)
    float max_gain = base_gain;
    
	// 限制增益范围: 0.0到最大增益之间
	if(gain < 0.0f)
    {
		gain = 0.0f;
    }
    else if(gain > max_gain)
    {
        gain = max_gain;
    }
    
    // 反推需要的dat值: gain = base_gain * dat/255
    // => dat = gain * 255 / base_gain
    float dat_float = gain * 255.0f / base_gain;
    uchar dat = (uchar)(dat_float + 0.5f);  // 四舍五入
    
    // 设置数字电位器
    MCP41xx_1writedata(dat);
}
