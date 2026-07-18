#include "adf4002.h"
#include "main.h"

long int ReadData;

//long int Reg0x00 = 0x001F40;//r
//////long int Reg0x01 = 0x0DAC01;//n

long int functionReg0x02R = 0x0D80C2; //功能寄存器: 设置 MUXOUT 输出为 R 分频输出 (R DIVIDER OUTPUT)
long int initReg0x03R = 0x0D80C3;     //初始化寄存器: 设置 MUXOUT 输出为 R 分频输出 (R DIVIDER OUTPUT)

long int functionReg0x02N = 0x0D80A2; //功能寄存器: 设置 MUXOUT 输出为 N 分频输出 (N DIVIDER OUTPUT)
long int initReg0x03N = 0x0D80A3;     //初始化寄存器: 设置 MUXOUT 输出为 N 分频输出 (N DIVIDER OUTPUT)

long int Reg0x02_LEDON  = 0x0D80B2; //MUXOUT 控制: 输出高电平 (DVDD)
long int Reg0x02_LEDOFF = 0x0D80F2; //MUXOUT 控制: 输出低电平 (DGND)


void Delay(unsigned int z)
{
  unsigned int i,j;
  
  for(i = z; i > 0; i--)
    for(j = 10; j > 0; j--) ;
}

void DelayMs(void)
{
  unsigned int i, j;
  
  for(i = 0; i < 1000; i++)
  {
    for(j = 0; j < 1000; j++)
    {
      Delay(1000);
    }
  }
}
//ADF4002 IO初始化
void InitADF4002(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure = {0};

    // 启用 GPIO 时钟 (使用宏定义，方便移植)
	ADF_CLK_ENABLE();

    // 配置 GPIO 引脚 (使用宏定义，方便移植)
	GPIO_InitStructure.Pin = ADF_SCK_PIN | ADF_SDI_PIN | ADF_LE_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(ADF_GPIO_PORT, &GPIO_InitStructure);

	PLL_SCK_0;
	PLL_SDI_0;
	PLL_SEN_0;   
	//SendDataPll(initReg0x03R); //INIT	
    //SendDataPll(functionReg0x02R); //funtion
	SendDataPll(Reg0x02_LEDOFF); //INIT	
	//RDivideTest(1);

}

void SendDataPll(unsigned long int Data)          //向 PLL 发送数据
{
  unsigned char i;

	PLL_SCK_0;
  PLL_SEN_0;  
  
  for(i = 0; i < 24; i++)
  {
    if(Data & 0x800000)
    {
      PLL_SDI_1;
    }
    else
    {
       PLL_SDI_0;
    }
    Data <<= 1;     
    PLL_SCK_1;
    
    Delay(100);
    
    PLL_SCK_0;
    Delay(100);
  }
  PLL_SDI_0;
  
  PLL_SEN_0;
  Delay(100);
  PLL_SEN_1;
}

void RDivideTest(uint16_t RData)
{
	uint32_t S_R = 0;
	
  S_R = Pre_R + (RData<<2) + R_Address;
  SendDataPll(functionReg0x02R); //配置为 R 输出
  SendDataPll(S_R);
	SendDataPll(0X000001);	
}
void NDivideTest(uint16_t NData)
{
	uint32_t S_N = 0;
	
    // 软件补偿：实际输出频率是理论值的8倍，说明分频系数偏小8倍。
    // 因此将设定值乘以8。
    // 注意：这将导致最大可用分频系数变为 8191 (原本是 65535)
    uint32_t CompensatedN = (uint32_t)NData * 8;
	
	uint16_t B_Val = CompensatedN / 8; // P = 8/9
	uint16_t A_Val = CompensatedN % 8;
	
  // S_N = Pre_N + (NData<<8) + N_Address; // 原错误代码：直接把 NData 放到了 B 计数器位置
  
  // 修正后的代码：正确拼接 B 计数器 (Bit 20-8) 和 A 计数器 (Bit 7-2)
  // 寄存器格式 (假设 Pre_N 包含了控制位但这里 Pre_N 为 0):
  // N Register: [Reserved][B Counter (13 bits)][A Counter (6 bits)][Control Bits (2 bits)]
  // 但 ADF4002 具体格式：
  // DB23-DB21: Reserved
  // DB20-DB8:  13-bit B Counter
  // DB7-DB2:   6-bit A Counter
  // DB1-DB0:   Control Bits (01 for N Counter Latch)
  
  // 注意：Pre_N 定义为 0x000000，可能需要根据具体配置调整，这里先按标准拼接
  S_N = (unsigned long int)(B_Val) << 8; // B 计数器移位到 DB8
  S_N |= (unsigned long int)(A_Val) << 2; // A 计数器移位到 DB2
  S_N |= N_Address; //加上地址位 (01)
  
  SendDataPll(functionReg0x02N); //配置为 N 输出
	SendDataPll(0X000000);
  SendDataPll(S_N);

}
