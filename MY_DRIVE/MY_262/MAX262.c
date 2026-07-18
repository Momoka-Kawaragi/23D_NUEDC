/**********************************************************
                       康威电子
功能：stm32f103rbt6控制MAX262滤波器
接口：控制引脚接口参见max262.h
时间：2023/06/XX
版本：2.0
作者：康威电子
其他：文件仅供学习使用，请勿商业用途

                    max262    单片机(H743)
硬件连接:	A0 连接至 PE5;
            A1 连接至 PE6;
            A2 连接至 PI8;
            A3 连接至 PC13;
            D0 连接至 PI11;
            D1 连接至 PI9;
            WR=LE 连接至 PI10;
            CLK 连接至 PB4
          GND--GND(0V)

更多电子需求，请到淘宝店，康威电子竭诚为您服务 ^_^
https://kvdz.taobao.com/
**********************************************************/

#include "max262.h"
#include "delay.h"
#include <math.h>

/***************************************************************
MAX262 IO口初始化 (H743迁移：添加GPIO自初始化)
****************************************************************/
void MAX262_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PE5(A0), PE6(A1) */
    GPIO_InitStruct.Pin   = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* PI8(A2), PI9(D1), PI10(LE), PI11(D0) */
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* PC13(A3) */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PB4(CLK) */
    GPIO_InitStruct.Pin   = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始电平 */
    HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_SET);
    setWr;
}

/***************************************************************
* 名    称：MAX262_Write(uint8_t add,uint8_t dat2bit)
* 功    能：写2bit数据到某地址
* 入口参数：A3~A0的地址add,
            D1~D0的2bit数据dat2bit
* 出口参数：无
* 说    明：D1~D0取dat2bit的低两位，参照PDL口的按位写操作
***************************************************************/
void MAX262_Write(uint8_t add,uint8_t dat2bit)
{
    HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, (add & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, (add & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, (add & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, (add & 0x08U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, (dat2bit & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, (dat2bit & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    resetWr;
    HAL_Delay(1);
    setWr;
    HAL_Delay(1);
}

/***************************************************************
* 名    称：Set_Af函数
* 功    能：设置A通道F值  fCLK/f0
* 入口参数：A通道F值datF 0-63
* 出口参数：无
* 说    明：无
***************************************************************/
void Set_Af(uint8_t datF)    //6位中心频率f0控制位F0-F5 对应十进制0-63
{
    MAX262_Write(1,datF);
    datF = datF>>2;
    MAX262_Write(2,datF);
    datF = datF>>2;
    MAX262_Write(3,datF);
}

/***************************************************************
* 名    称：Set_AQ函数
* 功    能：设置A通道Q值
* 入口参数：A通道Q值datQ 1-127
* 出口参数：无
* 说    明：无
***************************************************************/
void Set_AQ(uint8_t datQ)    //7位品质因数Q控制位Q0-Q6 对应十进制0-127
{
    MAX262_Write(4,datQ);
    datQ = datQ>>2;
    MAX262_Write(5,datQ);
    datQ = datQ>>2;
    MAX262_Write(6,datQ);
    datQ = (datQ>>2)&1;  //对地址7只取一位
    MAX262_Write(7,datQ);
}

/***************************************************************
* 名    称：Set_Bf函数
* 功    能：设置B通道F值 fCLK/f0
* 入口参数：B通道F值datF 0-63
* 出口参数：无
* 说    明：无
***************************************************************/
void Set_Bf(uint8_t datF)
{
    MAX262_Write(9,datF);
    datF = datF>>2;
    MAX262_Write(10,datF);
    datF = datF>>2;
    MAX262_Write(11,datF);
}

/***************************************************************
* 名    称：Set_BQ函数
* 功    能：设置B通道Q值
* 入口参数：B通道Q值datQ 0-127
* 出口参数：无
* 说    明：无
***************************************************************/
void Set_BQ(uint8_t datQ)
{
    MAX262_Write(12,datQ);
    datQ = datQ>>2;
    MAX262_Write(13,datQ);
    datQ = datQ>>2;
    MAX262_Write(14,datQ);
    datQ = (datQ>>2)&1;
    MAX262_Write(15,datQ);
}

/***************************************************************
* 名    称：FnFin_config(double f0,uint8_t workMode)
* 功    能：计算fn寄存器值
* 入口参数：f0截止频率
            workMode工作模式 MODE_1~MODE_4
* 出口参数：fn寄存器值
* 说    明：确保时钟频率不超过 4MHz 时，选择合适的fn
***************************************************************/
uint8_t FnFin_config(double f0,uint8_t workMode)
{
    uint8_t i=0,fn=0;
    if(workMode==MODE_2)//模式2
    {
        for(i=0;i<64;i++)
        {
            if(((26+i)*1.11072)*f0>4000000)
            {
                if(i>0)
                    fn=i-1;//确保时钟频率不超过 4MHz 时，选择合适的fn
                break;
            }
            else fn=63;
        }
        //fclk/f0 = (26 + fn)*1.11072;
        //时钟频率/截止频率=(26 + 频率控制字)*1.11072;
        //fclk = (26 + fn)*1.11072*f0;
        //确保时钟频率不超过 4MHz 时，选择合适的fn
    }
    else//模式134
    {
        for(i=0;i<64;i++)
        {
            if(((26+i)*PI/2)*f0>4000000)
            {
                if(i>0)
                    fn=i-1;
                break;
            }
            else fn=63;
        }
        //fclk/f0 = (26 + fn)*PI/2;
        //时钟频率/截止频率=(26 + 频率控制字)*PI/2;
        //fclk = (26 + fn)*PI/2*f0;
        //确保时钟频率不超过 4MHz 时，选择合适的fn
    }
    return fn;
}

/***************************************************************
* 名    称：Qn_config(float Q,uint8_t workMode)
* 功    能：计算Qn寄存器值
* 入口参数：Q值
            workMode工作模式 MODE_1~MODE_4
* 出口参数：Qn寄存器值
* 说    明：无
***************************************************************/
uint8_t Qn_config(float Q,uint8_t workMode)
{
    uint8_t Qn=0;
    if(workMode==MODE_2)//模式2
    {
        Qn = (uint8_t)(128-90.51/Q);
    }
    else    //模式134
    {
        Qn = (uint8_t)(128-64/Q);
    }
    return Qn;
}

/***************************************************************
* 名    称：lhp_WorkFclk函数
* 功    能：设置低通和高通滤波器，并计算所需时钟频率fclk
* 入口参数：f0截止频率，切换模式可将低通改成高通
            Q品质因数
            workMode工作模式  //低通只有工作模式3
            channel通道CH_A,CH_B
* 出口参数：时钟频率fclk
* 说    明：需要时钟给到之后才能运作。Q越大频率曲线波动越大
* 使用说明：模式3时Q值要大于0.707，用于低通和高通
***************************************************************/
float lhp_WorkFclk(float f0,float Q,uint8_t workMode,uint8_t channel)
{
    float fclk ;
    uint8_t Qn, Fn;

    Qn=10;//Qn_config(Q, workMode);
    Fn = 1;//FnFin_config(f0, workMode);

    if(channel==CH_A)
    {
        MAX262_Write(0,workMode);   //
        Set_Af(Fn);  //要使用大范围可调电容的滤波器频率才去最大值
        Set_AQ(Qn);
    }
    else
    {
        MAX262_Write(8,workMode);
        Set_Bf(Fn);     //要使用大范围可调电容的滤波器频率才去最大值
        Set_BQ(Qn);
    }
    if(workMode==MODE_2)
    {
        fclk = (26+Fn)*1.11072*f0;  // 计算CLK时钟, 芯片手册p11
    }
    else
    {
        fclk = (26+Fn)*PI/2*f0;  // 计算CLK时钟, 芯片手册p11
    }
    return fclk ;
}


/***************************************************************
* 名称：bp_WorkFclk函数
* 功能：设置带通滤波器，并计算所需时钟频率fclk。模式切换可将带通改成带阻
* 入口参数：上限截止频率fh，下限截止频率fl，工作模式workMode，通道channel
* 出口参数：时钟频率fclk
* 说明：需要时钟给到之后才能运作
* 使用说明：设置带通频率时，需要两个频率
            中心频率、滤波器参数>0.707
***************************************************************/
float bp_WorkFclk(float fh,float fl,uint8_t workMode,uint8_t channel)
{
    float f0,Q,fclk;
    uint8_t Qn, Fn;

    f0 = sqrt(fh*fl);
    Q = f0/(fh-fl);   //计算q值 参考芯片手册p18

    Qn=Qn_config(Q, workMode);

    Fn = FnFin_config(f0, workMode);

    if(channel==CH_A)
    {
        MAX262_Write(0,workMode);
        Set_Af(Fn);  //要使用大范围可调电容的滤波器频率才去最大值
        Set_AQ(Qn); //
    }
    else
    {
        MAX262_Write(8,workMode);
        Set_Bf(Fn);     //要使用大范围可调电容的滤波器频率才去最大值
        Set_BQ(Qn);
    }
    if(workMode==MODE_2)
    {
        fclk = (26+Fn)*1.11072*f0;  // 计算CLK时钟, 芯片手册p11
    }
    else
    {
        fclk = (26+Fn)*PI/2*f0;  // 计算CLK时钟, 芯片手册p11
    }
    return fclk ;
}


/***************************************************************
* 名称：bs_WorkFclk函数
* 功能：设置带阻滤波器，并计算所需时钟频率fclk。模式切换可将带通改成带阻
* 入口参数：带阻频率fn，品质因数Q，工作模式workMode，通道channel
* 出口参数：时钟频率fclk
* 说明：需要时钟给到之后才能运作。Q越大频率曲线波动越大；建议带阻模式3时Q值要大于0.707
* 使用说明：只有模式1使用带阻 fn=f0; 模式2只用来去fn=0.725f0
***************************************************************/
float bs_WorkFclk(float f0,float Q,uint8_t workMode,uint8_t channel)
{
    uint8_t Qn,Fn;
    float fclk;

    Qn=Qn_config(Q, workMode);
    Fn = FnFin_config(f0, workMode);

    if(channel==CH_A)
    {
        MAX262_Write(0,workMode);
        Set_Af(Fn);  //要使用大范围可调电容的滤波器频率才去最大值
        Set_AQ(Qn);
    }
    else
    {
        MAX262_Write(8,workMode);
        Set_Bf(Fn);     //要使用大范围可调电容的滤波器频率才去最大值
        Set_BQ(Qn);
    }
    if(workMode==MODE_2)
    {
        fclk = (26+Fn)*1.11072*f0;  // 计算CLK时钟, 芯片手册p11
    }
    else
    {
        fclk = (26+Fn)*PI/2*f0;  // 计算CLK时钟, 芯片手册p11
    }
    return fclk ;
}

/***************************************************************
* 名称：ap_WorkFclk函数
* 功能：设置全通滤波器，并计算所需时钟频率fclk。模式切换可将带通改成带阻
* 入口参数：中心频率f0，品质因数Q，通道channel
* 出口参数：时钟频率fclk
* 说明：需要时钟给到之后才能运作。仅支持全通
* 使用说明：Q值不能太大，否则f0会不准
***************************************************************/
float ap_WorkFclk(float f0,float Q,uint8_t channel)
{
    uint8_t Qn,Fn;
    Qn = (uint8_t)(128-64/Q);
    Fn = FnFin_config(f0, MODE_4);

    if(channel==CH_A)
    {
        MAX262_Write(0,MODE_4);
        Set_Af(Fn);  //要使用大范围可调电容的滤波器频率才去最大值
        Set_AQ(Qn);
    }
    else
    {
        MAX262_Write(8,MODE_4);
        Set_Bf(Fn);     //要使用大范围可调电容的滤波器频率才去最大值
        Set_BQ(Qn);
    }
    return  (26+Fn)*PI/2*f0;
}
