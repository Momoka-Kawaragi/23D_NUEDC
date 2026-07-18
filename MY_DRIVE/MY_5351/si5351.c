/**************************************************************************/
/*!
    @file     si5351.c

    @author   K. Townsend (Adafruit Industries)

    @brief    SI5351 160MHz clock generator driver

    @section  REFERENCES

    Si5351A/B/C datasheet:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351.pdf

    Si5351 register map and application note:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf

    @section  LICENSE
    Original license text omitted in this port file comment.
*/
/**************************************************************************/
#include "main.h"
  // STM32 HAL dependency
#include "stm32h7xx_hal.h"
#include <math.h>
#include <si5351.h>

/**************************************************************************/
/*!
  Initialize SI5351 configuration and default device state.
*/
/**************************************************************************/
err_t si5351_Init(void)
{

  /* Initialize software configuration structure. */
	  m_si5351Config.initialised     = 0;
	  m_si5351Config.crystalFreq     = SI5351_CRYSTAL_FREQ_25MHZ;
	  m_si5351Config.crystalLoad     = SI5351_CRYSTAL_LOAD_10PF;
	  m_si5351Config.crystalPPM      = 30;
	  m_si5351Config.plla_configured = 0;
	  m_si5351Config.plla_freq       = 0;
	  m_si5351Config.pllb_configured = 0;
	  m_si5351Config.pllb_freq       = 0;
	  m_si5351Config.ms0_freq		 = 0;
	  m_si5351Config.ms1_freq		 = 0;
	  m_si5351Config.ms2_freq		 = 0;
	  m_si5351Config.ms0_r_div		 = 0;
	  m_si5351Config.ms1_r_div		 = 0;
	  m_si5351Config.ms2_r_div		 = 0;



  /* Disable all outputs by default (CLKx disabled). */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF));

  /* Power down all output drivers. */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_16_CLK0_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_17_CLK1_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_18_CLK2_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_19_CLK3_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_20_CLK4_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_21_CLK5_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_22_CLK6_CONTROL, 0x80));
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_23_CLK7_CONTROL, 0x80));

  /* Configure crystal load capacitance. */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
                       m_si5351Config.crystalLoad));

    /* Reserved/interrupt-related config can be added here if required.
      See AN619 and ClockBuilder-generated defaults if needed. */

  /* Clear cached PLL configuration status. */
  m_si5351Config.plla_configured = 0;
  m_si5351Config.plla_freq = 0;
  m_si5351Config.pllb_configured = 0;
  m_si5351Config.pllb_freq = 0;

  /* Mark driver initialized. */
  m_si5351Config.initialised = 1;

  return ERROR_NONE;
}


/**************************************************************************/
/*!
  @brief  Configure PLL with integer multiplier.

  @param  pll   Target PLL
                - SI5351_PLL_A
                - SI5351_PLL_B
  @param  mult  PLL multiplier (15 to 90)
*/
/**************************************************************************/
err_t si5351_setupPLLInt(si5351PLL_t pll, uint8_t mult)
{
  return si5351_setupPLL(pll, mult, 0, 1);
}

/**************************************************************************/
/*!
  @brief  Configure PLL frequency using fractional form.

    @param  pll   Target PLL
                  - SI5351_PLL_A
                  - SI5351_PLL_B
    @param  mult  PLL integer multiplier (15 to 90)
    @param  num   Fraction numerator (20-bit, 0..1,048,575)
    @param  denom Fraction denominator (20-bit, 1..1,048,575)

    @section PLL Configuration

    fVCO is PLL VCO frequency in the range 600..900 MHz:

        fVCO = fXTAL * (a+(b/c))

    fXTAL = crystal frequency
    a     = integer part (15..90)
    b     = numerator (0..1,048,575)
    c     = denominator (1..1,048,575)

    Prefer integer mode when possible for lower jitter
    (set b=0 and c=1).

    Reference: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
*/
/**************************************************************************/
err_t si5351_setupPLL(si5351PLL_t pll,
                                uint8_t     mult,
                                uint32_t    num,
                                uint32_t    denom)
{
  uint32_t P1;       /* PLL config register P1 */
  uint32_t P2;	     /* PLL config register P2 */
  uint32_t P3;	     /* PLL config register P3 */

  /* Parameter validation */
  ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED );
  ASSERT( (mult > 14) && (mult < 91), ERROR_INVALIDPARAMETER ); /* mult = 15..90 */
  ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* denominator > 0 */
  ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20-bit numerator */
  ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20-bit denominator */

  /* Fractional PLL register equations:
   * a=mult, b=num, c=denom
   * P1[17:0] = 128*a + floor(128*(b/c)) - 512
   * P2[19:0] = 128*b - c*floor(128*(b/c))
   * P3[19:0] = c
   */

  /* Calculate PLL configuration registers. */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* Base address: PLLA starts at 26, PLLB starts at 34. */
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);

  /* Write PLL configuration registers. */
  ASSERT_STATUS( si5351_write8( baseaddr,   (P3 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351_write8( baseaddr+1, (P3 & 0x000000FF)));
  ASSERT_STATUS( si5351_write8( baseaddr+2, (P1 & 0x00030000) >> 16));
  ASSERT_STATUS( si5351_write8( baseaddr+3, (P1 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351_write8( baseaddr+4, (P1 & 0x000000FF)));
  ASSERT_STATUS( si5351_write8( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) ));
  ASSERT_STATUS( si5351_write8( baseaddr+6, (P2 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351_write8( baseaddr+7, (P2 & 0x000000FF)));

  /* Reset PLLs to apply new settings. */
  ASSERT_STATUS( si5351_write8(SI5351_REGISTER_177_PLL_RESET, (1<<7) | (1<<5) ));

  /* Cache PLL output frequency for multisynth calculations. */
  if (pll == SI5351_PLL_A)
  {
    float fvco = m_si5351Config.crystalFreq * (mult + ( (float)num / (float)denom ));
    m_si5351Config.plla_configured = 1; //true
    m_si5351Config.plla_freq = (uint32_t)floor(fvco);
  }
  else
  {
    float fvco = m_si5351Config.crystalFreq * (mult + ( (float)num / (float)denom ));
    m_si5351Config.pllb_configured = 1; //true
    m_si5351Config.pllb_freq = (uint32_t)floor(fvco);
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
  @brief  Configure integer multisynth divider.

    @param  output    Output channel (0..2)
    @param  pllSource PLL source
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       Integer multisynth divider
              Supported values:
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
*/
/**************************************************************************/
err_t si5351_setupMultisynthInt(uint8_t               output,
                                          si5351PLL_t           pllSource,
                                          si5351MultisynthDiv_t div)
{
  return si5351_setupMultisynth(output, pllSource, div, 0, 1);
}


err_t si5351_setupRdiv(uint8_t  output, si5351RDiv_t div) {
  ASSERT( output < 3,                 ERROR_INVALIDPARAMETER);  /* channel range */
  
  uint8_t Rreg, regval, rDiv;

  if (output == 0) Rreg = SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3;
  if (output == 1) Rreg = SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3;
  if (output == 2) Rreg = SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3;

  si5351_read8(Rreg, &regval);

  regval &= 0x0F;
  uint8_t divider = div;
  divider &= 0x07;
  divider <<= 4;
  regval |= divider;
  si5351_write8(Rreg, regval);

  switch(div)
  {
  case 0:
  rDiv = 1;
  break;

  case 1:
  rDiv = 2;
  break;

  case 2:
  rDiv = 4;
  break;

  case 3:
  rDiv = 8;
  break;

  case 4:
  rDiv = 16;
  break;

  case 5:
  rDiv = 32;
  break;

  case 6:
  rDiv = 64;
  break;

  case 7:
  rDiv = 128;
  break;
  }

  switch(output)
  {
  case 0:
  m_si5351Config.ms0_r_div = rDiv;
  break;

  case 1:
  m_si5351Config.ms1_r_div = rDiv;
  break;

  case 2:
  m_si5351Config.ms2_r_div = rDiv;
  break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
  @brief  Configure multisynth divider for target output frequency.

    @param  output    Output channel (0..2)
    @param  pllSource PLL source
                      - SI5351_PLL_A
                      - SI5351_PLL_B
    @param  div       Multisynth divider value
              Integer mode values:
                      - SI5351_MULTISYNTH_DIV_4
                      - SI5351_MULTISYNTH_DIV_6
                      - SI5351_MULTISYNTH_DIV_8
              Fractional mode range: 8..900
    @param  num       Fraction numerator (20-bit)
    @param  denom     Fraction denominator (20-bit)

    @section Output Clock Configuration

    The output frequency is generated by dividing PLL VCO frequency.
    Valid practical range is around 500kHz to 160MHz.

        fOUT = fVCO / MSx

    Integer multisynth supports dividers 4, 6, 8.
    Fractional multisynth supports values from 8+1/1048575 to 900.

    Fractional divider form:

        a + b / c

    a = integer divider (4/6/8 in integer mode, 8..900 in fractional mode)
    b = numerator (0..1,048,575)
    c = denominator (1..1,048,575)

    @note   Prefer integer division when possible for lower jitter.

        @note   For outputs >150MHz, divider must be 4 and PLL must match.
          This implementation currently targets about 500kHz..150MHz.

    @note   For very low frequency (<500kHz), use R divider.
*/
/**************************************************************************/
err_t si5351_setupMultisynth(uint8_t     output,
                                       si5351PLL_t pllSource,
                                       uint32_t    div,
                                       uint32_t    num,
                                       uint32_t    denom)
{
  uint32_t P1;       /* MultiSynth config register P1 */
  uint32_t P2;	     /* MultiSynth config register P2 */
  uint32_t P3;	     /* MultiSynth config register P3 */

  /* Parameter validation */
  ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);
  ASSERT( output < 3,                 ERROR_INVALIDPARAMETER);  /* channel range */
  //ASSERT( div > 3,                    ERROR_INVALIDPARAMETER);  /* divider lower bound */
  //ASSERT( div < 901,                  ERROR_INVALIDPARAMETER);  /* divider upper bound */
  //ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* denominator > 0 */
  //ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20-bit numerator */
  //ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20-bit denominator */


  /* Ensure selected PLL has been configured. */
  if (pllSource == SI5351_PLL_A)
  {
    ASSERT(m_si5351Config.plla_configured = 1, ERROR_INVALIDPARAMETER);
  }
  else
  {
    ASSERT(m_si5351Config.pllb_configured = 1, ERROR_INVALIDPARAMETER);
  }

  /* Fractional multisynth register equations:
   * a=div, b=num, c=denom
   * P1[17:0] = 128*a + floor(128*(b/c)) - 512
   * P2[19:0] = 128*b - c*floor(128*(b/c))
   * P3[19:0] = c
   */

  /* Calculate multisynth register values. */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* Get output multisynth register base address. */
  uint8_t baseaddr = 0;
  switch (output)
  {
    case 0:
      baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
      break;
    case 1:
      baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
      break;
    case 2:
      baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
      break;
  }

  /* Write MSx configuration registers. */
  si5351_write8( baseaddr,   (P3 & 0x0000FF00) >> 8);
  si5351_write8( baseaddr+1, (P3 & 0x000000FF));
  si5351_write8( baseaddr+2, (P1 & 0x00030000) >> 16);	/* DIVBY4/R-div related bits */
  si5351_write8( baseaddr+3, (P1 & 0x0000FF00) >> 8);
  si5351_write8( baseaddr+4, (P1 & 0x000000FF));
  si5351_write8( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  si5351_write8( baseaddr+6, (P2 & 0x0000FF00) >> 8);
  si5351_write8( baseaddr+7, (P2 & 0x000000FF));


  if (pllSource == SI5351_PLL_A)
  {
          float fvco = m_si5351Config.plla_freq / (div + ( (float)num / (float)denom ));
          switch (output)
          {
           case 0:
           m_si5351Config.ms0_freq = (uint32_t)floor(fvco);
           break;
           case 1:
           m_si5351Config.ms1_freq = (uint32_t)floor(fvco);
           break;
           case 2:
           m_si5351Config.ms2_freq = (uint32_t)floor(fvco);
           break;
          }
  }
  else
  {
          float fvco = m_si5351Config.pllb_freq / (div + ( (float)num / (float)denom));
          switch (output)
          {
           case 0:
           m_si5351Config.ms0_freq = (uint32_t)floor(fvco);
           break;
           case 1:
           m_si5351Config.ms1_freq = (uint32_t)floor(fvco);
           break;
           case 2:
           m_si5351Config.ms2_freq = (uint32_t)floor(fvco);
           break;
          }
  }



  /* Configure CLKx control register. */
  uint8_t clkControlReg = 0x0F;                             /* 8mA drive, non-inverted, powered */
  if (pllSource == SI5351_PLL_B) clkControlReg |= (1 << 5); /* use PLLB */
  if (num == 0 && (div == 4 || div == 6 || div == 8))       /* integer mode, 仅4/6/8有效 */
    clkControlReg |= (1 << 6);
  else
    clkControlReg &= ~(1 << 6);
  switch (output)
  {
    case 0:
      ASSERT_STATUS(si5351_write8(SI5351_REGISTER_16_CLK0_CONTROL, clkControlReg));
      break;
    case 1:
      ASSERT_STATUS(si5351_write8(SI5351_REGISTER_17_CLK1_CONTROL, clkControlReg));
      break;
    case 2:
      ASSERT_STATUS(si5351_write8(SI5351_REGISTER_18_CLK2_CONTROL, clkControlReg));
      break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enable or disable all clock outputs.
*/
/**************************************************************************/
err_t si5351_enableOutputs(uint8_t enabled)
{
  /* Ensure device is initialized. */
  ASSERT(m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);

  /* Update output enable register (register 3). */
  ASSERT_STATUS(si5351_write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00: 0xFF));

  return ERROR_NONE;
}

/* ---------------------------------------------------------------------- */
/* Private helpers                                                          */
/* ---------------------------------------------------------------------- */

/**************************************************************************/
/*!
  @brief  Write one 8-bit value to Si5351 register via I2C.
*/
/**************************************************************************/
err_t si5351_write8 (uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;
  
	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }

    status = HAL_I2C_Mem_Write(&hi2c1,							// I2C handle
              (uint8_t)(SI5351_ADDRESS<<1),		// 7-bit address shifted
                (uint8_t)reg,						// register address
                I2C_MEMADD_SIZE_8BIT,				// 8-bit mem address size
                (uint8_t*)(&value),				// data pointer
                1,								// data length
                100);							// timeout ms

  return ERROR_NONE;
}

/**************************************************************************/
/*!
  @brief  Read one 8-bit value from Si5351 register via I2C.
*/
/**************************************************************************/
err_t si5351_read8(uint8_t reg, uint8_t *value)
{
	HAL_StatusTypeDef status = HAL_OK;

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(SI5351_ADDRESS<<1), 3, 100) != HAL_OK) { }

    status = HAL_I2C_Mem_Read(&hi2c1,							// I2C handle
              (uint8_t)(SI5351_ADDRESS<<1),		// 7-bit address shifted
                (uint8_t)reg,						// register address
                I2C_MEMADD_SIZE_8BIT,				// 8-bit mem address size
                (uint8_t*)value,				// data pointer
                1,								// data length
                100);							// timeout ms

  return ERROR_NONE;
}

/**
 * @brief Set frequency for one output channel.
 */
err_t SI5351_SetFrequency(uint8_t clk, uint32_t fout)
{
    if (clk > 2) return ERROR_INVALIDPARAMETER;

    uint32_t f_xtal = m_si5351Config.crystalFreq; // 25 MHz
    uint8_t pll = (clk == 0) ? SI5351_PLL_A : SI5351_PLL_B;

    uint32_t fvco;
    uint32_t mult, num = 0, denom = 1;

    /* =====================================================
     * 1. Choose a VCO target in 600..900 MHz
     * ===================================================== */
    uint32_t target_fvco = fout * 32;
    if (target_fvco < 600000000UL)  target_fvco = 600000000UL;
    if (target_fvco > 900000000UL)  target_fvco = 900000000UL;

    mult = target_fvco / f_xtal;
    fvco = f_xtal * mult;

    /* =====================================================
     * 2. Configure PLL in integer mode
     * ===================================================== */
    ASSERT_STATUS(si5351_setupPLL(pll, mult, 0, 1));

    /* =====================================================
     * 3. Compute multisynth divider: MS = fvco / fout
     * ===================================================== */
    float ms_f = (float)fvco / (float)fout;
    uint32_t ms_int = (uint32_t)ms_f;

    /* Select integer or fractional multisynth mode. */
    if (fabs(ms_f - ms_int) < 0.000001)
    {
        /* Integer mode */
        num = 0;
        denom = 1;
    }
    else
    {
        /* Fractional mode */
        denom = 1000000;                      // about 1 ppm resolution
        num = (uint32_t)((ms_f - ms_int) * denom);
    }

    /* =====================================================
     * 4. Configure MultiSynth
     * ===================================================== */
    ASSERT_STATUS(si5351_setupMultisynth(clk, pll, ms_int, num, denom));

    /* =====================================================
     * 5. Optional R divider for low output frequency
     * ===================================================== */
    if (fout < 500000)   si5351_setupRdiv(clk, SI5351_R_DIV_8);
    if (fout <  62500)   si5351_setupRdiv(clk, SI5351_R_DIV_128);

    /* Enable outputs */
    ASSERT_STATUS(si5351_enableOutputs(1));

    return ERROR_NONE;
}

/**
 * @brief Enable or disable one output channel.
 */
err_t SI5351_SetChannelPower(uint8_t channel, uint8_t enabled)
{
    uint8_t reg;
    uint8_t value;
    err_t status;

    // 1. Parameter check: only CLK0/1/2 are supported
    ASSERT(channel <= 2, ERROR_INVALIDPARAMETER);
    
    // 2. Map channel to CLKx control register (starts at register 16)
    // 16 + 0 = 16 (CLK0), 16 + 1 = 17 (CLK1), 16 + 2 = 18 (CLK2)
    reg = SI5351_REGISTER_16_CLK0_CONTROL + channel;

    // 3. Read current register value
    status = si5351_read8(reg, &value);
    ASSERT_STATUS(status);

    // 4. Update Power Down bit (bit 7)
    if (enabled) {
        // Enable channel (Power Down = 0)
        value &= ~(1 << 7); 
    } else {
        // Disable channel (Power Down = 1)
        value |= (1 << 7);
    }

    // 5. Write back the updated value
    status = si5351_write8(reg, value);
    ASSERT_STATUS(status);
    
    // 6. If enabling one channel, ensure global output gate is enabled too
    if (enabled) {
      // Global enable: 1 means outputs enabled
        status = si5351_enableOutputs(1);
        ASSERT_STATUS(status);
    }

    return ERROR_NONE;
}

