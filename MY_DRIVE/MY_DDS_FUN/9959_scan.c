/* 9959_scan.c - Simple frequency sweep driver for AD9959 */
#include "9959_scan.h"
#include "ad9959.h"
// #include "delay.h"

void SCAN_RunSweep(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                   uint16_t amplitude, uint32_t settle_ms)
{
  if (step_hz == 0U || stop_hz <= start_hz) return;

  for (uint32_t freq = start_hz; freq <= stop_hz; freq += step_hz)
  {
    Write_Frequence(SWEEP_CHANNEL, freq);
    Write_Amplitude(SWEEP_CHANNEL, amplitude);
    AD9959_IO_Update();
    // Delay_us(settle_ms * 1000U);
    HAL_Delay(settle_ms);
  }
}
