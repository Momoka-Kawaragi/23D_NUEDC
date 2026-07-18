/* 9959_scan.h - Simple frequency sweep driver for AD9959 */
#ifndef __9959_SCAN_H__
#define __9959_SCAN_H__

#include <stdint.h>

/* Sweep configuration */
#define SWEEP_START_FREQ_HZ    1000U
#define SWEEP_STOP_FREQ_HZ     500000U
#define SWEEP_STEP_HZ          500U
#define SWEEP_SETTLE_MS        100U
#define SWEEP_CHANNEL          0U

/* Run frequency sweep. Iterates from START to STOP with STEP Hz,
   setting AD9959 frequency + amplitude, waiting settle_ms between steps. */
void SCAN_RunSweep(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                   uint16_t amplitude, uint32_t settle_ms);

#endif /* __9959_SCAN_H__ */
