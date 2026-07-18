#include "delay.h"
#include "stm32h7xx.h"

static uint32_t cycles_per_us = 0U;

void Delay_Init(void)
{
    /* Enable DWT cycle counter if available */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) cycles_per_us = 1U;
}

void Delay_us(uint32_t us)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U || cycles_per_us == 0U)
    {
        Delay_Init();
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t target = us * cycles_per_us;
    while ((DWT->CYCCNT - start) < target)
    {
        __NOP();
    }
}

void Delay_ns(uint32_t ns)
{
    /* Approximate ns delay using cycles_per_us. This is not cycle-accurate for small ns values.
       ns -> cycles = ns * SystemCoreClock / 1e9
    */
    if (cycles_per_us == 0U) Delay_Init();
    uint64_t cycles = ((uint64_t)ns * (uint64_t)SystemCoreClock) / 1000000000ULL;
    if (cycles == 0ULL) return;
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < (uint32_t)cycles)
    {
        __NOP();
    }
}
