#include "stm32f4xx.h"
#include "delay.h"

/*
 * delay.c — implementação via DWT (Data Watchpoint & Trace)
 *
 * NÃO usa SysTick, portanto não conflita com o FreeRTOS,
 * que precisa do SysTick exclusivamente para o seu tick.
 *
 * delay_ms() funciona tanto antes quanto depois do scheduler iniciar.
 * Dentro de tasks, prefira vTaskDelay() para não desperdiçar CPU.
 *
 * millis() retorna ms desde boot usando o contador DWT.
 * Dentro de tasks, prefira xTaskGetTickCount() (que é o tick do RTOS).
 */

void delay_init(void)
{
    /* Habilita o bloco DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Zera e liga o contador de ciclos */
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_ms(uint32_t ms)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    while (ms--)
    {
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles_per_ms)
        {
            __NOP();
        }
    }
}

uint32_t millis(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000u);
}