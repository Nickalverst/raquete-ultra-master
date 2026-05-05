#include "stm32f4xx.h"
#include "delay.h"

/* contador de milissegundos (incrementado no SysTick) */
static volatile uint32_t g_ms = 0;

/* IRQ do SysTick: incrementa 1 ms */
void SysTick_Handler(void){
    g_ms++;
}

/* Configura SysTick para 1 kHz usando o clock atual da CPU. */
void delay_init(void){
    SystemCoreClockUpdate();                             
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;         /* 1 ms */
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk              /* usa clock da CPU */
                  | SysTick_CTRL_TICKINT_Msk                /* habilita IRQ */
                  | SysTick_CTRL_ENABLE_Msk;                /* liga SysTick */
}

void delay_ms(uint32_t ms){
    uint32_t start = g_ms;
    while ((g_ms - start) < ms){
        __NOP();                                            
    }
}

uint32_t millis(void){
    return g_ms;
}
