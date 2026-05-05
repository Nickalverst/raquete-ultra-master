#pragma once
#include <stdint.h>

/* Inicializa SysTick para gerar 1 kHz (1 ms por tick). */
void delay_init(void);

/* Espera bloqueando pelo número de milissegundos indicado. */
void delay_ms(uint32_t ms);

/* Retorna o contador de milissegundos desde o delay_init(). */
uint32_t millis(void);
