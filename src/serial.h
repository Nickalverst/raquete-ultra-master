#pragma once
#include <stdint.h>

/* Inicializa USART1 (PA9 TX, PA10 RX) no baud desejado. */
void serial_init(uint32_t baud);

/* Escreve string terminada em '\0' (TX polling). */
void serial_write(const char *s);

/* ---- Utilidades de TX/RX simples ---- */
void     serial_putc(uint8_t c);   /* envia 1 byte (polling) */
int      serial_tx_done(void);     /* 1 = transmissão finalizada (TC=1) */
int      serial_readable(void);    /* 1 = há byte disponível (RXNE=1)   */
int      serial_getc_blocking(void); /* lê 1 byte (bloqueante) */

/* ---- Retarget de stdio (printf) ----
   Configura USART1 e ajusta stdout para sem buffer.
   Depois disso: printf("oi\r\n"); já imprime na serial. */
void serial_stdio_init(uint32_t baud);
