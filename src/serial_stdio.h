#ifndef SERIAL_STDIO_H
#define SERIAL_STDIO_H

#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa USART1 (PA9/PA10, AF7) e redireciona printf/puts/printf… via _write()
// Ex.: serial_stdio_init(115200);
void serial_stdio_init(uint32_t baud);

// Opcional: acesso bruto a TX
void serial_putc(uint8_t c);
int  serial_tx_done(void);

#ifdef __cplusplus
}
#endif
#endif
