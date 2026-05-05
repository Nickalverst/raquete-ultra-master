#include "serial_stdio.h"
#include <sys/unistd.h>  // _write
#include <stdio.h>

static inline void uart1_putc(uint8_t c) {
    while ((USART1->SR & USART_SR_TXE) == 0u) {}
    USART1->DR = c;
}
static inline int uart1_txc_done(void) {
    return (USART1->SR & USART_SR_TC) != 0;
}

// Redundant - using implementation from serial.c instead
// void serial_stdio_init(uint32_t baud) {
//     ...
// }
// 
// int _write(int fd, const void *buf, size_t count) {
//     ...
// }

// Redundant - using implementation from serial.c instead
// __attribute__((weak)) int _read(int fd, void *buf, size_t count) {
//     (void)fd; (void)buf; (void)count; return 0;
// }
// __attribute__((weak)) caddr_t _sbrk(int incr) {
//     extern uint8_t _end;     // fornecido pelo linker
//     static uint8_t *heap_end;
//     uint8_t *prev;
//     if (heap_end == 0) heap_end = &_end;
//     prev = heap_end;
//     heap_end += incr;
//     return (caddr_t)prev;
// }

// Exposição opcional:
// void serial_putc(uint8_t c) { uart1_putc(c); }
// int  serial_tx_done(void)   { return uart1_txc_done(); }
