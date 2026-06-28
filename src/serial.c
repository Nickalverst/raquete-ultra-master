#include "stm32f4xx.h"
#include "serial.h"

#include <stdio.h>       // setvbuf
#include <sys/unistd.h>  // _write

/* ================= UART1 low-level helpers ================= */
static inline void uart1_putc_ll(uint8_t c) {
    while ((USART1->SR & USART_SR_TXE) == 0u) { /* wait */ }
    USART1->DR = c;
}
static inline int uart1_txc_done_ll(void) {
    return (USART1->SR & USART_SR_TC) != 0;
}

/* ================= Clock helpers ================= */
static inline uint32_t _ahb_div(uint32_t hpre){
    static const uint16_t map[16] = {1,1,1,1,1,1,1,1, 2,4,8,16,64,128,256,512};
    return map[hpre & 0xF];
}
static inline uint32_t _apb_div(uint32_t ppre){
    static const uint8_t map[8] = {1,1,1,1, 2,4,8,16};
    return map[ppre & 0x7];
}
static inline uint32_t _pclk2_hz(void){
    SystemCoreClockUpdate();
    uint32_t cfgr  = RCC->CFGR;
    uint32_t hpre  = (cfgr >> 4)  & 0xF;   // AHB prescaler bits
    uint32_t ppre2 = (cfgr >> 13) & 0x7;   // APB2 prescaler bits
    uint32_t hclk  = SystemCoreClock / _ahb_div(hpre);
    return hclk / _apb_div(ppre2);         // PCLK2 real (para USART1)
}

/* Calcula BRR para OVER8=0 (oversampling 16) */
static inline uint32_t usart1_brr_from_clk(uint32_t pclk, uint32_t baud){
    uint32_t div16 = (pclk + (baud/2u)) / baud;
    uint32_t mant  = div16 / 16u;
    uint32_t frac  = div16 % 16u;
    return (mant << 4) | (frac & 0xFu);
}

/* ================= API simples ================= */
void serial_init(uint32_t baud){
    /* Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9=TX, PA10=RX (AF7) */
    GPIOA->MODER   &= ~((3u<<(9*2)) | (3u<<(10*2)));
    GPIOA->MODER   |=  ((2u<<(9*2)) | (2u<<(10*2)));     /* AF */
    GPIOA->PUPDR   &= ~((3u<<(9*2)) | (3u<<(10*2)));     /* no pull */
    GPIOA->OSPEEDR |=  (2u<<(9*2));                      /* TX um pouco mais rápido */
    GPIOA->AFR[1]  &= ~((0xFu<<((9-8)*4)) | (0xFu<<((10-8)*4)));
    GPIOA->AFR[1]  |=  ((7u  <<((9-8)*4)) | (7u  <<((10-8)*4)));

    /* USART1: 8N1, oversampling 16, sem flow-control */
    USART1->CR1 = 0u;
    USART1->CR2 = 0u;
    USART1->CR3 = 0u;

    /* BRR: calcula automaticamente o clock real de APB2 */
    uint32_t pclk2 = _pclk2_hz();
    USART1->CR1 &= ~USART_CR1_OVER8;    // oversampling 16
    USART1->BRR = usart1_brr_from_clk(pclk2, baud);

    /* Habilita TX/RX + USART */
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;

    /* pequeno atraso para estabilizar antes do primeiro envio */
    for (volatile uint32_t i = 0; i < 30000; ++i) __NOP();
}

void serial_write(const char *s){
    while (*s) {
        char c = *s++;
        if (c == '\n') uart1_putc_ll('\r'); /* CRLF */
        uart1_putc_ll((uint8_t)c);
    }
}

void serial_putc(uint8_t c){ uart1_putc_ll(c); }
int  serial_tx_done(void)  { return uart1_txc_done_ll(); }
int  serial_readable(void) { return (USART1->SR & USART_SR_RXNE) != 0; }

int  serial_getc_blocking(void){
    while (!serial_readable()) { /* wait */ }
    return (int)(USART1->DR & 0xFF);
}

/* ================= Retarget de stdio ================= */
void serial_stdio_init(uint32_t baud){
    serial_init(baud);
    setvbuf(stdout, NULL, _IONBF, 0);
}

int _write(int fd, const void *buf, size_t count) {
    (void)fd;
    const uint8_t *p = (const uint8_t*)buf;
    for (size_t i = 0; i < count; i++) {
        uint8_t c = p[i];
        if (c == '\n') uart1_putc_ll('\r');
        uart1_putc_ll(c);
    }
    while (!uart1_txc_done_ll()) {}
    return (int)count;
}

/* ---- Stubs opcionais (evitam link-errors com newlib-nano) ---- */
__attribute__((weak)) int _read(int fd, void *buf, size_t count) {
    (void)fd; (void)buf; (void)count; return 0;
}
__attribute__((weak)) caddr_t _sbrk(int incr) {
    extern uint8_t _end;
    static uint8_t *heap_end;
    uint8_t *prev;
    if (heap_end == 0) heap_end = &_end;
    prev = heap_end;
    heap_end += incr;
    return (caddr_t)prev;
}
