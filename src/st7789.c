#include "stm32f4xx.h"
#include "st7789.h"
#include "font5x7.h"

/* delay_ms vem do main (ou de delay.c) */
extern void delay_ms(uint32_t ms);

/* ========================== GPIO helpers =========================== */
static inline void pin_set(GPIO_TypeDef* p, uint32_t pin){ p->BSRR = (1u << pin); }
static inline void pin_clr(GPIO_TypeDef* p, uint32_t pin){ p->BSRR = (1u << (pin + 16)); }

/* ============================ SPI core ============================= */
/* Inicializa SPI1 em MODE3 com divisor configurável.
   br_div: 0:/2 .. 7:/256
   Pinos: PA5=SCK, PA7=MOSI (AF5). CS opcional (forçado LOW se existir).
   GPIOB: DC, RST, BLK, CS conforme board.h */
static void spi1_init_mode3_div(uint8_t br_div){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* PA5 SCK, PA7 MOSI, AF5 */
    GPIOA->MODER   &= ~((3u<<(5*2))|(3u<<(7*2)));
    GPIOA->MODER   |=  (2u<<(5*2))|(2u<<(7*2));
    GPIOA->AFR[0]  &= ~((0xFu<<(5*4))|(0xFu<<(7*4)));
    GPIOA->AFR[0]  |=  (5u<<(5*4))|(5u<<(7*4));
    GPIOA->OSPEEDR |=  (3u<<(5*2))|(3u<<(7*2));

    /* DC/RST/BLK/CS -> saída */
    GPIOB->MODER   &= ~((3u<<(LCD_DC_PIN*2))|(3u<<(LCD_RST_PIN*2))|(3u<<(LCD_BLK_PIN*2))|(3u<<(LCD_CS_PIN*2)));
    GPIOB->MODER   |=  (1u<<(LCD_DC_PIN*2))|(1u<<(LCD_RST_PIN*2))|(1u<<(LCD_BLK_PIN*2))|(1u<<(LCD_CS_PIN*2));
    GPIOB->OSPEEDR |=  (3u<<(LCD_DC_PIN*2))|(3u<<(LCD_RST_PIN*2))|(3u<<(LCD_BLK_PIN*2))|(3u<<(LCD_CS_PIN*2));

    /* CS LOW fixo (se existir) */
    pin_clr(LCD_CS_PORT, LCD_CS_PIN);

    SPI1->CR1 = 0; SPI1->CR2 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | ((uint32_t)br_div << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;  /* MODE3 */
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Espera SPI1 ficar ocioso (BSY=0) e drena SR/DR. */
static inline void spi_wait_idle(void){ while(SPI1->SR & SPI_SR_BSY); (void)SPI1->SR; (void)SPI1->DR; }
/* Configura SPI1 p/ quadro de 8 bits.  */
static inline void spi_set_8bit(void){  SPI1->CR1 &= ~SPI_CR1_SPE; SPI1->CR1 &= ~SPI_CR1_DFF; SPI1->CR1 |= SPI_CR1_SPE; }
/* Configura SPI1 p/ quadro de 16 bits. */
static inline void spi_set_16bit(void){ SPI1->CR1 &= ~SPI_CR1_SPE; SPI1->CR1 |=  SPI_CR1_DFF; SPI1->CR1 |=  SPI_CR1_SPE; }
/* Transmite 1 byte por SPI1 e aguarda idle. */
static inline void spi_tx8(uint8_t b){ while(!(SPI1->SR & SPI_SR_TXE)); *(__IO uint8_t*)&SPI1->DR = b; spi_wait_idle(); }

/* ========================= Comandos do LCD ========================= */
/* Envia comando 8-bit ao ST7789. */
static inline void lcd_cmd(uint8_t c){ pin_clr(LCD_DC_PORT,LCD_DC_PIN); spi_set_8bit();  spi_tx8(c); }
/* Envia dado 8-bit ao ST7789. */
static inline void lcd_d8 (uint8_t d){ pin_set(LCD_DC_PORT,LCD_DC_PIN); spi_set_8bit();  spi_tx8(d); }
/* Envia dado 16-bit ao ST7789. */
static inline void lcd_d16(uint16_t d){ pin_set(LCD_DC_PORT,LCD_DC_PIN); spi_set_16bit(); while(!(SPI1->SR & SPI_SR_TXE)); SPI1->DR=d; spi_wait_idle(); }

/* Reset por pino RST com atrasos padrão. */
static inline void lcd_reset(void){
    pin_set(LCD_DC_PORT,LCD_DC_PIN);
    pin_set(LCD_RST_PORT,LCD_RST_PIN); delay_ms(10);
    pin_clr(LCD_RST_PORT,LCD_RST_PIN); delay_ms(50);
    pin_set(LCD_RST_PORT,LCD_RST_PIN); delay_ms(150);
}

/* Define janela de escrita e envia comando RAMWR (0x2C).
   Coordenadas inclusivas. */
static inline void set_addr(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
    lcd_cmd(0x2A); lcd_d16(x0); lcd_d16(x1);
    lcd_cmd(0x2B); lcd_d16(y0); lcd_d16(y1);
    lcd_cmd(0x2C);
}

/* Sequência de init para ST7789 (16bpp, RGB, INVON).
   Se quiser BGR, troque o MADCTL (0x36) para 0x08. */
static void st7789_init_sequence(void){
    lcd_cmd(0x01); delay_ms(120);   /* SWRESET  */
    lcd_cmd(0x11); delay_ms(120);   /* SLPOUT   */
    lcd_cmd(0x36); lcd_d8(0x00);    /* MADCTL   (RGB)   */
    lcd_cmd(0x3A); lcd_d8(0x55);    /* COLMOD   (16bpp) */
    lcd_cmd(0xB2); lcd_d8(0x0C); lcd_d8(0x0C); lcd_d8(0x00); lcd_d8(0x33); lcd_d8(0x33);
    lcd_cmd(0xB7); lcd_d8(0x35);
    lcd_cmd(0xBB); lcd_d8(0x2B);
    lcd_cmd(0xC0); lcd_d8(0x2C);
    lcd_cmd(0xC2); lcd_d8(0x01);
    lcd_cmd(0xC3); lcd_d8(0x0B);
    lcd_cmd(0xC4); lcd_d8(0x20);
    lcd_cmd(0xC6); lcd_d8(0x0F);
    lcd_cmd(0xD0); lcd_d8(0xA4); lcd_d8(0xA1);
    lcd_cmd(0x21);                /* INVON     */
    lcd_cmd(0x29); delay_ms(20);  /* DISPON    */
    set_addr(0,0,LCD_W-1,LCD_H-1);
}

/* Burst de meia-palavra constante (CPU) para n pixels. */
static inline void push_solid(uint32_t n, uint16_t c){
    pin_set(LCD_DC_PORT,LCD_DC_PIN);
    spi_set_16bit();
    while(n--){
        while(!(SPI1->SR & SPI_SR_TXE));
        SPI1->DR = c;
    }
    spi_wait_idle();
}

/* ============================ API pública ========================== */
void st7789_init(void){
    spi1_init_mode3_div(5);                 /* /64 na partida */
    pin_set(LCD_BLK_PORT, LCD_BLK_PIN);     /* backlight ON   */
    lcd_reset();
    st7789_init_sequence();
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;     /* habilita DMA2  */
}

void st7789_set_speed_div(uint8_t br_div){
    if (br_div > 7) br_div = 7;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= ((uint32_t)br_div << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

void st7789_fill_screen(uint16_t color){
    set_addr(0,0,LCD_W-1,LCD_H-1);
    push_solid((uint32_t)LCD_W * LCD_H, color);
}

void st7789_fill_rect(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t color){
    if (x>=LCD_W || y>=LCD_H) return;
    if (x+w>LCD_W) w = LCD_W-x;
    if (y+h>LCD_H) h = LCD_H-y;
    set_addr(x,y,x+w-1,y+h-1);
    push_solid((uint32_t)w*h, color);
}

/* ============================ DMA helpers ========================== */
/* Garante SPI ocioso (pós-DMA). */
static void spi1_tx_dma_wait_idle(void){
    while (SPI1->SR & SPI_SR_BSY);
    (void)SPI1->SR; (void)SPI1->DR;
}

/* Envia N half-words (16-bit) da memória para SPI1 via DMA (MINC=1). */
static void spi1_tx_dma_block(const uint16_t *src, uint32_t count){
    /* SPI em 16-bit, DC=1 (dados) */
    pin_set(LCD_DC_PORT, LCD_DC_PIN);
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |=  SPI_CR1_DFF;   /* 16-bit */
    SPI1->CR1 |=  SPI_CR1_SPE;

    /* Desliga Stream3 e limpa flags (LOW) */
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN);
    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                  DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    /* Endereços e tamanho */
    DMA2_Stream3->PAR  = (uint32_t)&SPI1->DR;   /* periférico */
    DMA2_Stream3->M0AR = (uint32_t)src;         /* memória (buffer) */
    DMA2_Stream3->NDTR = (uint32_t)count;       /* número de half-words */

    /* Canal 3, Mem->Periph, PSIZE=16, MSIZE=16, MINC=1 */
    DMA2_Stream3->CR =
        (3u << DMA_SxCR_CHSEL_Pos) |
        DMA_SxCR_DIR_0            |   /* 01: Mem->Periph */
        DMA_SxCR_PSIZE_0          |   /* 16-bit */
        DMA_SxCR_MSIZE_0          |   /* 16-bit */
        DMA_SxCR_MINC             |   /* incrementa memória */
        DMA_SxCR_PL_1;

    /* Habilita TXDMA e dispara */
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    DMA2_Stream3->CR |= DMA_SxCR_EN;

    /* Espera TC */
    while (!(DMA2->LISR & DMA_LISR_TCIF3));

    /* Desliga e limpa */
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN);
    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                  DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    SPI1->CR2 &= ~SPI_CR2_TXDMAEN;
    spi1_tx_dma_wait_idle();
}

/* Envia um “sólido” (mesma cor) usando blocos repetidos. */
static void spi1_tx_dma_solid(uint16_t color, uint32_t total_pixels){
    enum { CHUNK = 128 };            /* 128 pixels por envio */
    static uint16_t buf[CHUNK];

    /* Preenche o bloco com a cor desejada */
    for (int i = 0; i < CHUNK; i++) buf[i] = color;

    while (total_pixels){
        uint32_t n = (total_pixels > CHUNK) ? CHUNK : total_pixels;
        spi1_tx_dma_block(buf, n);
        total_pixels -= n;
    }
}

/* ============================ API DMA ============================== */
void st7789_fill_rect_dma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
    if (x>=LCD_W || y>=LCD_H) return;
    if (x+w>LCD_W) w = LCD_W-x;
    if (y+h>LCD_H) h = LCD_H-y;
    if (w==0 || h==0) return;

    /* Janela + RAMWR */
    lcd_cmd(0x2A); lcd_d16(x);       lcd_d16(x+w-1);
    lcd_cmd(0x2B); lcd_d16(y);       lcd_d16(y+h-1);
    lcd_cmd(0x2C);

    /* Envio sólido por blocos (MINC=1) */
    spi1_tx_dma_solid(color, (uint32_t)w*h);
}

void st7789_fill_screen_dma(uint16_t color){
    st7789_fill_rect_dma(0,0,LCD_W,LCD_H,color);
}

/* ============================ GFX básicas ========================== */
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color){
    if (x >= LCD_W || y >= LCD_H) return;
    set_addr(x, y, x, y);
    push_solid(1, color);
}

void st7789_draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color){
    if (y >= LCD_H || x >= LCD_W) return;
    if (x+w > LCD_W) w = LCD_W - x;
    set_addr(x, y, x+w-1, y);
    push_solid(w, color);
}

void st7789_draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color){
    if (x >= LCD_W || y >= LCD_H) return;
    if (y+h > LCD_H) h = LCD_H - y;
    set_addr(x, y, x, y+h-1);
    push_solid(h, color);
}

void st7789_draw_line(int x0, int y0, int x1, int y1, uint16_t color){
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;){
        st7789_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2*err;
        if (e2 >= dy){ err += dy; x0 += sx; }
        if (e2 <= dx){ err += dx; y0 += sy; }
    }
}

void st7789_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
    st7789_draw_hline(x, y, w, color);
    st7789_draw_hline(x, y+h-1, w, color);
    st7789_draw_vline(x, y, h, color);
    st7789_draw_vline(x+w-1, y, h, color);
}

void st7789_draw_circle(int x0, int y0, int r, uint16_t color){
    int x = r, y = 0, err = 0;
    while (x >= y){
        st7789_draw_pixel(x0+x, y0+y, color);
        st7789_draw_pixel(x0+y, y0+x, color);
        st7789_draw_pixel(x0-y, y0+x, color);
        st7789_draw_pixel(x0-x, y0+y, color);
        st7789_draw_pixel(x0-x, y0-y, color);
        st7789_draw_pixel(x0-y, y0-x, color);
        st7789_draw_pixel(x0+y, y0-x, color);
        st7789_draw_pixel(x0+x, y0-y, color);
        y++;
        if (err <= 0){ err += 2*y+1; }
        if (err > 0){ x--; err -= 2*x+1; }
    }
}

void st7789_fill_circle(int x0, int y0, int r, uint16_t color){
    int x = r, y = 0, err = 0;
    while (x >= y){
        st7789_draw_hline(x0-x, y0+y, 2*x+1, color);
        st7789_draw_hline(x0-x, y0-y, 2*x+1, color);
        st7789_draw_hline(x0-y, y0+x, 2*y+1, color);
        st7789_draw_hline(x0-y, y0-x, 2*y+1, color);
        y++;
        if (err <= 0){ err += 2*y+1; }
        if (err > 0){ x--; err -= 2*x+1; }
    }
}

/* ============================== Texto ============================= */
static void draw_char_5x7(int x, int y, char ch, uint16_t fg, int scale, int bg_en, uint16_t bg){
    if (ch < 32 || ch > 127) ch = '?';
    const uint8_t* col = FONT5x7[ch - 32];
    for (int cx=0; cx<5; cx++){
        uint8_t bits = col[cx];
        for (int sx=0; sx<scale; sx++){
            int px = x + cx*scale + sx;
            if (px < 0 || px >= LCD_W) continue;
            for (int cy=0; cy<7; cy++){
                int py0 = y + cy*scale;
                if (py0 >= LCD_H) break;
                if (bits & (1<<cy)) {
                    set_addr(px, py0, px, py0 + (scale-1));
                    push_solid(scale, fg);
                } else if (bg_en){
                    set_addr(px, py0, px, py0 + (scale-1));
                    push_solid(scale, bg);
                }
            }
        }
    }
    /* coluna de espaçamento à direita (bg opcional) */
    if (bg_en){
        for (int sx=0; sx<scale; sx++){
            int px = x + 5*scale + sx;
            if (px < 0 || px >= LCD_W) continue;
            set_addr(px, y, px, y + 7*scale - 1);
            push_solid(7*scale, bg);
        }
    }
}

void st7789_draw_text_5x7(int x, int y, const char* s, uint16_t fg, int scale, int bg_en, uint16_t bg){
    int cx = x, cy = y;
    if (scale < 1) scale = 1;
    while(*s){
        if (*s=='\n'){ cy += 8*scale; cx = x; s++; continue; }
        draw_char_5x7(cx, cy, *s, fg, scale, bg_en, bg);
        cx += 6*scale;
        s++;
        if (cx >= (LCD_W-6*scale)) { cy += 8*scale; cx = x; }
        if (cy >= (LCD_H-8*scale)) break;
    }
}
