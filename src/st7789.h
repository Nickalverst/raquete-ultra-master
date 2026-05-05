#pragma once
#include <stdint.h>
#include "board.h"

void st7789_init(void);

/* alterar o prescaler após init  br_div: 0:/2 1:/4 2:/8 3:/16 4:/32 5:/64 6:/128 7:/256 */
void st7789_set_speed_div(uint8_t br_div);

/* Desenho básico (CPU) */
void st7789_fill_screen(uint16_t color);
void st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* Versões DMA */
void st7789_fill_screen_dma(uint16_t color);
void st7789_fill_rect_dma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* GFX adicionais */
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void st7789_draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void st7789_draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void st7789_draw_line(int x0, int y0, int x1, int y1, uint16_t color);
void st7789_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void st7789_draw_circle(int x0, int y0, int r, uint16_t color);
void st7789_fill_circle(int x0, int y0, int r, uint16_t color);

/* Texto 5x7 com escala; fundo opcional quando bg_enable != 0 */
void st7789_draw_text_5x7(int x, int y, const char* s, uint16_t fg, int scale, int bg_enable, uint16_t bg);
