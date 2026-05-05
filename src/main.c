#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "serial.h"
#include "../include/stm32f4xx.h"
#include "delay.h"
#include "st7789.h"

static inline int  uart_rx_ready(void) { return (USART1->SR & USART_SR_RXNE) != 0; }
static inline char uart_getc(void)     { return (char)USART1->DR; }

void mux_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable MUX_PORT (GPIOB)

    // Set PB5, PB6, PB13, PB14, PB15 as output
    MUX_PORT->MODER &= ~(
        (3u<<(MUX_S0_PIN*2)) |
        (3u<<(MUX_S1_PIN*2)) |
        (3u<<(MUX_S2_PIN*2)) |
        (3u<<(MUX_S3_PIN*2)) |
        (3u<<(MUX_EN_PIN*2))
    );

    MUX_PORT->MODER |= (
        (1u<<(MUX_S0_PIN*2)) |
        (1u<<(MUX_S1_PIN*2)) |
        (1u<<(MUX_S2_PIN*2)) |
        (1u<<(MUX_S3_PIN*2)) |
        (1u<<(MUX_EN_PIN*2))
    );

    // Enable mux (LOW)
    MUX_PORT->BSRR = (1u << (MUX_EN_PIN + 16));
}

void mux_select(uint8_t ch)
{
    // Clear/set
    MUX_PORT->BSRR =
                  ((ch & 0x01) ? (1u << MUX_S0_PIN) : (1u << (MUX_S0_PIN+16))) |
                  ((ch & 0x02) ? (1u << MUX_S1_PIN) : (1u << (MUX_S1_PIN+16))) |
                  ((ch & 0x04) ? (1u << MUX_S2_PIN) : (1u << (MUX_S2_PIN+16))) |
                  ((ch & 0x08) ? (1u << MUX_S3_PIN) : (1u << (MUX_S3_PIN+16)));
}

void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA0 analog
    GPIOA->MODER |= (3u << (0*2));

    ADC1->CR2 = 0;
    ADC1->SQR3 = 0; // channel 0
    ADC1->SMPR2 |= (7u << 0); // max sample time

    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t adc_read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC));

    return ADC1->DR;
}

void test_multiplexer(void)
{
    delay_init();
    serial_init(115200);
    mux_gpio_init();
    adc_init();

    // liga clock do GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // PC13 como saída (01)
    GPIOC->MODER &= ~(3u << (LED_PIN*2));
    GPIOC->MODER |=  (1u << (LED_PIN*2)); // output

    printf("Iniciando execução.\n");

    while (1)
    {
        for (uint8_t ch = 0; ch < 16; ch++)
        {
            mux_select(ch);

            delay_ms(5); // settle time

            uint16_t val = adc_read();

            printf("CH %d = %d\n", ch, val);

            if (val > 2000)
                GPIOC->ODR |= (1u << LED_PIN);
            else
                GPIOC->ODR &= ~(1u << LED_PIN);

            delay_ms(200);
        }
    }
}

void compute_heatmap(uint8_t heatmap[9], uint32_t *hit_counter)
{
    // Lendo 9 canais do ADC e atualizando heatmap
    for (uint8_t ch = 0; ch < 9; ch++)
    {
        mux_select(ch);

        delay_ms(1); // settle time

        uint16_t val = adc_read();

        printf("CH %d = %d\n", ch, val);

        if (val > 2000)
        {
            heatmap[ch]++;
            (*hit_counter)++;
        }

    }
}

void display_heatmap(uint8_t heatmap[9])
{
    // Exibe heatmap no LCD (3x3)
    for (uint8_t i = 0; i < 9; i++)
    {
        uint16_t red_level = (uint16_t)heatmap[i] * 31u / 255u;
        uint16_t color = red_level << 11;
        uint16_t x = (i % 3) * (LCD_W / 3);
        uint16_t y = (i / 3) * (LCD_H / 3);
        st7789_fill_rect(x, y, LCD_W / 3, LCD_H / 3, color);
    }
}

int main(void)
{
    delay_init();
    serial_stdio_init(115200);
    mux_gpio_init();
    adc_init();
    st7789_init();
    st7789_set_speed_div(0);

    uint8_t heatmap[9] = {0}; 
    uint32_t hit_counter = 0;

    st7789_fill_screen(C_BLACK);

    for(;;){
        if (uart_rx_ready()){
            char c = uart_getc();
            compute_heatmap(heatmap, &hit_counter);
            // print heatmap values for debugging
            printf("Heatmap: ");
            for (int i = 0; i < 9; i++) {
                printf("%ld ", heatmap[i]);
            }
            printf("\n");
            printf("Total hits: %ld\n", hit_counter);
            // Normalize heatmap values to 0-255 range for color mapping
            uint8_t normalized_heatmap[9];
            // Get the max value from the heatmap for normalization
            uint32_t max_value = 0;
            for (int i = 0; i < 9; i++) {
                if (heatmap[i] > max_value) {
                    max_value = heatmap[i];
                }
            }
            for (int i = 0; i < 9; i++) {
                normalized_heatmap[i] = (max_value > 0) ? (heatmap[i] * 255 / max_value) : 0;
            }
            // print normalized heatmap values for debugging
            printf("Normalized Heatmap: ");
            for (int i = 0; i < 9; i++) {
                printf("%d ", normalized_heatmap[i]);
            }
            printf("\n");
            display_heatmap(normalized_heatmap);
        }
    }
}

// Toggle LED:
// GPIOC->ODR ^= (1u << 13u);