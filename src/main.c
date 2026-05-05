#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "serial.h"
#include "../include/stm32f4xx.h"
#include "delay.h"
#include "st7789.h"

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

int main(void)
{
  test_multiplexer();
}

// Toggle LED:
// GPIOC->ODR ^= (1u << 13u);