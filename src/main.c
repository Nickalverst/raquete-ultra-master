#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "serial.h"
#include "serial_stdio.h"
#include "st7789.h"
#include "../include/stm32f4xx.h"
#include "delay.h"
#include "mpu6050.h"

static inline int  uart_rx_ready(void) { return (USART1->SR & USART_SR_RXNE) != 0; }
static inline char uart_getc(void)     { return (char)USART1->DR; }

uint16_t adc_read_channel(uint8_t ch)
{
    ADC1->SQR3 = ch;

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC));

    return ADC1->DR;
}

void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;

    // PA0-PA7 analog
    GPIOA->MODER |=
        (3u<<(0*2)) |
        (3u<<(1*2)) |
        (3u<<(2*2)) |
        (3u<<(3*2)) |
        (3u<<(4*2)) |
        (3u<<(5*2)) |
        (3u<<(6*2)) |
        (3u<<(7*2));

    // PB0-PB1 analog
    GPIOB->MODER |= (3u<<(0*2)) |
                    (3u<<(1*2));

    // PC0-PC1 analog
    GPIOC->MODER |= (3u<<(0*2)) |
                    (3u<<(1*2));

    ADC1->CR2 = 0;

    // Long sample time helps high impedance sensors
    ADC1->SMPR2 = 0x3FFFFFFF;
    ADC1->SMPR1 = 0x0000003F;

    ADC1->CR2 |= ADC_CR2_ADON;
}

void compute_heatmap(uint32_t heatmap[PIEZO_COUNT], uint32_t *hit_counter)
{
    for (uint8_t ch = 0; ch < PIEZO_COUNT; ch++)
    {
        uint16_t val = adc_read_channel(PIEZO_ADC_CHANNELS[ch]);

        if (val > PIEZO_THRESHOLD)
        {
            printf("CH %d = %d\n", ch, val);
            heatmap[ch]++;
            (*hit_counter)++;
        }

        //delay_ms(1);
    }
}

void display_heatmap(uint8_t heatmap[PIEZO_COUNT])
{
    // Exibe heatmap no LCD (3x3)
    for (uint8_t i = 0; i < PIEZO_COUNT; i++)
    {
        uint16_t red_level = (uint16_t)heatmap[i] * 31u / 255u;
        uint16_t color = red_level << 11;
        uint16_t x = (i % 3) * (LCD_W / 3);
        uint16_t y = (i / 3) * (LCD_H / 3);
        st7789_fill_rect_dma(x, y, LCD_W / 3, LCD_H / 3, color);
    }
}

int main(void)
{
    delay_init();
    serial_stdio_init(115200);

    printf("Starting MPU6050 setup...\n");

    /* PB8=SCL, PB9=SDA on I2C1 */
    i2c1_init_100k(50000000u);
    delay_ms(100);

    uint8_t whoami;
    if (i2c1_read_reg(0x68, MPU6050_REG_WHOAMI, &whoami) == 0) {
        printf("Found MPU6050 at 0x68\n");
    }
    else if (i2c1_read_reg(0x69, MPU6050_REG_WHOAMI, &whoami) == 0) {
        printf("Found MPU6050 at 0x69\n");
    } else {
        printf("MPU6050 not responding on I2C1, check wiring and pull-ups\n");
    }

    if (mpu6050_init() < 0) {
        printf("MPU6050 init failed\n");
        for (;;);
    }

    mpu6050_raw_t imu;

    for (;;) {
        if (mpu6050_read_all(&imu) == 0) {
            float ax = mpu6050_accel_g(imu.ax);
            float ay = mpu6050_accel_g(imu.ay);
            float az = mpu6050_accel_g(imu.az);
            printf("Accel: X=%.3fg Y=%.3fg Z=%.3fg\n", ax, ay, az);
        } else {
            printf("MPU6050 read failed\n");
        }
        delay_ms(250);
    }
}