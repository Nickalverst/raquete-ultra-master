#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "serial.h"
#include "serial_stdio.h"
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

int main(void)
{
    delay_init();
    serial_stdio_init(115200);

    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER  &= ~((3u<<(8*2)) | (3u<<(9*2))); // PB8/PB9 como GPIO output
    GPIOB->MODER  |=  ((1u<<(8*2)) | (1u<<(9*2)));
    GPIOB->BSRR    =  (1u<<8) | (1u<<9);           // força HIGH
    for (int i = 0; i < 9; i++) {                   // 9 pulsos de clock para destravar
        GPIOB->BSRR = (1u<<(8+16));
        delay_ms(1);
        GPIOB->BSRR = (1u<<8);
        delay_ms(1);
    }

    /* PB8=SCL, PB9=SDA on I2C1 */
    i2c1_init_100k(16000000u);
    delay_ms(100);

    if (mpu6050_init() < 0) {
        printf("MPU6050 init failed\n");
        for (;;);
    }

    mpu6050_raw_t    imu;
    mpu6050_raw_t    ref;
    float            yaw = 0.0f;
    const float      DT  = 0.250f;   // deve bater com delay_ms(250)

    // Referência de offset (posição inicial = zero)
    mpu6050_read_all(&ref);

    for (;;) {
        if (mpu6050_read_all(&imu) == 0) {

            // Subtrai offset de aceleração
            imu.ax -= ref.ax;
            imu.ay -= ref.ay;

            // Calcula orientação completa
            mpu6050_orientation_t ori =
                mpu6050_orientation_update(&imu, yaw, DT);
            yaw = ori.yaw_deg;  // preserva yaw para próxima iteração

            // Converte para inteiros (sem -u _printf_float)
            int ax_mg = (int)(mpu6050_accel_g(imu.ax) * 1000);
            int ay_mg = (int)(mpu6050_accel_g(imu.ay) * 1000);
            int az_mg = (int)(mpu6050_accel_g(imu.az) * 1000);
            int roll  = (int)ori.roll_deg;
            int pitch = (int)ori.pitch_deg;
            int yaw_i = (int)ori.yaw_deg;

            // Indicador de estado
            const char *tilt;
            if      (pitch >  45) tilt = "FRENTE  >>>";
            else if (pitch < -45) tilt = "<<< ATRAS  ";
            else if (roll  >  45) tilt = "DIREITA vvv";
            else if (roll  < -45) tilt = "^^^ ESQUERDA";
            else                  tilt = "PLANO  [===]";

            printf("----------------------------------\r\n");
            printf("Accel : X=%5dmg  Y=%5dmg  Z=%5dmg\r\n", ax_mg, ay_mg, az_mg);
            printf("Roll  : %4ddeg   Pitch: %4ddeg   Yaw: %4ddeg\r\n",
                roll, pitch, yaw_i);
            printf("Estado: %s\r\n", tilt);

        } else {
            printf("MPU6050 read failed\r\n");
        }
        delay_ms(250);
    }
}