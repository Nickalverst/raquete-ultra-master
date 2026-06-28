#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"  
#include <stdio.h>

#define LED_PORT GPIOC
#define LED_PIN  13u
#define PIN_MASK (1u << LED_PIN)

static inline void led_on(void)  { LED_PORT->BSRR = (PIN_MASK << 16); }
static inline void led_off(void) { LED_PORT->BSRR = PIN_MASK; }
static inline void led_toggle(void) {
    if (LED_PORT->ODR & PIN_MASK) { led_on(); } else { led_off(); }
}

static void gpio_init_led_pc13(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    LED_PORT->MODER   = (LED_PORT->MODER & ~(0x3u << (LED_PIN * 2))) | (0x1u << (LED_PIN * 2));
    LED_PORT->OTYPER &= ~(1u << LED_PIN);
    LED_PORT->OSPEEDR &= ~(0x3u << (LED_PIN * 2));
    LED_PORT->PUPDR   &= ~(0x3u << (LED_PIN * 2));
}


// Task A: pisca rápido 
static void vTaskA(void *arg) {
    (void)arg;
    for (;;) {
        led_toggle();
        printf("[A] LED toggled\n");
        vTaskDelay(pdMS_TO_TICKS(200));   // pausa 200 ms
    }
}

// Task B: pisca mais devagar 
static void vTaskB(void *arg) {
    (void)arg;
    for (;;) {
        led_toggle();
        printf("[B] LED toggled\n");
        vTaskDelay(pdMS_TO_TICKS(700));   // pausa 700 ms
    }
}

int main(void) {
    SystemCoreClockUpdate();
    gpio_init_led_pc13();
    led_off();

    serial_stdio_init(115200);

    xTaskCreate(vTaskA, "A", 256, NULL, 1, NULL);
    xTaskCreate(vTaskB, "B", 256, NULL, 1, NULL);

    // Inicia o escalonador
    vTaskStartScheduler();

    while (1) { __NOP(); }
}
