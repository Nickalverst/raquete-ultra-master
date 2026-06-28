/*
 * raquete_freertos — main.c
 *
 * Task 1: vTaskIMU  — leitura MPU-6050          (prio 2, 50 ms)
 * Task 2: vTaskADC  — varredura piezo            (prio 3,  5 ms)
 * Task 3: vTaskTX   — transmissão HC-12          (prio 1, bloqueante)
 * Task 4: vTaskLCD  — heatmap no display ST7789  (prio 1, bloqueante)
 *
 * IPC:
 *   xQueueIMU    (cap 4)  — vTaskIMU  → vTaskTX
 *   xQueueHit    (cap 8)  — vTaskADC  → vTaskTX
 *   xQueueHitLCD (cap 8)  — vTaskADC  → vTaskLCD
 *   xUartMutex            — protege serial_write em vTaskTX
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "mpu6050.h"
#include "delay.h"
#include "st7789.h"
#include "racket_types.h"

/* ─── Identificação da raquete ───────────────────────────── */
#define RACKET_ID   "RAQ01"

/* ─── Thresholds e dimensões ─────────────────────────────── */
#define ADC_IMPACT_THRESHOLD   1000u
#define ADC_CHANNELS           9u
#define DEBOUNCE_MS            120u   /* ms entre dois hits no mesmo sensor */

/* ─── Configuração visual do heatmap ─────────────────────── */
#define GRID_SIZE   3
#define CELL_DIM    80

/* Patamares absolutos de hits para cada cor */
#define MIN_HITS_CYAN    10u
#define MIN_HITS_GREEN   25u
#define MIN_HITS_YELLOW  50u
#define MIN_HITS_ORANGE  80u
#define MIN_HITS_RED    120u

/* ─── Filas FreeRTOS ─────────────────────────────────────── */
static QueueHandle_t xQueueIMU;       /* imu_data_t — IMU  → TX  */
static QueueHandle_t xQueueHit;       /* hit_data_t — ADC  → TX  */
static QueueHandle_t xQueueHitLCD;   /* hit_data_t — ADC  → LCD */

/* ─── Mutex para acesso à UART ───────────────────────────── */
static SemaphoreHandle_t xUartMutex;

/* ─── Mapa de canais ADC: PA0-PA7 = CH0-7, PB0 = CH8 ─────── */
static const uint8_t adc_ch[ADC_CHANNELS] = {0,1,2,3,4,5,6,7,8};

/* ─── Heatmap (lido/escrito apenas por vTaskADC) ─────────── */
static uint16_t g_heatmap[ADC_CHANNELS]   = {0};
static uint32_t g_last_hit_ms[ADC_CHANNELS] = {0};

/* ================================================================
 * Funções auxiliares de hardware
 * ================================================================ */

static void adc_hw_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER |= 0xFFFFu;            /* PA0-PA7 analógico */
    GPIOB->MODER |= (3u << 0);          /* PB0 analógico     */

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0;
    ADC1->SMPR2 = 0x3FFFFFFFu;          /* tempo de amostragem longo */
    ADC1->SMPR1 = 0x0000003Fu;
    ADC1->CR2  |= ADC_CR2_ADON;
}

static uint16_t adc_read_ch(uint8_t ch)
{
    ADC1->SQR3  = ch;
    ADC1->CR2  |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

static void i2c_bus_recovery(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER  &= ~((3u<<(8*2)) | (3u<<(9*2)));
    GPIOB->MODER  |=  ((1u<<(8*2)) | (1u<<(9*2)));
    GPIOB->BSRR    =  (1u<<8) | (1u<<9);
    for (int i = 0; i < 9; i++) {
        GPIOB->BSRR = (1u<<(8+16)); delay_ms(1);
        GPIOB->BSRR = (1u<<8);      delay_ms(1);
    }
}

/* ================================================================
 * Funções do heatmap (usadas apenas por vTaskLCD)
 * ================================================================ */

static uint16_t get_heatmap_gradient(uint32_t hits, uint32_t max_hits)
{
    if (hits == 0) return C_BLACK;

    uint32_t p = (max_hits > 0) ? (hits * 100u) / max_hits : 0u;

    if (p >= 90 && hits >= MIN_HITS_RED)    return C_RED;
    if (p >= 75 && hits >= MIN_HITS_ORANGE) return 0xFD20u; /* laranja */
    if (p >= 60 && hits >= MIN_HITS_YELLOW) return C_YELL;
    if (p >= 40 && hits >= MIN_HITS_GREEN)  return C_GREEN;
    if (p >= 20 && hits >= MIN_HITS_CYAN)   return C_CYAN;
    return C_BLUE;
}

static void draw_zone(int index, const uint16_t heatmap[ADC_CHANNELS],
                      uint32_t max_hits)
{
    int row = index / GRID_SIZE;
    int col = index % GRID_SIZE;
    uint16_t color = get_heatmap_gradient(heatmap[index], max_hits);

    st7789_fill_rect_dma(col * CELL_DIM, row * CELL_DIM,
                         CELL_DIM, CELL_DIM, color);
    st7789_draw_rect(col * CELL_DIM, row * CELL_DIM,
                     CELL_DIM, CELL_DIM, C_WHITE);

    char buf[16];
    snprintf(buf, sizeof(buf), "%u", (unsigned)heatmap[index]);
    st7789_draw_text_5x7(col * CELL_DIM + 30, row * CELL_DIM + 35,
                         buf, C_WHITE, 2, 1, color);
}

/* ================================================================
 * Task 1 — Leitura IMU (MPU-6050)
 * Prioridade: 2  |  Período: 50 ms
 * ================================================================ */
static void vTaskIMU(void *arg)
{
    (void)arg;

    mpu6050_raw_t ref, imu;
    mpu6050_read_all(&ref);

    float yaw = 0.0f;
    const float DT = 0.050f;

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));

        if (mpu6050_read_all(&imu) != 0) continue;

        imu.ax -= ref.ax;
        imu.ay -= ref.ay;

        mpu6050_orientation_t ori = mpu6050_orientation_update(&imu, yaw, DT);
        yaw = ori.yaw_deg;

        imu_data_t pkt;
        pkt.timestamp_ms = xTaskGetTickCount();
        pkt.ax_mg    = (int16_t)(mpu6050_accel_g(imu.ax) * 1000.0f);
        pkt.ay_mg    = (int16_t)(mpu6050_accel_g(imu.ay) * 1000.0f);
        pkt.az_mg    = (int16_t)(mpu6050_accel_g(imu.az) * 1000.0f);
        pkt.roll_deg  = (int16_t)ori.roll_deg;
        pkt.pitch_deg = (int16_t)ori.pitch_deg;
        pkt.yaw_deg   = (int16_t)ori.yaw_deg;

        xQueueSendToBack(xQueueIMU, &pkt, 0);
    }
}

/* ================================================================
 * Task 2 — Varredura ADC e detecção de impacto
 * Prioridade: 3 (mais alta)  |  Período: 5 ms
 *
 * Publica hit_data_t em DUAS filas:
 *   xQueueHit    → consumida por vTaskTX  (transmissão)
 *   xQueueHitLCD → consumida por vTaskLCD (display)
 * ================================================================ */
static void vTaskADC(void *arg)
{
    (void)arg;

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(5));

        uint16_t peak     = 0;
        uint8_t  peak_ch  = 0;

        for (uint8_t i = 0; i < ADC_CHANNELS; i++) {
            uint16_t v = adc_read_ch(adc_ch[i]);
            if (v > peak) { peak = v; peak_ch = i; }
        }

        if (peak >= ADC_IMPACT_THRESHOLD)
        {
            uint32_t now_ms = xTaskGetTickCount(); /* 1 tick = 1 ms */

            /* Debounce por canal */
            if ((now_ms - g_last_hit_ms[peak_ch]) >= DEBOUNCE_MS)
            {
                g_last_hit_ms[peak_ch] = now_ms;
                g_heatmap[peak_ch]++;

                hit_data_t pkt;
                pkt.timestamp_ms = now_ms;
                pkt.region       = peak_ch;
                pkt.peak_raw     = peak;
                memcpy(pkt.heatmap, g_heatmap, sizeof(g_heatmap));

                /* Publica nas duas filas — sem bloquear */
                xQueueSendToBack(xQueueHit,    &pkt, 0);
                xQueueSendToBack(xQueueHitLCD, &pkt, 0);
            }
        }
    }
}

/* ================================================================
 * Task 3 — Transmissão serial (HC-12)
 * Prioridade: 1  |  Bloqueante nas filas
 * ================================================================ */
static void vTaskTX(void *arg)
{
    (void)arg;
    static char buf[256];

    for (;;)
    {
        imu_data_t imu_pkt;
        while (xQueueReceive(xQueueIMU, &imu_pkt, 0) == pdTRUE)
        {
            snprintf(buf, sizeof(buf),
                "$RAQ,%s,%lu,%d,%d,%d,%d,%d,%d\r\n",
                RACKET_ID,
                (unsigned long)imu_pkt.timestamp_ms,
                (int)imu_pkt.yaw_deg,
                (int)imu_pkt.roll_deg,
                (int)imu_pkt.pitch_deg,
                (int)imu_pkt.ax_mg,
                (int)imu_pkt.ay_mg,
                (int)imu_pkt.az_mg);

            xSemaphoreTake(xUartMutex, portMAX_DELAY);
            serial_write(buf);
            xSemaphoreGive(xUartMutex);
        }

        hit_data_t hit_pkt;
        while (xQueueReceive(xQueueHit, &hit_pkt, 0) == pdTRUE)
        {
            snprintf(buf, sizeof(buf),
                "$HIT,%s,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                RACKET_ID,
                (unsigned long)hit_pkt.timestamp_ms,
                (unsigned)hit_pkt.region,
                (unsigned)hit_pkt.peak_raw,
                (unsigned)hit_pkt.heatmap[0], (unsigned)hit_pkt.heatmap[1],
                (unsigned)hit_pkt.heatmap[2], (unsigned)hit_pkt.heatmap[3],
                (unsigned)hit_pkt.heatmap[4], (unsigned)hit_pkt.heatmap[5],
                (unsigned)hit_pkt.heatmap[6], (unsigned)hit_pkt.heatmap[7],
                (unsigned)hit_pkt.heatmap[8]);

            xSemaphoreTake(xUartMutex, portMAX_DELAY);
            serial_write(buf);
            xSemaphoreGive(xUartMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================================================================
 * Task 4 — Heatmap no display ST7789
 * Prioridade: 1  |  Bloqueia em xQueueHitLCD
 *
 * Aguarda indefinidamente um hit_data_t. Quando recebe, redesenha
 * apenas as células necessárias (lógica idêntica ao original):
 *   - Se o máximo global subiu → redesenha todas as 9 células
 *   - Caso contrário           → redesenha só a célula atingida
 * ================================================================ */
static void vTaskLCD(void *arg)
{
    (void)arg;

    uint32_t current_max = 0u;

    /* Desenha grade inicial vazia */
    uint16_t empty[ADC_CHANNELS] = {0};
    for (int i = 0; i < (int)ADC_CHANNELS; i++)
        draw_zone(i, empty, 0);

    for (;;)
    {
        hit_data_t pkt;
        /* Bloqueia até chegar um hit — não consome CPU enquanto espera */
        if (xQueueReceive(xQueueHitLCD, &pkt, portMAX_DELAY) != pdTRUE)
            continue;

        uint32_t prev_max = current_max;

        /* Atualiza máximo global a partir do heatmap recebido */
        for (uint8_t i = 0; i < ADC_CHANNELS; i++) {
            if (pkt.heatmap[i] > current_max)
                current_max = pkt.heatmap[i];
        }

        if (current_max > prev_max) {
            /* Proporção de todas as células mudou — redesenha tudo */
            for (int i = 0; i < (int)ADC_CHANNELS; i++)
                draw_zone(i, pkt.heatmap, current_max);
        } else {
            /* Só a célula atingida mudou */
            draw_zone(pkt.region, pkt.heatmap, current_max);
        }
    }
}

/* ================================================================
 * main
 * ================================================================ */
int main(void)
{
    SystemCoreClockUpdate();
    delay_init();

    serial_stdio_init(115200);
    printf("Raquete FreeRTOS iniciando...\r\n");

    /* I2C */
    i2c_bus_recovery();
    i2c1_init_100k(16000000u);
    delay_ms(100);

    if (mpu6050_init() < 0) {
        serial_write("ERRO: MPU6050 init falhou\r\n");
        for (;;);
    }

    /* ADC */
    adc_hw_init();

    /* LCD ST7789 */
    st7789_init();
    st7789_set_speed_div(0);        /* SPI na velocidade máxima */
    st7789_fill_screen(C_BLACK);

    /* Filas */
    xQueueIMU    = xQueueCreate(4, sizeof(imu_data_t));
    xQueueHit    = xQueueCreate(8, sizeof(hit_data_t));
    xQueueHitLCD = xQueueCreate(8, sizeof(hit_data_t));

    /* Mutex UART */
    xUartMutex = xSemaphoreCreateMutex();

    /* Tasks */
    xTaskCreate(vTaskIMU, "IMU",  512, NULL, 2, NULL);
    xTaskCreate(vTaskADC, "ADC",  256, NULL, 3, NULL);
    xTaskCreate(vTaskTX,  "TX",   512, NULL, 1, NULL);
    xTaskCreate(vTaskLCD, "LCD",  768, NULL, 1, NULL);

    printf("Scheduler iniciando.\r\n");
    vTaskStartScheduler();

    while (1) { __NOP(); }
}