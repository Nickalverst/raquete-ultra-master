/*
 * raquete_freertos — main.c
 *
 * Task 1: vTaskIMU  — leitura MPU-6050  (prio 2, 50 ms)
 * Task 2: vTaskADC  — varredura piezo   (prio 3, 5 ms)
 * Task 3: vTaskTX   — transmissão HC-12 (prio 1, bloqueante)
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
#include "racket_types.h"

/* ─── Identificação da raquete ───────────────────────────── */
#define RACKET_ID   "RAQ01"

/* ─── Thresholds ─────────────────────────────────────────── */
#define ADC_IMPACT_THRESHOLD   500u   // ajuste conforme seus sensores
#define ADC_CHANNELS           9u

/* ─── Filas FreeRTOS ─────────────────────────────────────── */
static QueueHandle_t xQueueIMU;   // imu_data_t, profundidade 4
static QueueHandle_t xQueueHit;   // hit_data_t, profundidade 8

/* ─── Mutex para acesso à UART (printf) ──────────────────── */
static SemaphoreHandle_t xUartMutex;

/* ─── Mapa de canais ADC para os 9 piezos ───────────────── */
// PA0-PA7 → CH0-CH7, PB0 → CH8
static const uint8_t adc_ch[ADC_CHANNELS] = {0,1,2,3,4,5,6,7,8};

/* ─── Heatmap global (atualizado apenas por vTaskADC) ───── */
static uint16_t g_heatmap[ADC_CHANNELS] = {0};

/* ================================================================
 * Inicializações de hardware
 * ================================================================ */

static void adc_hw_init(void)
{
    /* Clocks de GPIO */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;

    /* PA0-PA7 analógico */
    GPIOA->MODER |= 0xFFFFu;         // todos pinos 0..7 → modo analógico

    /* PB0 analógico */
    GPIOB->MODER |= (3u << 0);

    /* Clock e configuração do ADC1 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0;
    ADC1->SMPR2 = 0x3FFFFFFFu;   // tempo de amostragem longo (piezo = alta impedância)
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
    /* Pulsos manuais em PB8 para desengripar o barramento */
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
 * Task 1 — Leitura IMU (MPU-6050)
 * Prioridade: 2  |  Período: 50 ms
 * ================================================================ */
static void vTaskIMU(void *arg)
{
    (void)arg;

    mpu6050_raw_t ref, imu;
    mpu6050_read_all(&ref);   // captura offset inicial

    float yaw = 0.0f;
    const float DT = 0.050f; // 50 ms

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(50));

        if (mpu6050_read_all(&imu) != 0) continue;

        /* Remove offset de aceleração */
        imu.ax -= ref.ax;
        imu.ay -= ref.ay;

        /* Calcula orientação */
        mpu6050_orientation_t ori = mpu6050_orientation_update(&imu, yaw, DT);
        yaw = ori.yaw_deg;

        /* Monta payload */
        imu_data_t pkt;
        pkt.timestamp_ms = xTaskGetTickCount();   // ms desde boot
        pkt.ax_mg   = (int16_t)(mpu6050_accel_g(imu.ax) * 1000.0f);
        pkt.ay_mg   = (int16_t)(mpu6050_accel_g(imu.ay) * 1000.0f);
        pkt.az_mg   = (int16_t)(mpu6050_accel_g(imu.az) * 1000.0f);
        pkt.roll_deg  = (int16_t)ori.roll_deg;
        pkt.pitch_deg = (int16_t)ori.pitch_deg;
        pkt.yaw_deg   = (int16_t)ori.yaw_deg;

        /* Publica na fila — não bloqueia se estiver cheia */
        xQueueSendToBack(xQueueIMU, &pkt, 0);
    }
}

/* ================================================================
 * Task 2 — Varredura ADC e detecção de impacto
 * Prioridade: 3 (mais alta)  |  Período: 5 ms
 * ================================================================ */
static void vTaskADC(void *arg)
{
    (void)arg;

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(5));

        uint16_t readings[ADC_CHANNELS];
        uint16_t peak = 0;
        uint8_t  peak_ch = 0;

        /* Varre todos os 9 canais */
        for (uint8_t i = 0; i < ADC_CHANNELS; i++) {
            readings[i] = adc_read_ch(adc_ch[i]);
            if (readings[i] > peak) {
                peak = readings[i];
                peak_ch = i;
            }
        }

        /* Detecta impacto apenas se ultrapassar threshold */
        if (peak >= ADC_IMPACT_THRESHOLD) {
            g_heatmap[peak_ch]++;

            hit_data_t pkt;
            pkt.timestamp_ms = xTaskGetTickCount();
            pkt.region   = peak_ch;
            pkt.peak_raw = peak;
            memcpy(pkt.heatmap, g_heatmap, sizeof(g_heatmap));

            /* Publica na fila — não bloqueia */
            xQueueSendToBack(xQueueHit, &pkt, 0);
        }
    }
}

/* ================================================================
 * Task 3 — Transmissão serial (HC-12 / USB-Serial)
 * Prioridade: 1 (mais baixa)  |  Bloqueante nas filas
 * ================================================================ */
static void vTaskTX(void *arg)
{
    (void)arg;

    static char buf[256];

    for (;;)
    {
        /* ── Drena fila IMU ── */
        imu_data_t imu_pkt;
        while (xQueueReceive(xQueueIMU, &imu_pkt, 0) == pdTRUE)
        {
            /* Formato: $RAQ,<id>,<ts>,<yaw>,<roll>,<pitch>,<ax>,<ay>,<az> */
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

        /* ── Drena fila de impactos ── */
        hit_data_t hit_pkt;
        while (xQueueReceive(xQueueHit, &hit_pkt, 0) == pdTRUE)
        {
            /* Formato: $HIT,<id>,<ts>,<region>,<peak>,h0..h8 */
            snprintf(buf, sizeof(buf),
                "$HIT,%s,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                RACKET_ID,
                (unsigned long)hit_pkt.timestamp_ms,
                (unsigned)hit_pkt.region,
                (unsigned)hit_pkt.peak_raw,
                (unsigned)hit_pkt.heatmap[0],
                (unsigned)hit_pkt.heatmap[1],
                (unsigned)hit_pkt.heatmap[2],
                (unsigned)hit_pkt.heatmap[3],
                (unsigned)hit_pkt.heatmap[4],
                (unsigned)hit_pkt.heatmap[5],
                (unsigned)hit_pkt.heatmap[6],
                (unsigned)hit_pkt.heatmap[7],
                (unsigned)hit_pkt.heatmap[8]);

            xSemaphoreTake(xUartMutex, portMAX_DELAY);
            serial_write(buf);
            xSemaphoreGive(xUartMutex);
        }

        /* Cede o processador quando não há dados pendentes */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================================================================
 * main — inicializa hardware, cria filas/tarefas, inicia scheduler
 * ================================================================ */
int main(void)
{
    SystemCoreClockUpdate();
    delay_init();

    /* Serial para depuração e HC-12 */
    serial_stdio_init(115200);
    printf("Estou começando.");

    /* Recuperação do barramento I2C */
    i2c_bus_recovery();
    i2c1_init_100k(16000000u);
    delay_ms(100);

    

    if (mpu6050_init() < 0) {
        serial_write("MPU6050 init FALHOU\r\n");
        for (;;);
    }

    /* ADC */
    adc_hw_init();

    /* ── Filas FreeRTOS ── */
    xQueueIMU = xQueueCreate(4, sizeof(imu_data_t));
    xQueueHit = xQueueCreate(8, sizeof(hit_data_t));

    /* ── Mutex UART ── */
    xUartMutex = xSemaphoreCreateMutex();

    /* ── Tasks ── */
    xTaskCreate(vTaskIMU, "IMU",  512, NULL, 2, NULL);
    xTaskCreate(vTaskADC, "ADC",  256, NULL, 3, NULL);
    xTaskCreate(vTaskTX,  "TX",   512, NULL, 1, NULL);

    /* Inicia o escalonador — não retorna */
    vTaskStartScheduler();

    while (1) { __NOP(); }
}