#ifndef RACKET_TYPES_H
#define RACKET_TYPES_H

#include <stdint.h>

/* Payload publicado pela task IMU */
typedef struct {
    uint32_t timestamp_ms;
    int16_t  ax_mg, ay_mg, az_mg;   // aceleração em mg
    int16_t  roll_deg, pitch_deg, yaw_deg;
} imu_data_t;

/* Payload publicado pela task ADC ao detectar impacto */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  region;          // 0-8 (célula 3×3 com maior leitura)
    uint16_t peak_raw;        // valor ADC do pico
    uint16_t heatmap[9];      // contagem acumulada por região
} hit_data_t;

#endif