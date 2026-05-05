#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR       0x68u   // AD0=GND
#define MPU6050_REG_WHOAMI 0x75u
#define MPU6050_REG_PWR1   0x6Bu
#define MPU6050_REG_ACCEL  0x3Bu
#define MPU6050_REG_GYRO   0x43u
#define MPU6050_REG_SMPLRT 0x19u
#define MPU6050_REG_CONFIG 0x1Au
#define MPU6050_REG_GYROCFG  0x1Bu
#define MPU6050_REG_ACCELCFG 0x1Cu

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;
} mpu6050_raw_t;

void i2c1_init_100k(uint32_t apb1_hz);
int  i2c1_write_reg(uint8_t addr7, uint8_t reg, uint8_t data);
int  i2c1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *data);
int  i2c1_read_multi(uint8_t addr7, uint8_t reg, uint8_t *buf, uint32_t len);

int  mpu6050_init(void);
int  mpu6050_read_all(mpu6050_raw_t *out);

/* Conversões úteis (assumindo ±2g e ±250 dps) */
static inline float mpu6050_accel_g(int16_t raw) { return raw / 16384.0f; }
static inline float mpu6050_gyro_dps(int16_t raw) { return raw / 131.0f; }
static inline float mpu6050_temp_c(int16_t raw) { return (raw/340.0f) + 36.53f; }

#endif
