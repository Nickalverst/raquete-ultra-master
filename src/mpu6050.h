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
#include <math.h>

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} mpu6050_orientation_t;

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
static inline mpu6050_orientation_t mpu6050_orientation_update(
    mpu6050_raw_t *r,
    float yaw_prev,
    float dt_s)          // tempo em segundos desde a última chamada
{
    float ax = mpu6050_accel_g(r->ax);
    float ay = mpu6050_accel_g(r->ay);
    float az = mpu6050_accel_g(r->az);
    float gz = mpu6050_gyro_dps(r->gz);  // graus/s no eixo Z

    // Zona morta: ignora ruído quando quase parado  <-- AQUI
    if (gz > -1.0f && gz < 1.0f) gz = 0.0f;

    mpu6050_orientation_t o;

    // Roll e pitch via acelerômetro (absolutos, sem drift)
    o.roll_deg  = atan2f(ay, az)                    * (180.0f / 3.14159265f);
    o.pitch_deg = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f / 3.14159265f);

    // Yaw via integração do giroscópio (relativo, acumula drift)
    o.yaw_deg = yaw_prev + gz * dt_s;

    // Mantém no intervalo [-180, 180]
    if (o.yaw_deg >  180.0f) o.yaw_deg -= 360.0f;
    if (o.yaw_deg < -180.0f) o.yaw_deg += 360.0f;

    return o;
}

#endif
