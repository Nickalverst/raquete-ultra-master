#pragma once
/*
 * flatbuf.h — hand-written FlatBuffers builders for the racket firmware.
 *
 * No dynamic allocation, no codegen dependency.
 * Each function writes a complete, valid FlatBuffer into a caller-supplied
 * stack buffer.  Layouts are fixed (all fields always present, heatmap
 * always 9 elements), so buffer sizes are compile-time constants.
 *
 * Wire framing (prepend before transmitting each buffer):
 *   [type : uint8][len_lo : uint8][len_hi : uint8]
 *
 * Schema: racket.fbs
 */

#include <stdint.h>

/* ── type tags ────────────────────────────────────────────────────────────── */
#define FB_TYPE_IMU   0x01u
#define FB_TYPE_HIT   0x02u

/* ── exact payload sizes (see flatbuf.c for layout diagrams) ─────────────── */
#define FB_IMU_SIZE   42
#define FB_HIT_SIZE   54

/*
 * fb_build_imu()
 *   Fills buf[FB_IMU_SIZE] with a valid ImuPacket FlatBuffer.
 */
void fb_build_imu(uint8_t buf[FB_IMU_SIZE],
                  uint32_t timestamp_ms,
                  int16_t  yaw_deg,
                  int16_t  roll_deg,
                  int16_t  pitch_deg,
                  int16_t  ax_mg,
                  int16_t  ay_mg,
                  int16_t  az_mg);

/*
 * fb_build_hit()
 *   Fills buf[FB_HIT_SIZE] with a valid HitPacket FlatBuffer.
 *   heatmap must point to exactly 9 uint16_t values.
 */
void fb_build_hit(uint8_t        buf[FB_HIT_SIZE],
                  uint32_t       timestamp_ms,
                  uint8_t        region,
                  uint16_t       peak_raw,
                  const uint16_t heatmap[9]);
