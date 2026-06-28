#include "flatbuf.h"

/* ── little-endian write helpers ─────────────────────────────────────────── */
static inline void w16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}
static inline void w32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline void wi16(uint8_t *p, int16_t v)  { w16(p, (uint16_t)v); }
static inline void wi32(uint8_t *p, int32_t v)  { w32(p, (uint32_t)v); }

/*
 * ImuPacket — 42 bytes, fixed layout:
 *
 *  Off  Sz   Content
 *  ───────────────────────────────────────────────────────
 *   0   4    root_offset = 22          (→ table at byte 22)
 *   4   2    vtable_size = 18          (4 + 2×7 fields)
 *   6   2    object_inline_size = 20   (4 soffset + 4 u32 + 6×2 i16)
 *   8   2    voff[0] timestamp_ms = 4
 *  10   2    voff[1] yaw_deg      = 8
 *  12   2    voff[2] roll_deg     = 10
 *  14   2    voff[3] pitch_deg    = 12
 *  16   2    voff[4] ax_mg        = 14
 *  18   2    voff[5] ay_mg        = 16
 *  20   2    voff[6] az_mg        = 18
 *  22   4    vtable_soffset = −18  (vtable@4, table@22 → 4−22=−18)
 *  26   4    timestamp_ms
 *  30   2    yaw_deg
 *  32   2    roll_deg
 *  34   2    pitch_deg
 *  36   2    ax_mg
 *  38   2    ay_mg
 *  40   2    az_mg
 */
void fb_build_imu(uint8_t buf[FB_IMU_SIZE],
                  uint32_t timestamp_ms,
                  int16_t  yaw_deg,
                  int16_t  roll_deg,
                  int16_t  pitch_deg,
                  int16_t  ax_mg,
                  int16_t  ay_mg,
                  int16_t  az_mg)
{
    /* root */
    w32(buf +  0, 22u);

    /* vtable */
    w16(buf +  4, 18u);     /* vtable_size        */
    w16(buf +  6, 20u);     /* object_inline_size */
    w16(buf +  8,  4u);     /* voff timestamp_ms  */
    w16(buf + 10,  8u);     /* voff yaw_deg       */
    w16(buf + 12, 10u);     /* voff roll_deg      */
    w16(buf + 14, 12u);     /* voff pitch_deg     */
    w16(buf + 16, 14u);     /* voff ax_mg         */
    w16(buf + 18, 16u);     /* voff ay_mg         */
    w16(buf + 20, 18u);     /* voff az_mg         */

    /* table */
    wi32(buf + 22, -18);    /* vtable_soffset     */
    w32 (buf + 26, timestamp_ms);
    wi16(buf + 30, yaw_deg);
    wi16(buf + 32, roll_deg);
    wi16(buf + 34, pitch_deg);
    wi16(buf + 36, ax_mg);
    wi16(buf + 38, ay_mg);
    wi16(buf + 40, az_mg);
}

/*
 * HitPacket — 54 bytes, fixed layout:
 *
 *  Off  Sz   Content
 *  ───────────────────────────────────────────────────────
 *   0   4    root_offset = 16          (→ table at byte 16)
 *   4   2    vtable_size = 12          (4 + 2×4 fields)
 *   6   2    object_inline_size = 16
 *   8   2    voff[0] timestamp_ms = 8
 *  10   2    voff[1] region       = 14
 *  12   2    voff[2] peak_raw     = 12
 *  14   2    voff[3] heatmap      = 4   (UOffset field)
 *  16   4    vtable_soffset = −12  (vtable@4, table@16 → 4−16=−12)
 *  20   4    heatmap UOffset = 12   (vector@32, field@20 → 32−20=12)
 *  24   4    timestamp_ms
 *  28   2    peak_raw
 *  30   1    region
 *  31   1    padding
 *  32   4    vector count = 9
 *  36  18    heatmap[0..8]  (9 × uint16)
 */
void fb_build_hit(uint8_t        buf[FB_HIT_SIZE],
                  uint32_t       timestamp_ms,
                  uint8_t        region,
                  uint16_t       peak_raw,
                  const uint16_t heatmap[9])
{
    int i;

    /* root */
    w32(buf +  0, 16u);

    /* vtable */
    w16(buf +  4, 12u);     /* vtable_size        */
    w16(buf +  6, 16u);     /* object_inline_size */
    w16(buf +  8,  8u);     /* voff timestamp_ms  */
    w16(buf + 10, 14u);     /* voff region        */
    w16(buf + 12, 12u);     /* voff peak_raw      */
    w16(buf + 14,  4u);     /* voff heatmap       */

    /* table */
    wi32(buf + 16, -12);    /* vtable_soffset     */
    w32 (buf + 20, 12u);    /* heatmap UOffset (32−20=12) */
    w32 (buf + 24, timestamp_ms);
    w16 (buf + 28, peak_raw);
    buf[30] = region;
    buf[31] = 0u;           /* padding            */

    /* heatmap vector */
    w32(buf + 32, 9u);
    for (i = 0; i < 9; i++)
        w16(buf + 36 + i * 2, heatmap[i]);
}
