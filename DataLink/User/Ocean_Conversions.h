#ifndef OCEAN_CONVERSIONS_H
#define OCEAN_CONVERSIONS_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Ocean device fixed-point conversion helpers
 * Phase-1: restructure only (no behavior change)
 * -------------------------------------------------------------------------- */

/* Q14.2 (U16) → volts
 * Used for channel/output voltage fields in the measurement block.
 * raw / 4.0f
 */
static inline float ocean_q14_2_to_volt(uint16_t raw)
{
    return (float)raw / 4.0f;
}

/* Q9.7 (U16) → amps
 * Used for per-channel current fields in the measurement block.
 * raw / 128.0f  */
static inline float ocean_q9_7_to_amp(uint16_t raw)
{
    return (float)raw / 128.0f;
}

/* Q2.6 (U16) → watts
 * Used for read-only report at 0x000A (Channel power report U16).
 * raw / 64.0f */
static inline float ocean_q2_6_to_watts_u16(uint16_t raw)
{
    return (float)raw / 64.0f;
}

/* --------------------------------------------------------------------------
 * Optional symmetry for writable setpoint at 0x8108 (U8 Q2.6)
 * Enable when you want to use function forms at call sites.
 * -------------------------------------------------------------------------- */

/* Q2.6 (U8) → watts: raw / 64.0f */
static inline float ocean_q2_6_to_watts_u8(uint8_t raw)
{
    return (float)raw / 64.0f;
}

/* watts → Q2.6 (U8) with clamping to [0, 255]
 * Encodes a floating-point watt value into the device's 1-byte Q2.6 format. */
static inline uint8_t ocean_watts_to_q2_6_u8(float watts)
{
    if (watts < 0.0f) watts = 0.0f;
    float q = watts * 64.0f + 0.5f;    /* round-to-nearest */
    if (q > 255.0f) q = 255.0f;
    return (uint8_t)q;
}

#endif /* OCEAN_CONVERSIONS_H */
