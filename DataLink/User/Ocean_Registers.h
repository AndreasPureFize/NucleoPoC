#ifndef USER_OCEAN_REGISTERS_H_
#define USER_OCEAN_REGISTERS_H_

/* Ocean device register map — moved from main.c (Phase-1, no behavior change) */
/* Device Info & Status snapshots */
#ifndef OCEAN_ADDR_DEVICE_INFO
#define OCEAN_ADDR_DEVICE_INFO       0x0008u   /* ActiveCh(U8), ChPower(Q2.6 U16), FW(U32), PID(U32) */
#endif
#ifndef OCEAN_LEN_DEVICE_INFO
#define OCEAN_LEN_DEVICE_INFO        12u
#endif

#ifndef OCEAN_ADDR_STATUS
#define OCEAN_ADDR_STATUS            0x0000u   /* Status(U32) + ErrorFlags(U32) */
#endif
#ifndef OCEAN_LEN_STATUS
#define OCEAN_LEN_STATUS             8u
#endif

/* Measurement window used by the PoC */
#ifndef OCEAN_ADDR_MEAS_BLOCK
#define OCEAN_ADDR_MEAS_BLOCK        0x0100u   /* We read a 42-byte span and pick fields by absolute addr */
#endif
#ifndef OCEAN_LEN_MEAS_BLOCK
#define OCEAN_LEN_MEAS_BLOCK         42u
#endif

/* Statistics example used by the PoC */
#ifndef OCEAN_ADDR_TEMPERATURE_SUM
#define OCEAN_ADDR_TEMPERATURE_SUM   0x2130u   /* U32 */
#endif
#ifndef OCEAN_LEN_TEMPERATURE_SUM
#define OCEAN_LEN_TEMPERATURE_SUM    4u
#endif

/* User & Factory configuration snapshots (kept for completeness) */
#ifndef OCEAN_ADDR_USER_CONFIG
#define OCEAN_ADDR_USER_CONFIG       0x8000u
#endif
#ifndef OCEAN_LEN_USER_CONFIG
#define OCEAN_LEN_USER_CONFIG        0x36u
#endif

#ifndef OCEAN_ADDR_FACTORY_CONFIG
#define OCEAN_ADDR_FACTORY_CONFIG    0x8200u   /* Serial(U32) */
#endif
#ifndef OCEAN_LEN_FACTORY_CONFIG
#define OCEAN_LEN_FACTORY_CONFIG     4u
#endif

/* Protected region: UNLOCK + Channel Power Setpoint */
#ifndef OCEAN_ADDR_UNLOCK
#define OCEAN_ADDR_UNLOCK            0x8100u   /* write 8 bytes (two LE32 keys) */
#endif
#ifndef OCEAN_LEN_UNLOCK
#define OCEAN_LEN_UNLOCK             8u
#endif

#ifndef OCEAN_UNLOCK_KEY0
#define OCEAN_UNLOCK_KEY0            0xE1C85CDAu
#endif
#ifndef OCEAN_UNLOCK_KEY1
#define OCEAN_UNLOCK_KEY1            0x367F966Bu
#endif

#ifndef OCEAN_ADDR_CHANNEL_POWER_SET
#define OCEAN_ADDR_CHANNEL_POWER_SET 0x8108u   /* Q2.6 setpoint */
#endif
#ifndef OCEAN_LEN_CHANNEL_POWER_SET
#define OCEAN_LEN_CHANNEL_POWER_SET  2u        /* keep as-is from v0.1.2/v0.1.3 code */
#endif

/* --------------------------------------------------------------------------
 * Optional (used as literals in current functions; centralize now if you want)
 * Uncomment if/when you replace hard-coded literals in functions:
 * -------------------------------------------------------------------------- */
/*
#define OCEAN_ADDR_OUTPUT_STATE      0x800Cu   // U8
#define OCEAN_ADDR_NUM_CHANNELS      0x8109u   // U8
#define OCEAN_ADDR_ACCUM_ON_TIME     0x8032u   // U32
*/

#endif /* USER_OCEAN_REGISTERS_H_ */
