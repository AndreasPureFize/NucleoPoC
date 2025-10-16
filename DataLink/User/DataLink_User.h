#ifndef DATALINK_USER_H
#define DATALINK_USER_H

#include <stdint.h>
#include <stdbool.h>

// Public commands
/* Configuration */
bool SetChannels(uint8_t channels);			/* Writes NUM_CHANNELS @ 0x8109, valid: 1..4 */
bool ReadChannels(uint8_t *out_channels);	/* Reads from 0x0008 block */

/* Power */
bool SetPower(float watts);                 /* valid: 0.5 .. 1.0 */
bool ChangePower (float watts);				/* valid: 0.5 .. 1.0 */
bool ReadPower(float *out_value);

/* Output and default states */
bool WriteOutputState(uint8_t state);       /* valid: 0 or 1 */
bool ReadOutputState(uint8_t *out_state);

bool WriteDefaultState(uint8_t state);      /* valid: 0 or 1 */
bool ReadDefaultState(uint8_t *out_state);

/* Errors */
bool ReadErrorflag(uint32_t *out_flags);
bool ResetError(void);

/* Data acquisition */
bool ReadData(void);


// Internal commands & functions
void read_and_print_serial(void);
void read_and_print_accum_on_time(void);
void read_one_time_blocks(void);
void poll_periodic(void);
bool TestSequense(void);
bool RampPower(void);

/* Exposed symbols for main.c (unchanged) */
extern bool g_config_loaded;
void print_line(const char* s);

#endif /* DATALINK_USER_H */


