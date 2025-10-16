#include "DataLink_User.h"
#include "DataLink_Driver.h"   /* dl_read_retry / dl_write_retry, dl_status_t */
#include "DataLink_HAL.h"
#include "Ocean_Registers.h"   /* OCEAN_* addresses/lengths/keys */
#include "Ocean_Conversions.h" /* ocean_q14_2_to_volt / _q9_7_to_amp / _q2_6_to_watts_u16 */
#include "main.h"              /* HAL_GetTick / HAL_Delay */
#include "usart.h"             /* extern huart2 for console prints (demo functions) */
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================================
 * File-scope cached values (demo usage)
 * ==========================================================================*/
static uint8_t  g_active_channels = 0;
static float    g_channel_power   = 0.0f;
static uint32_t g_firmware_version = 0;
static uint32_t g_product_id       = 0;
/* static uint32_t g_serial_number = 0; */
/* static uint16_t g_statistics_samples = 0; */

bool g_config_loaded = false;

/* ============================================================================
 * Measurement selection table (addresses and printing names) - demo only
 * ==========================================================================*/
typedef enum { QFMT_Q14_2, QFMT_Q9_7 } qfmt_t;
typedef struct { uint16_t addr; const char* name; qfmt_t qfmt; } meas_sel_t;
static const meas_sel_t k_meas_sel[] =
{
    { 0x0118, "Channel 4 voltage", QFMT_Q14_2 },
    { 0x011A, "Channel 4 current", QFMT_Q9_7 },
    { 0x011C, "Channel 3 voltage", QFMT_Q14_2 },
    { 0x011E, "Channel 3 current", QFMT_Q9_7 },
    { 0x0120, "Channel 2 voltage", QFMT_Q14_2 },
    { 0x0122, "Channel 2 current", QFMT_Q9_7 },
    { 0x0124, "Channel 1 voltage", QFMT_Q14_2 },
    { 0x0126, "Channel 1 current", QFMT_Q9_7 },
    { 0x0128, "Output voltage",    QFMT_Q14_2 },
};

/* Writes a 32-bit little-endian value 'v' into a byte buffer 'p' (4 bytes). */
static void le32_to_buf(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8)  & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* Issues an UNLOCK to the protected region at 0x810x (two LE32 keys at 0x8100).
   Returns true if the write succeeds. */
static bool ocean_unlock(void)
{
    uint8_t payload[OCEAN_LEN_UNLOCK];
    le32_to_buf(&payload[0], OCEAN_UNLOCK_KEY0);
    le32_to_buf(&payload[4], OCEAN_UNLOCK_KEY1);

    if (dl_write_retry(OCEAN_ADDR_UNLOCK, OCEAN_LEN_UNLOCK, payload) == DL_OK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Channel configuration

static bool read_channels_quick(uint8_t *out, uint32_t budget_ms)
{
    if (out == NULL) return false;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < budget_ms)
    {
        uint8_t b[12]; // Device Info block
        // Short, non-retry read (50/80 ms): avoids long backoffs
        dl_status_t st = dl_read(/*addr*/0x0008u, /*len*/12u, b,
                                 /*hdr*/50u, /*pay*/80u);
        if (st == DL_OK) { *out = b[0]; return true; }
        HAL_Delay(20u);
    }
    return false;
}

bool SetChannels(uint8_t nc)
{
    // --- Early exit if already set (fast check; ~<100 ms) ---
    uint8_t current = 0xFFu;
    if (read_channels_quick(&current, /*budget_ms*/120u) && current == nc) {
        return true;
    }

    // --- Send write with short waits; do NOT use *_retry here ---
    (void)dl_write(/*addr*/0x8109u, /*len*/1u, &nc, /*hdr*/30u, /*pay*/30u);

    // Device reconfigures; give it a brief settle
    HAL_Delay(120u);

    // --- Short polling (no handshake) for up to ~400 ms ---
    uint8_t reported = 0xFFu;
    if (read_channels_quick(&reported, /*budget_ms*/400u) && reported == nc) {
        return true; // fast path, typically 300–500 ms overall
    }

    // --- Quick handshake (fast re-sync) ---
    (void)dl_handshake_quick(/*ans_attempts*/3, /*host_attempts*/3,
                             /*ans_gap_ms*/25u, /*host_gap_ms*/80u);

    // Verify again (short poll)
    reported = 0xFFu;
    if (read_channels_quick(&reported, /*budget_ms*/300u) && reported == nc) {
        return true;
    }

    // --- One UNLOCK + one more short write, quick re-sync, verify ---
    (void)ocean_unlock();
    (void)dl_write(0x8109u, 1u, &nc, 30u, 30u);
    HAL_Delay(120u);
    (void)dl_handshake_quick(3, 3, 25u, 80u);

    reported = 0xFFu;
    if (read_channels_quick(&reported, 300u) && reported == nc) {
        return true;
    }

    // --- Ultimate fallback: full handshake + your existing ReadChannels() ---
    (void)dl_handshake(); // current full logic
    return ReadChannels(&reported) && (reported == nc);
}

/* Reads Active channels (U8) from Device Info block @ 0x0008 */
bool ReadChannels(uint8_t *num_channels)
{
    uint8_t b[12];

    if (num_channels == NULL)
    {
        return false;
    }

    if (dl_read_retry(0x0008u, 12u, b) != DL_OK)
    {
        return false;
    }

    *num_channels = b[0]; /* Active channels */
    return true;
}

// Power configuration
// --- helpers ---------------------------------------------------------------

static uint8_t encode_q26_u8(float pow)
{
    if (pow < 0.0f)  pow = 0.0f;
    float scaled = pow * 64.0f;
    if (scaled > 255.0f) scaled = 255.0f;
    return (uint8_t)(scaled + 0.5f);
}

static bool read_setpoint_quick(uint8_t *out, uint32_t budget_ms)
{
    if (!out) return false;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < budget_ms) {
        uint8_t rb = 0xFF;
        dl_status_t st = dl_read(/*0x8108*/0x8108u, /*len*/1u, &rb,
                                 /*hdr*/40u, /*pay*/60u);
        if (st == DL_OK) { *out = rb; return true; }
        HAL_Delay(20u);
    }
    return false;
}

static bool read_power_quick(uint16_t *out_q26, uint32_t budget_ms)
{
    if (!out_q26) return false;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < budget_ms) {
        uint8_t b[2] = {0};
        dl_status_t st = dl_read(/*0x000A*/0x000Au, /*len*/2u, b,
                                 /*hdr*/40u, /*pay*/60u);
        if (st == DL_OK) {
            *out_q26 = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
            return true;
        }
        HAL_Delay(20u);
    }
    return false;
}

bool SetPower(float pow)
{
    const uint8_t q26 = encode_q26_u8(pow);

    // --- Early exit if already set (quick) ---
    uint8_t sp = 0xFF;
    if (read_setpoint_quick(&sp, /*budget_ms*/120u) && sp == q26) {
        return true;
    }

    // --- Fire write with short waits (no retry) ---
    (void)dl_write(/*addr*/0x8108u, /*len*/1u, &q26, /*hdr*/30u, /*pay*/30u);
    HAL_Delay(120u);                  // device may reconfigure briefly
    (void)dl_handshake_quick(3,3,25u,80u);  // quick re-sync

    // --- Verify (prefer setpoint; optionally confirm reported power) ---
    sp = 0xFF;
    if (read_setpoint_quick(&sp, /*budget_ms*/300u) && sp == q26) {
        return true;
    }
    // Optional second check: reported power within ±1 LSB of target
    {
        uint16_t rp_q26 = 0;
        if (read_power_quick(&rp_q26, /*budget_ms*/200u)) {
            uint16_t tgt_q26_u16 = (uint16_t)q26;
            if ((rp_q26 == tgt_q26_u16) ||                       // exact
                (rp_q26 + 1 == tgt_q26_u16) || (rp_q26 == tgt_q26_u16 + 1)) {
                return true;
            }
        }
    }

    // --- One UNLOCK + one more short write, quick re-sync, verify again ---
    (void)ocean_unlock();
    (void)dl_write(0x8108u, 1u, &q26, 30u, 30u);
    HAL_Delay(120u);
    (void)dl_handshake_quick(3,3,25u,80u);

    sp = 0xFF;
    if (read_setpoint_quick(&sp, 300u) && sp == q26) {
        return true;
    }
    {
        uint16_t rp_q26 = 0;
        if (read_power_quick(&rp_q26, 200u)) {
            uint16_t tgt_q26_u16 = (uint16_t)q26;
            if ((rp_q26 == tgt_q26_u16) ||
                (rp_q26 + 1 == tgt_q26_u16) || (rp_q26 == tgt_q26_u16 + 1)) {
                return true;
            }
        }
    }

    // --- Fallback: one full handshake + standard ReadPower() verify ---
    (void)dl_handshake();
    float watts = 0.0f;
    if (ReadPower(&watts)) {
        // same rounding as encode_q26_u8:
        const float diff = fabsf(watts - pow);
        if (diff <= (1.0f / 64.0f) + 1e-3f) { // within 1 LSB Q2.6
            return true;
        }
    }
    return false;
}

bool ChangePower(float pow)
{
    /* --- encode Q2.6 (U8 at 0x8108) --- */
    if (pow < 0.0f)
    {
        pow = 0.0f;
    }
    float scaled = pow * 64.0f;
    if (scaled > 255.0f)
    {
        scaled = 255.0f;
    }
    uint8_t q26 = (uint8_t)(scaled + 0.5f);

    /* Force OFF */
    {
        uint8_t off = 0u;
        (void)dl_write_retry(0x800Cu, 1u, &off);
        {
            uint32_t t0 = HAL_GetTick();
            for (;;)
            {
                uint8_t s = 0xFF;
                if ((dl_read_retry(0x800Cu, 1u, &s) == DL_OK) && (s == 0u))
                {
                    break;
                }
                if ((HAL_GetTick() - t0) > 1000u)
                {
                    break;
                }
                HAL_Delay(10u);
            }
        }
    }

    /* Unlock and write setpoint (1 byte @ 0x8108) */
    (void)ocean_unlock();
    {
        dl_status_t wr = dl_write_retry(0x8108u, 1u, &q26);
        bool ok = false;

        if (wr == DL_OK)
        {
            uint8_t rb = 0xFF;
            if (dl_read_retry(0x8108u, 1u, &rb) == DL_OK)
            {
                if (rb == q26)
                {
                    ok = true;
                }
            }
        }

        /* Turn ON again */
        {
            uint8_t on = 1u;
            (void)dl_write_retry(0x800Cu, 1u, &on);
            {
                uint32_t t1 = HAL_GetTick();
                for (;;)
                {
                    uint8_t s = 0u;
                    if ((dl_read_retry(0x800Cu, 1u, &s) == DL_OK) && (s != 0u))
                    {
                        break;
                    }
                    if ((HAL_GetTick() - t1) > 1000u)
                    {
                        break;
                    }
                    HAL_Delay(10u);
                }
            }
        }

        return ok;
    }
}

/* Reads 2 bytes at 0x000A (Channel power report, Q2.6 in U16) -> float watts */
bool ReadPower(float *watts)
{
    uint8_t b[2];

    if (watts == NULL)
    {
        return false;
    }

    if (dl_read_retry(0x000Au, 2u, b) != DL_OK)
    {
        return false;
    }

    {
        uint16_t raw = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
        *watts = ocean_q2_6_to_watts_u16(raw);
    }
    return true;
}

// Output control
/* Writes OUTPUT_STATE @0x800C (exactly 1 byte, 0 or 1). Verifies read-back. */
bool WriteOutputState(uint8_t state)
{
  state = (state != 0) ? 1u : 0u;
  dl_status_t st = dl_write_retry(0x800Cu, 1, &state);
  if (st != DL_OK) {
    /* unlock once, then retry */
    uint8_t keys[8];
    le32_to_buf(&keys[0], OCEAN_UNLOCK_KEY0);
    le32_to_buf(&keys[4], OCEAN_UNLOCK_KEY1);
    (void)dl_write_retry(0x8100u, 8, keys);
    st = dl_write_retry(0x800Cu, 1, &state);
    if (st != DL_OK) return false;
  }
  /* read-back verify */
  uint8_t rb = 0xFF;
  if (dl_read_retry(0x800Cu, 1, &rb) != DL_OK) return false;
  return (rb == state);
}

/* Reads OUTPUT_STATE @ 0x800C (U8). Returns true on success and sets *state. */
bool ReadOutputState(uint8_t *state)
{
    uint8_t v = 0xFF;

    if (state == NULL)
    {
        return false;
    }

    if (dl_read_retry(0x800Cu, 1u, &v) != DL_OK)
    {
        return false;
    }

    *state = v;
    return true;
}

/* Writes DEFAULT_OUTPUT_STATE @ 0x800E (U8). Verifies by read-back. */
bool WriteDefaultState(uint8_t state)
{
    uint8_t v = (state != 0u) ? 1u : 0u;
    dl_status_t st = dl_write_retry(0x800Eu, 1u, &v);

    if (st != DL_OK)
    {
        /* unlock once, then retry */
        (void)ocean_unlock();
        st = dl_write_retry(0x800Eu, 1u, &v);
        if (st != DL_OK)
        {
            return false;
        }
    }

    /* read-back verify */
    {
        uint8_t rb = 0xFFu;
        if (dl_read_retry(0x800Eu, 1u, &rb) != DL_OK)
        {
            return false;
        }
        if (rb != v)
        {
            return false;
        }
    }

    return true;
}

/* Reads DEFAULT_OUTPUT_STATE @ 0x800E (U8) */
bool ReadDefaultState(uint8_t *state)
{
    uint8_t v = 0xFFu;

    if (state == NULL)
    {
        return false;
    }

    if (dl_read_retry(0x800Eu, 1u, &v) != DL_OK)
    {
        return false;
    }

    *state = v;
    return true;
}

// Error flags
/* Reads Error Flags (U32) from Status block offset 0x0004 */
bool ReadErrorflag(uint32_t *out_flags)
{
    uint8_t b[4];

    if (out_flags == NULL)
    {
        return false;
    }

    if (dl_read_retry(0x0004u, 4u, b) != DL_OK)
    {
        return false;
    }

    *out_flags = (uint32_t)b[0]
           | ((uint32_t)b[1] << 8)
           | ((uint32_t)b[2] << 16)
           | ((uint32_t)b[3] << 24);

    return true;
}

/* Writes RESET_ERROR @ 0x8014 (U32 mask). Here we push 0xFFFFFFFF to clear all. */
bool ResetError(void)
{
    uint8_t w[4];
    le32_to_buf(&w[0], 0xFFFFFFFFu);

    /* attempt write; on failure try UNLOCK then retry */
    dl_status_t st = dl_write_retry(0x8014u, 4u, w);
    if (st != DL_OK)
    {
        (void)ocean_unlock();
        st = dl_write_retry(0x8014u, 4u, w);
        if (st != DL_OK)
        {
            return false;
        }
    }

    return true;
}

// Voltage and current values
bool ReadData(void)
{
    // Block [0x0118 .. 0x0128] inclusive: 9 regs × 2 bytes = 18 bytes (0x12)
    const uint16_t base = 0x0118u;
    const uint8_t  len  = 0x12u; // 18
    uint8_t b[0x12];

    // Read the whole measurement window in one transaction
    if (dl_read_retry(base, len, b) != DL_OK) {
        return false;
    }

    // Inline little-endian U16 extraction (no helper)
    const uint16_t ch4_v_raw = (uint16_t)b[0]  | ((uint16_t)b[1]  << 8); // 0x0118 Q14.2
    const uint16_t ch4_i_raw = (uint16_t)b[2]  | ((uint16_t)b[3]  << 8); // 0x011A Q9.7
    const uint16_t ch3_v_raw = (uint16_t)b[4]  | ((uint16_t)b[5]  << 8); // 0x011C Q14.2
    const uint16_t ch3_i_raw = (uint16_t)b[6]  | ((uint16_t)b[7]  << 8); // 0x011E Q9.7
    const uint16_t ch2_v_raw = (uint16_t)b[8]  | ((uint16_t)b[9]  << 8); // 0x0120 Q14.2
    const uint16_t ch2_i_raw = (uint16_t)b[10] | ((uint16_t)b[11] << 8); // 0x0122 Q9.7
    const uint16_t ch1_v_raw = (uint16_t)b[12] | ((uint16_t)b[13] << 8); // 0x0124 Q14.2
    const uint16_t ch1_i_raw = (uint16_t)b[14] | ((uint16_t)b[15] << 8); // 0x0126 Q9.7
    const uint16_t out_v_raw = (uint16_t)b[16] | ((uint16_t)b[17] << 8); // 0x0128 Q14.2

    // Convert using existing helpers
    const float ch4_v = ocean_q14_2_to_volt(ch4_v_raw);
    const float ch4_i = ocean_q9_7_to_amp (ch4_i_raw);
    const float ch3_v = ocean_q14_2_to_volt(ch3_v_raw);
    const float ch3_i = ocean_q9_7_to_amp (ch3_i_raw);
    const float ch2_v = ocean_q14_2_to_volt(ch2_v_raw);
    const float ch2_i = ocean_q9_7_to_amp (ch2_i_raw);
    const float ch1_v = ocean_q14_2_to_volt(ch1_v_raw);
    const float ch1_i = ocean_q9_7_to_amp (ch1_i_raw);
    const float out_v = ocean_q14_2_to_volt(out_v_raw);

    // Print on USART2 (same pattern used elsewhere)
    char line[96];
    int n;

    n = snprintf(line, sizeof line, "Output voltage: %.2f V\r\n", (double)out_v);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 1 voltage: %.2f V\r\n", (double)ch1_v);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 1 current: %.3f A\r\n", (double)ch1_i);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 2 voltage: %.2f V\r\n", (double)ch2_v);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 2 current: %.3f A\r\n", (double)ch2_i);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 3 voltage: %.2f V\r\n", (double)ch3_v);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 3 current: %.3f A\r\n", (double)ch3_i);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 4 voltage: %.2f V\r\n", (double)ch4_v);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    n = snprintf(line, sizeof line, "Channel 4 current: %.3f A\r\n", (double)ch4_i);
    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);

    return true;
}

// Internal commands & functions
void read_and_print_serial(void)
{
    uint8_t b[4];

    if (dl_read_retry(0x8200u, 4u, b) == DL_OK)
    {
        uint32_t serial = (uint32_t)b[0]
                        | ((uint32_t)b[1] << 8)
                        | ((uint32_t)b[2] << 16)
                        | ((uint32_t)b[3] << 24);
        {
            char line[64];
            int n = snprintf(line, sizeof line, "Serial number: 0x%08lX\r\n", (unsigned long)serial);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
        }
    }
}

void read_and_print_accum_on_time(void)
{
    uint8_t b[4];

    if (dl_read_retry(0x8032u, 4u, b) == DL_OK)
    {
        uint32_t ontime = (uint32_t)b[0]
                        | ((uint32_t)b[1] << 8)
                        | ((uint32_t)b[2] << 16)
                        | ((uint32_t)b[3] << 24);
        {
            char line[64];
            int n = snprintf(line, sizeof line, "Accumulated on time: %lu\r\n", (unsigned long)ontime);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
        }
    }
}

void read_one_time_blocks(void)
{
    uint8_t buf[64];

    /* Device Information: 0x0008 (12B): ActiveCh(U8), ChPower(Q2.6 U16), FW(U32), PID(U32) */
    if (dl_read_retry(OCEAN_ADDR_DEVICE_INFO, OCEAN_LEN_DEVICE_INFO, buf) == DL_OK)
    {
        g_active_channels = buf[0]; /* Active channels */
        {
            uint16_t chp_raw = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
            g_channel_power = ocean_q2_6_to_watts_u16(chp_raw); /* Q2.6 -> float watts */
        }
        g_firmware_version = (uint32_t)buf[4]
                           | ((uint32_t)buf[5] << 8)
                           | ((uint32_t)buf[6] << 16)
                           | ((uint32_t)buf[7] << 24);
        g_product_id = (uint32_t)buf[8]
                     | ((uint32_t)buf[9] << 8)
                     | ((uint32_t)buf[10] << 16)
                     | ((uint32_t)buf[11] << 24);

        /* Print the four requested info lines (console on USART2) */
        {
            char line[160];
            int n = snprintf(line, sizeof line, "Active channels: %u\r\n", (unsigned)g_active_channels);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
            n = snprintf(line, sizeof line, "Firmware: 0x%08lX\r\n", (unsigned long)g_firmware_version);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
            n = snprintf(line, sizeof line, "Product ID: 0x%08lX\r\n", (unsigned long)g_product_id);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
            n = snprintf(line, sizeof line, "Channel power: %.3f\r\n", (double)g_channel_power);
            HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
        }
    }

    /* Status + ErrorFlags once at startup; print only "Error 0xXXXXXXXX" */
    if (dl_read_retry(OCEAN_ADDR_STATUS, OCEAN_LEN_STATUS, buf) == DL_OK)
    {
        uint32_t err = (uint32_t)buf[4]
                     | ((uint32_t)buf[5] << 8)
                     | ((uint32_t)buf[6] << 16)
                     | ((uint32_t)buf[7] << 24);
        print_error_hex(err);
    }

    g_config_loaded = true;
}

void poll_periodic(void)
{
    uint8_t buf[OCEAN_LEN_MEAS_BLOCK];

    if (dl_read_retry(OCEAN_ADDR_MEAS_BLOCK, OCEAN_LEN_MEAS_BLOCK, buf) == DL_OK)
    {
        size_t i;
        for (i = 0u; i < (sizeof(k_meas_sel)/sizeof(k_meas_sel[0])); i++)
        {
            uint16_t off = (uint16_t)(k_meas_sel[i].addr - OCEAN_ADDR_MEAS_BLOCK);
            if ((off + 1u) >= OCEAN_LEN_MEAS_BLOCK)
            {
                continue;
            }

            {
                uint16_t raw = (uint16_t)buf[off] | ((uint16_t)buf[off + 1u] << 8);
                char line[96];

                if (k_meas_sel[i].qfmt == QFMT_Q14_2)
                {
                    float v = ocean_q14_2_to_volt(raw);
                    int n = snprintf(line, sizeof line, "%s: %.2f V\r\n", k_meas_sel[i].name, (double)v);
                    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
                }
                else
                {
                    float a = ocean_q9_7_to_amp(raw);
                    int n = snprintf(line, sizeof line, "%s: %.3f A\r\n", k_meas_sel[i].name, (double)a);
                    HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
                }
            }
        }
    }

    {
        uint8_t sblk[OCEAN_LEN_STATUS];
        if (dl_read_retry(OCEAN_ADDR_STATUS, OCEAN_LEN_STATUS, sblk) == DL_OK)
        {
            uint32_t err = (uint32_t)sblk[4]
                         | ((uint32_t)sblk[5] << 8)
                         | ((uint32_t)sblk[6] << 16)
                         | ((uint32_t)sblk[7] << 24);
            print_error_hex(err);
        }
    }
}

