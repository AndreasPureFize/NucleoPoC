#ifndef DATALINK_DRIVER_H
#define DATALINK_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Protocol framing & constants */
#define DL_HDR_SIZE            2u								/* Number of bytes in the DataLink frame header (type + total length). */
#define DL_CRC_SIZE            2u								/* Number of bytes in the DataLink frame CRC trailer (CRC16 LE). */
#define DL_OVERHEAD            (DL_HDR_SIZE + DL_CRC_SIZE)		/* Total non-payload overhead per frame (header + CRC). */

/* Package types */
#define DL_TYPE_RESET          0x7Fu							/* DataLink frame type: RESET (device or host issues to resync link). */
#define DL_TYPE_RESET_RESP     0xFFu							/* DataLink frame type: RESET_RESPONSE (acknowledges RESET). */
#define DL_TYPE_READ           0x02u							/* DataLink frame type: READ request (host → device). */
#define DL_TYPE_READ_RESP      0x82u							/* DataLink frame type: READ_RESPONSE (device → host; includes status, addr, size, data). */
#define DL_TYPE_WRITE          0x01u							/* DataLink frame type: WRITE request (host → device; applies contiguous bytes at addr). */
#define DL_TYPE_WRITE_RESP     0x81u							/* DataLink frame type: WRITE_RESPONSE (device → host; echoes status, addr, size). */

/* Simple retry policy for command helpers */
#define DL_CMD_RETRIES         3u								/* Number of command attempts (READ/WRITE) before giving up; on failure a handshake is attempted to re-synchronize before retrying. Tunable for link robustness. */

/* Local DataLink status */
typedef enum {
    DL_OK = 0,
    DL_ERR_BUSY,
    DL_ERR_TIMEOUT,
    DL_ERR_INVALID_RESPONSE,
    DL_ERR_LINK
} dl_status_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/* Build CRC16 LUT once at startup (poly 0xA2EB, init 0xFFFF) */
void crc16_init(void);

/* Synchronize link (device/host reset handshake). Returns true if OK. */
bool dl_handshake(void);

// A lighter/faster re-sync used after known reconfig writes
bool dl_handshake_quick(uint8_t ans_attempts, uint8_t host_attempts,
                        uint32_t ans_gap_ms, uint32_t host_gap_ms);

/* Low-level blocking transactions */
dl_status_t dl_read(
    uint16_t addr, uint8_t len,
    uint8_t* outBuf,
    uint32_t headerWaitMs, uint32_t payloadWaitMs);

dl_status_t dl_write(
    uint16_t addr, uint8_t len,
    const uint8_t* inBuf,
    uint32_t headerWaitMs, uint32_t payloadWaitMs);

/* Convenience wrappers with retry + handshake on failure */
dl_status_t dl_read_retry (uint16_t addr, uint8_t len, uint8_t* outBuf);
dl_status_t dl_write_retry(uint16_t addr, uint8_t len, const uint8_t* inBuf);


#ifdef __cplusplus
}
#endif
#endif /* DATALINK_DRIVER_H */
