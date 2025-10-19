#include "DataLink_Driver.h"
#include "main.h"     /* HAL_GetTick, HAL_Delay */
#include "usart.h"    /* extern UART_HandleTypeDef huart1 */
#include <string.h>

/* =============================================================================
 * CRC16 (Ocean polynomial 0xA2EB) - identical logic as in v0.1.2 main.c
 * ===========================================================================*/
static uint16_t crc16_lut[256];

/* Computes the 16-bit bit-reversal of the input. Used to build the CRC16 LUT. */
static uint16_t reverse_bits16(uint16_t x)
{
  x = (uint16_t)(((x >> 8) | (x << 8)));
  x = (uint16_t)((((x & 0xF0F0u) >> 4) | ((x & 0x0F0Fu) << 4)));
  x = (uint16_t)((((x & 0xCCCCu) >> 2) | ((x & 0x3333u) << 2)));
  x = (uint16_t)((((x & 0xAAAAu) >> 1) | ((x & 0x5555u) << 1)));

  return x;
}

/* Initializes a 256-entry CRC16 lookup table (poly 0xA2EB, init 0xFFFF). */
void crc16_init(void)
{
  uint16_t poly = 0xA2EB;
  uint16_t rp   = reverse_bits16(poly);
  for (int i = 0; i < 256; ++i)
  {
    uint16_t c = (uint16_t)i;
    for (int b = 0; b < 8; ++b)
    {
      c = (uint16_t)(((c & 1u) ? ((c >> 1) ^ rp) : (c >> 1)));
    }
    crc16_lut[i] = c;
  }
}

/* Computes CRC16 (poly 0xA2EB, init provided by caller) over a byte span. */
static uint16_t crc16_compute(const uint8_t* data, uint16_t len, uint16_t init)
{
  uint32_t cs = init;
  while (len-- > 0)
  {
    cs = (cs >> 8) ^ crc16_lut[*data++ ^ (uint8_t)(cs & 0xFF)];
  }
  return (uint16_t)cs;
}

/* =============================================================================
 * HAL mapping helpers (keep HAL internal to .c)
 * ===========================================================================*/
static inline dl_status_t dl_from_hal(HAL_StatusTypeDef s)
{
  switch (s)
  {
    case HAL_OK:      return DL_OK;
    case HAL_BUSY:    return DL_ERR_BUSY;
    case HAL_TIMEOUT: return DL_ERR_TIMEOUT;
    default:          return DL_ERR_LINK;
  }
}

/* Polling UART helper that reads exactly 'n' bytes within an overall deadline.
 * Returns DL_OK if 'n' bytes were read in time, otherwise DL_ERR_TIMEOUT/BUSY/LINK. */
static dl_status_t uart_read_exact(uint8_t* p, uint16_t n, uint32_t overall_ms)
{
  uint32_t t0 = HAL_GetTick();

  while (n) {
    uint32_t now = HAL_GetTick();
    if ((now - t0) >= overall_ms) return DL_ERR_TIMEOUT;

    uint32_t left  = overall_ms - (now - t0);
    uint32_t slice = (left < 20u) ? left : 20u;

    HAL_StatusTypeDef hs = HAL_UART_Receive(&huart1, p, 1, slice);
    if (hs == HAL_OK) { ++p; --n; continue; }
    if (hs == HAL_TIMEOUT) continue; /* keep trying until deadline */
    return dl_from_hal(hs);           /* BUSY/ERROR */
  }

  return DL_OK;
}

/* =============================================================================
 * DataLink handshake - identical logic as in v0.1.2 main.c (ported)
 * ===========================================================================*/
/* Device-initiated branch: wait RESET, reply with RESET_RESPONSE, drain line. */
static bool dl_answer_device_reset(void)
{
  uint8_t rx[DL_OVERHEAD];
  dl_status_t st = uart_read_exact(rx, DL_HDR_SIZE, 50u);
  if (st != DL_OK)
	  return false;

  if (rx[0] != DL_TYPE_RESET || rx[1] != DL_OVERHEAD)
	  return false;

  st = uart_read_exact(rx + 2, DL_CRC_SIZE, 50u);
  if (st != DL_OK)
	  return false;

  if (crc16_compute(rx, DL_OVERHEAD, 0xFFFF) != 0)
	  return false;

  uint8_t tx[DL_OVERHEAD];
  tx[0] = DL_TYPE_RESET_RESP; tx[1] = DL_OVERHEAD;
  uint16_t c = crc16_compute(tx, DL_HDR_SIZE, 0xFFFF);
  tx[2] = (uint8_t)(c & 0xFF); tx[3] = (uint8_t)(c >> 8);
  (void)HAL_UART_Transmit(&huart1, tx, DL_OVERHEAD, 20);

  HAL_Delay(125u);

  /* drain line for up to 200 ms */
  uint32_t t = HAL_GetTick(); uint8_t tmp;
  while ((HAL_GetTick() - t) < 200u)
  {
    if (HAL_UART_Receive(&huart1, &tmp, 1, 20) != HAL_OK)
    	break;
  }

  return true;
}

/* Host-initiated branch: send RESET, expect RESET_RESPONSE, drain line. */
static bool dl_host_reset(void)
{
  uint8_t tx[DL_OVERHEAD], rx[DL_OVERHEAD];
  tx[0] = DL_TYPE_RESET; tx[1] = DL_OVERHEAD;
  uint16_t c = crc16_compute(tx, DL_HDR_SIZE, 0xFFFF);
  tx[2] = (uint8_t)(c & 0xFF); tx[3] = (uint8_t)(c >> 8);
  (void)HAL_UART_Transmit(&huart1, tx, DL_OVERHEAD, 20);

  dl_status_t st = uart_read_exact(rx, DL_HDR_SIZE, 500u);
  if (st != DL_OK)
	  return false;

  if (rx[0] != DL_TYPE_RESET_RESP || rx[1] != DL_OVERHEAD)
	  return false;

  st = uart_read_exact(rx + 2, DL_CRC_SIZE, 500u);
  if (st != DL_OK)
	  return false;

  if (crc16_compute(rx, DL_OVERHEAD, 0xFFFF) != 0)
	  return false;

  HAL_Delay(125u);

  /* drain line for up to 200 ms */
  uint32_t t = HAL_GetTick(); uint8_t tmp;
  while ((HAL_GetTick() - t) < 200u)
  {
    if (HAL_UART_Receive(&huart1, &tmp, 1, 20) != HAL_OK)
    	break;
  }
  return true;
}

/* High-level handshake: try device-reset branch repeatedly, then host-reset. */
bool dl_handshake(void)
{
  for (int i = 0; i < 20; ++i)
  {
    if (dl_answer_device_reset())
    	return true;

    HAL_Delay(25);
  }

  for (int i = 0; i < 20; ++i)
  {
    if (dl_host_reset())
    	return true;

    HAL_Delay(100);
  }

  return false;
}

bool dl_handshake_quick(uint8_t ans_attempts, uint8_t host_attempts, uint32_t ans_gap_ms, uint32_t host_gap_ms)
{
	// Try a few quick "device-reset answer" windows
	for (uint8_t i = 0; i < ans_attempts; ++i)
	{
		if (dl_answer_device_reset())
			return true;

		HAL_Delay(ans_gap_ms);
	}

	// Then a few quick host-initiated resets
	for (uint8_t i = 0; i < host_attempts; ++i)
	{
		if (dl_host_reset())
			return true;

		HAL_Delay(host_gap_ms);
	}

	return false;
}

/* =============================================================================
 * Low-level transactions - identical logic as in v0.1.2 main.c (ported)
 * ===========================================================================*/
/* READ: send (type=0x02, total=overhead+3, addr LSB/MSB, len, CRC), wait RESP */
dl_status_t dl_read(uint16_t addr, uint8_t len, uint8_t* outBuf, uint32_t headerWaitMs, uint32_t payloadWaitMs)
{
  uint8_t  tx[8];
  uint8_t  rx[96];
  uint16_t total;
  uint16_t c;

  tx[0] = DL_TYPE_READ;
  tx[1] = (uint8_t)(DL_OVERHEAD + 3u);
  tx[2] = (uint8_t)(addr & 0xFF);
  tx[3] = (uint8_t)(addr >> 8);
  tx[4] = len;
  c = crc16_compute(tx, (uint16_t)(DL_HDR_SIZE + 3u), 0xFFFF);
  tx[5] = (uint8_t)(c & 0xFF);
  tx[6] = (uint8_t)(c >> 8);
  (void)HAL_UART_Transmit(&huart1, tx, 7u, 20);

  dl_status_t st = uart_read_exact(rx, 2, headerWaitMs);

  if (st != DL_OK)
	  return st;                             /* timeout/busy/link */

  if (rx[0] != DL_TYPE_READ_RESP)
	  return DL_ERR_INVALID_RESPONSE;

  total = rx[1];
  if (total < (DL_OVERHEAD + 4u) || total > sizeof(rx))
	  return DL_ERR_INVALID_RESPONSE;

  st = uart_read_exact(rx + 2, (uint16_t)(total - 2), payloadWaitMs);
  if (st != DL_OK)
	  return st;

  if (crc16_compute(rx, total, 0xFFFF) != 0)
	  return DL_ERR_INVALID_RESPONSE;

  /* parse response payload: status(1), addr(2), size(1), data(size) */
  uint8_t  status = rx[2];
  uint16_t raddr  = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
  uint8_t  size   = rx[5];
  if (status != 0 || raddr != addr || size != len)
	  return DL_ERR_INVALID_RESPONSE;

  memcpy(outBuf, &rx[6], len);

  return DL_OK;
}

/* Read-with-retry: call dl_read; on failure, handshake and retry up to DL_CMD_RETRIES. */
dl_status_t dl_read_retry(uint16_t addr, uint8_t len, uint8_t* outBuf)
{
  dl_status_t last = DL_ERR_LINK;
  for (int attempt = 0; attempt < (int)DL_CMD_RETRIES; ++attempt)
  {
    dl_status_t st = dl_read(addr, len, outBuf, /*header*/500u, /*payload*/500u);
    if (st == DL_OK)
    	return DL_OK;

    last = st;
    (void)dl_handshake(); /* re-sync and retry */
  }

  return last;
}

/* WRITE: send (type=0x01, total=overhead+3+len, addr LSB/MSB, size, data, CRC), wait RESP */
dl_status_t dl_write(uint16_t addr, uint8_t len, const uint8_t* inBuf, uint32_t headerWaitMs, uint32_t payloadWaitMs)
{
  if (len > 32u)
	  return DL_ERR_INVALID_RESPONSE; /* same constraint as original */

  uint8_t  frame[7 + 32 + 2];
  uint16_t total = (uint16_t)(DL_OVERHEAD + 3u + len);

  frame[0] = DL_TYPE_WRITE;
  frame[1] = (uint8_t)total;
  frame[2] = (uint8_t)(addr & 0xFF);
  frame[3] = (uint8_t)(addr >> 8);
  frame[4] = len;

  if (len)
	  memcpy(&frame[5], inBuf, len);

  uint16_t c = crc16_compute(frame, (uint16_t)(DL_HDR_SIZE + 3u + len), 0xFFFF);
  frame[5 + len] = (uint8_t)(c & 0xFF);
  frame[6 + len] = (uint8_t)(c >> 8);

  (void)HAL_UART_Transmit(&huart1, frame, (uint16_t)(7u + len), 50);

  /* Expect response: type=WRITE_RESP, total=DL_OVERHEAD+4, payload=status(1)+addr(2)+size(1) */
  uint8_t  rxh[2];
  dl_status_t st = uart_read_exact(rxh, 2, headerWaitMs);

  if (st != DL_OK)
	  return st;

  if (rxh[0] != DL_TYPE_WRITE_RESP)
	  return DL_ERR_INVALID_RESPONSE;

  uint16_t rtotal = rxh[1];
  if (rtotal != (DL_OVERHEAD + 4u))
	  return DL_ERR_INVALID_RESPONSE;

  /* read full tail = payload(4) + CRC(2) = 6 bytes */
  uint8_t rtail[6];
  st = uart_read_exact(rtail, 6, payloadWaitMs);
  if (st != DL_OK)
	  return st;

  uint8_t full[8];
  full[0] = rxh[0];
  full[1] = rxh[1];
  memcpy(&full[2], rtail, sizeof(rtail));

  if (crc16_compute(full, rtotal, 0xFFFF) != 0)
	  return DL_ERR_INVALID_RESPONSE;

  uint8_t  status = full[2];
  uint16_t raddr  = (uint16_t)full[3] | ((uint16_t)full[4] << 8);
  uint8_t  rsize  = full[5];

  if (status != 0 || raddr != addr || rsize != len)
	  return DL_ERR_INVALID_RESPONSE;

  return DL_OK;
}

/* Write-with-retry: call dl_write; on failure, handshake and retry. */
dl_status_t dl_write_retry(uint16_t addr, uint8_t len, const uint8_t* inBuf)
{
  dl_status_t last = DL_ERR_LINK;
  for (int attempt = 0; attempt < (int)DL_CMD_RETRIES; ++attempt)
  {
    dl_status_t st = dl_write(addr, len, inBuf, /*header*/500u, /*payload*/500u);
    if (st == DL_OK)
    	return DL_OK;
    last = st;
    (void)dl_handshake(); /* re-sync and retry */
  }

  return last;
}

