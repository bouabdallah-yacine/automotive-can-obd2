/*
 * ============================================================================
 *  CAN 2.0A physical/data link layer, bit-accurate (ISO 11898-1)
 * ============================================================================
 *  Standard frame:   SOF | ID (11) | RTR | IDE | r0 | DLC (4) | data (0..64)
 *                    | CRC (15) | CRC delim. | ACK | ACK delim. | EOF (7)
 *
 *  - CRC-15 (polynomial 0x4599) computed from SOF to the end of the data
 *  - bit stuffing: after 5 identical bits, a complementary bit is inserted
 *  - arbitration: bit 0 (dominant) overrides bit 1 (recessive) → the lowest
 *    ID wins with no collision and no time lost
 * ============================================================================
 */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define CAN_MAX_BITS 200

typedef struct { uint16_t id; uint8_t dlc; uint8_t data[8]; } can_frame_t;

typedef enum { CAN_OK = 0, CAN_ERR_STUFF, CAN_ERR_CRC, CAN_ERR_FORM } can_err_t;

uint16_t    can_crc15(const uint8_t *bits, int n);
/* bits[] = sequence of 0/1 as it travels on the bus; stuff[] (optional) = 1 for a stuff bit */
int         can_encode(const can_frame_t *f, uint8_t *bits, uint8_t *stuff);
can_err_t   can_decode(const uint8_t *bits, int n, can_frame_t *out);
const char *can_err_str(can_err_t e);
/* Arbitration between n frames sent at the same time: returns the winner's index,
   lost_at[i] = identifier bit at which node i lost (-1 for the winner) */
int         can_arbitrate(const can_frame_t *const *cand, int n, int *lost_at);

#ifdef __cplusplus
}
#endif
