/*
 * Virtual CAN bus: several nodes (ECUs), arbitration, acceptance filters,
 * error detection and fault confinement (TEC/REC counters, "error active",
 * "error passive" and "bus-off" states) as in a real CAN controller.
 */
#pragma once
#include "can.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CAN_MAX_NODES 6
#define CAN_QLEN      16
#define CAN_BITRATE   500000UL     /* 500 kbit/s, the typical powertrain CAN bit rate */

typedef enum { NODE_ERROR_ACTIVE, NODE_ERROR_PASSIVE, NODE_BUS_OFF } can_node_state_t;

typedef struct { can_frame_t q[CAN_QLEN]; int head, count; } can_queue_t;

typedef struct {
  const char *name;
  uint16_t filt_id, filt_mask;       /* accepts if (id & mask) == (filt_id & mask) */
  can_queue_t tx, rx;
  uint16_t tec, rec;                 /* transmit / receive error counters */
  can_node_state_t state;
} can_node_t;

typedef struct {
  can_node_t nodes[CAN_MAX_NODES];
  int n_nodes;
  int inject_error;                  /* >0: corrupts the next N frames (1 bit) */
  int inject_node;                   /* node targeted by the noise (-1 = any node) */
  /* statistics */
  uint32_t frames_ok, frames_err, bits_total, arbitrations;
  /* last transmitted frame (for the bit-level display) */
  uint8_t last_bits[CAN_MAX_BITS], last_stuff[CAN_MAX_BITS];
  int last_nbits; can_frame_t last_frame; int last_sender;
  /* last contested arbitration */
  int arb_winner, arb_loser; uint16_t arb_win_id, arb_lose_id; int arb_lost_bit;
} can_bus_t;

void  can_bus_init(can_bus_t *b);
int   can_bus_add_node(can_bus_t *b, const char *name, uint16_t filt_id, uint16_t filt_mask);
int   can_send(can_bus_t *b, int node, const can_frame_t *f);      /* 0 = OK, -1 = queue full / bus-off */
int   can_receive(can_bus_t *b, int node, can_frame_t *f);         /* 1 if a frame was received */
/* Transmits ONE frame (including arbitration). Returns 1 if a frame went through,
   0 if there was nothing to send, -1 if an error was detected (the frame will be resent). */
int   can_bus_step(can_bus_t *b);
const char *can_state_str(can_node_state_t s);

#ifdef __cplusplus
}
#endif
