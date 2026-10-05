#include "can_bus.h"
#include <string.h>

static int q_push(can_queue_t *q, const can_frame_t *f) {
  if (q->count == CAN_QLEN) return -1;
  q->q[(q->head + q->count++) % CAN_QLEN] = *f; return 0;
}
static can_frame_t *q_peek(can_queue_t *q) { return q->count ? &q->q[q->head] : 0; }
static void q_pop(can_queue_t *q) { if (q->count) { q->head = (q->head + 1) % CAN_QLEN; q->count--; } }

const char *can_state_str(can_node_state_t s) {
  return s == NODE_ERROR_ACTIVE ? "active" : s == NODE_ERROR_PASSIVE ? "error passive" : "BUS-OFF";
}

static void update_state(can_node_t *n) {
  if (n->tec >= 256) n->state = NODE_BUS_OFF;             /* the node disconnects itself */
  else if (n->tec >= 128 || n->rec >= 128) n->state = NODE_ERROR_PASSIVE;
  else n->state = NODE_ERROR_ACTIVE;
}

void can_bus_init(can_bus_t *b) { memset(b, 0, sizeof *b); b->arb_winner = -1; b->inject_node = -1; }

int can_bus_add_node(can_bus_t *b, const char *name, uint16_t id, uint16_t mask) {
  if (b->n_nodes == CAN_MAX_NODES) return -1;
  can_node_t *n = &b->nodes[b->n_nodes];
  memset(n, 0, sizeof *n);
  n->name = name; n->filt_id = id; n->filt_mask = mask;
  return b->n_nodes++;
}

int can_send(can_bus_t *b, int node, const can_frame_t *f) {
  if (b->nodes[node].state == NODE_BUS_OFF) return -1;
  return q_push(&b->nodes[node].tx, f);
}

int can_receive(can_bus_t *b, int node, can_frame_t *f) {
  can_frame_t *p = q_peek(&b->nodes[node].rx);
  if (!p) return 0;
  *f = *p; q_pop(&b->nodes[node].rx); return 1;
}

int can_bus_step(can_bus_t *b) {
  /* 1. Arbitration between all nodes that want to transmit */
  const can_frame_t *cand[CAN_MAX_NODES]; int who[CAN_MAX_NODES], lost[CAN_MAX_NODES], n = 0;
  for (int i = 0; i < b->n_nodes; i++) {
    can_frame_t *f = q_peek(&b->nodes[i].tx);
    if (f && b->nodes[i].state != NODE_BUS_OFF) { cand[n] = f; who[n] = i; n++; }
  }
  if (!n) return 0;
  int w = can_arbitrate(cand, n, lost);
  if (n > 1) {
    b->arbitrations++;
    int l = (w == 0) ? 1 : 0;
    b->arb_winner = who[w]; b->arb_loser = who[l];
    b->arb_win_id = cand[w]->id; b->arb_lose_id = cand[l]->id; b->arb_lost_bit = lost[l];
  }
  int tx = who[w];
  const can_frame_t *f = cand[w];

  /* 2. Bit-level transmission (with an optional error injected on the wire) */
  b->last_nbits = can_encode(f, b->last_bits, b->last_stuff);
  b->last_frame = *f; b->last_sender = tx;
  if (b->inject_error > 0 && (b->inject_node < 0 || b->inject_node == tx)) {   /* noise on one bit */
    b->last_bits[25] ^= 1; b->inject_error--;
  }
  b->bits_total += b->last_nbits + 3;                                        /* + interframe space */

  /* 3. Every receiver checks the frame */
  can_frame_t rx;
  can_err_t e = can_decode(b->last_bits, b->last_nbits, &rx);
  if (e != CAN_OK) {                                     /* error frame → automatic retransmission */
    b->frames_err++;
    b->bits_total += 14;
    b->nodes[tx].tec += 8;
    for (int i = 0; i < b->n_nodes; i++) if (i != tx) { b->nodes[i].rec += 1; update_state(&b->nodes[i]); }
    update_state(&b->nodes[tx]);
    return -1;
  }
  b->frames_ok++;
  if (b->nodes[tx].tec) b->nodes[tx].tec--;
  update_state(&b->nodes[tx]);
  q_pop(&b->nodes[tx].tx);
  for (int i = 0; i < b->n_nodes; i++) {
    can_node_t *nd = &b->nodes[i];
    if (i == tx) continue;
    if (nd->rec) nd->rec--;
    update_state(nd);
    if ((rx.id & nd->filt_mask) == (nd->filt_id & nd->filt_mask)) q_push(&nd->rx, &rx);
  }
  return 1;
}
