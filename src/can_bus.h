/*
 * Bus CAN virtuel : plusieurs nœuds (calculateurs), arbitrage, filtres
 * d'acceptation, détection d'erreurs et confinement (compteurs TEC/REC,
 * états « actif », « passif », « bus-off ») comme dans un vrai contrôleur CAN.
 */
#pragma once
#include "can.h"
#ifdef __cplusplus
extern "C" {
#endif

#define CAN_MAX_NODES 6
#define CAN_QLEN      16
#define CAN_BITRATE   500000UL     /* 500 kbit/s, le débit classique du CAN moteur */

typedef enum { NODE_ERROR_ACTIVE, NODE_ERROR_PASSIVE, NODE_BUS_OFF } can_node_state_t;

typedef struct { can_frame_t q[CAN_QLEN]; int head, count; } can_queue_t;

typedef struct {
  const char *name;
  uint16_t filt_id, filt_mask;       /* accepte si (id & mask) == (filt_id & mask) */
  can_queue_t tx, rx;
  uint16_t tec, rec;                 /* compteurs d'erreurs émission / réception */
  can_node_state_t state;
} can_node_t;

typedef struct {
  can_node_t nodes[CAN_MAX_NODES];
  int n_nodes;
  int inject_error;                  /* >0 : corrompt les N prochaines trames (1 bit) */
  int inject_node;                   /* nœud visé par les parasites (-1 = n'importe lequel) */
  /* statistiques */
  uint32_t frames_ok, frames_err, bits_total, arbitrations;
  /* dernière trame transmise (pour l'affichage au bit près) */
  uint8_t last_bits[CAN_MAX_BITS], last_stuff[CAN_MAX_BITS];
  int last_nbits; can_frame_t last_frame; int last_sender;
  /* dernier arbitrage disputé */
  int arb_winner, arb_loser; uint16_t arb_win_id, arb_lose_id; int arb_lost_bit;
} can_bus_t;

void  can_bus_init(can_bus_t *b);
int   can_bus_add_node(can_bus_t *b, const char *name, uint16_t filt_id, uint16_t filt_mask);
int   can_send(can_bus_t *b, int node, const can_frame_t *f);      /* 0 = OK, -1 = file pleine / bus-off */
int   can_receive(can_bus_t *b, int node, can_frame_t *f);         /* 1 si une trame reçue */
/* Transmet UNE trame (arbitrage compris). Renvoie 1 si une trame est passée,
   0 si rien à émettre, -1 si erreur détectée (la trame sera réémise). */
int   can_bus_step(can_bus_t *b);
const char *can_state_str(can_node_state_t s);

#ifdef __cplusplus
}
#endif
