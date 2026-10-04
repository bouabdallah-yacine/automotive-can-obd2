/*
 * ============================================================================
 *  Couche physique/liaison CAN 2.0A, au bit près (ISO 11898-1)
 * ============================================================================
 *  Trame standard :  SOF | ID (11) | RTR | IDE | r0 | DLC (4) | données (0..64)
 *                    | CRC (15) | délim. CRC | ACK | délim. ACK | EOF (7)
 *
 *  - CRC-15 (polynôme 0x4599) calculé de SOF à la fin des données
 *  - bit stuffing : après 5 bits identiques, un bit inverse est inséré
 *  - arbitrage : bit 0 (dominant) écrase bit 1 (récessif) → l'ID le plus
 *    petit gagne sans collision ni perte de temps
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
/* bits[] = suite de 0/1 telle qu'elle passe sur le bus ; stuff[] (optionnel) = 1 pour un bit de bourrage */
int         can_encode(const can_frame_t *f, uint8_t *bits, uint8_t *stuff);
can_err_t   can_decode(const uint8_t *bits, int n, can_frame_t *out);
const char *can_err_str(can_err_t e);
/* Arbitrage entre n trames émises en même temps : renvoie l'indice du gagnant,
   lost_at[i] = bit de l'identifiant où le nœud i a perdu (-1 pour le gagnant) */
int         can_arbitrate(const can_frame_t *const *cand, int n, int *lost_at);

#ifdef __cplusplus
}
#endif
