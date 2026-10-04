/*
 * Diagnostic OBD-II (SAE J1979) sur CAN avec transport ISO-TP (ISO 15765-2)
 *  - requête fonctionnelle 0x7DF, réponse du calculateur moteur 0x7E8
 *  - mode 01 : données en direct (régime, vitesse, température, papillon, charge)
 *  - mode 03 : lire les codes défauts   - mode 04 : effacer les codes défauts
 *  - mode 09 : numéro de série du véhicule (VIN, 17 caractères → multi-trames ISO-TP)
 */
#pragma once
#include <stdint.h>
#include "can.h"
#ifdef __cplusplus
extern "C" {
#endif

#define OBD_REQ_ID      0x7DF
#define OBD_ECU_REQ_ID  0x7E0
#define OBD_ECU_RESP_ID 0x7E8

typedef struct {
  float rpm, speed_kmh, coolant_c, throttle_pct, load_pct;
  uint16_t dtc[8]; int n_dtc;
  char vin[18];
} vehicle_t;

/* Codes défauts : "P0217" <-> 0x0217 */
uint16_t dtc_encode(const char *s);
void     dtc_decode(uint16_t v, char out[6]);
const char *dtc_description(uint16_t v);

/* Côté calculateur : traite une requête OBD (sans l'octet de longueur ISO-TP) */
int  obd_handle(vehicle_t *v, const uint8_t *req, int len, uint8_t *resp, int max);
/* Côté outil de diagnostic : réponse → texte lisible */
void obd_describe(const uint8_t *resp, int len, char *out, int max);

/* --- ISO-TP : découpage des messages de plus de 7 octets --- */
typedef struct { uint16_t id; uint8_t buf[64]; uint16_t len, pos; uint8_t sn; int waiting_fc; } isotp_tx_t;
typedef struct { uint8_t buf[64]; uint16_t len, got; uint8_t sn; int active; } isotp_rx_t;

/* Émetteur : 1re trame (simple ou « First Frame ») */
void isotp_tx_start(isotp_tx_t *t, uint16_t id, const uint8_t *data, uint16_t len, can_frame_t *out);
/* Émetteur : trame suivante (« Consecutive Frame ») ; 1 si une trame, 0 si fini ou en attente du contrôle de flux */
int  isotp_tx_next(isotp_tx_t *t, can_frame_t *out);
void isotp_tx_flow_control(isotp_tx_t *t, const can_frame_t *fc);
/* Récepteur : 1 = message complet ; *send_fc = 1 si une trame de contrôle de flux doit partir (fc) */
int  isotp_rx_feed(isotp_rx_t *r, const can_frame_t *in, can_frame_t *fc, uint16_t fc_id, int *send_fc);

#ifdef __cplusplus
}
#endif
