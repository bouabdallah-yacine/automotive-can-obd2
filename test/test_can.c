/*
 * Tests du réseau CAN et du diagnostic OBD-II sur PC (même code que sur l'ESP32)
 *   gcc -O2 -Wall -Wextra -Isrc -o t test/test_can.c src/can.c src/can_bus.c src/obd.c && ./t
 */
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "can_bus.h"
#include "obd.h"

static int fails = 0;
#define CHECK(c, ...) do { int ok_ = (c); printf("%s ", ok_ ? "[OK]   " : "[ECHEC]"); printf(__VA_ARGS__); \
  printf("\n"); if (!ok_) fails++; } while (0)

int main(void) {
  uint8_t bits[CAN_MAX_BITS], stf[CAN_MAX_BITS];

  /* 1. Aller-retour encodage / décodage */
  can_frame_t f = { 0x0C0, 8, { 0x1A, 0xF8, 0x5A, 0x00, 0xFF, 0x00, 0x42, 0x7E } }, g;
  int n = can_encode(&f, bits, stf);
  CHECK(can_decode(bits, n, &g) == CAN_OK && g.id == f.id && g.dlc == 8 && !memcmp(g.data, f.data, 8),
        "trame 0x0C0 (8 octets) : encodée en %d bits puis décodée à l'identique", n);

  /* 2. Bit stuffing : jamais plus de 5 bits identiques de SOF à la fin du CRC */
  can_frame_t z = { 0x000, 8, { 0 } };
  n = can_encode(&z, bits, stf);
  int nst = 0, maxrun = 0, run = 0;
  for (int i = 0; i < n - 10; i++) { nst += stf[i]; run = (i && bits[i] == bits[i - 1]) ? run + 1 : 1; if (run > maxrun) maxrun = run; }
  CHECK(maxrun <= 5 && nst > 10, "trame tout à zéro : %d bits de bourrage insérés, jamais plus de 5 bits identiques", nst);

  /* 3. Le CRC détecte toute erreur sur 1 bit (n'importe où entre SOF et la fin du CRC) */
  n = can_encode(&f, bits, NULL);
  int detected = 0, total = n - 10;
  for (int i = 0; i < total; i++) {
    uint8_t b2[CAN_MAX_BITS]; memcpy(b2, bits, n); b2[i] ^= 1;
    if (can_decode(b2, n, &g) != CAN_OK) detected++;
  }
  CHECK(detected == total, "erreur d'un bit injectée à chacune des %d positions : %d détectées", total, detected);

  /* 4. Deux bits faux : toujours détectés aussi (CRC-15, distance de Hamming 6) */
  int det2 = 0, tot2 = 0;
  for (int i = 0; i < total; i += 3) for (int j = i + 1; j < total; j += 5) {
    uint8_t b2[CAN_MAX_BITS]; memcpy(b2, bits, n); b2[i] ^= 1; b2[j] ^= 1; tot2++;
    if (can_decode(b2, n, &g) != CAN_OK) det2++;
  }
  CHECK(det2 == tot2, "erreurs doubles : %d / %d détectées", det2, tot2);

  /* 5. Arbitrage : l'ID le plus petit gagne, au bit près */
  can_frame_t a = { 0x0C0, 0, {0} }, b = { 0x1A0, 0, {0} }, c = { 0x7DF, 0, {0} };
  const can_frame_t *cand[] = { &b, &c, &a };
  int lost[3];
  int w = can_arbitrate(cand, 3, lost);
  CHECK(w == 2 && lost[0] == 2 && lost[1] == 0, "arbitrage 0x1A0 / 0x7DF / 0x0C0 : 0x0C0 gagne (0x7DF perd au bit 0, 0x1A0 au bit 2)");

  /* 6. Bus : arbitrage, filtres, retransmission après erreur, confinement */
  can_bus_t bus; can_bus_init(&bus);
  int moteur = can_bus_add_node(&bus, "moteur", 0x7DF, 0x7F0);
  int abs_ = can_bus_add_node(&bus, "abs", 0, 0x000);
  int tdb = can_bus_add_node(&bus, "tableau", 0x000, 0x000);
  can_send(&bus, abs_, &b); can_send(&bus, moteur, &a);
  can_bus_step(&bus);
  can_frame_t r; int got = can_receive(&bus, tdb, &r);
  CHECK(got && r.id == 0x0C0, "bus : 0x0C0 du moteur passe en premier, le tableau de bord la reçoit");
  CHECK(!can_receive(&bus, moteur, &r) || r.id != 0x0C0, "filtre d'acceptation : le moteur ignore les trames qui ne le concernent pas");
  can_bus_step(&bus);
  bus.inject_error = 1;
  can_send(&bus, moteur, &a);
  int s1 = can_bus_step(&bus), s2 = can_bus_step(&bus);
  CHECK(s1 == -1 && s2 == 1 && bus.frames_err == 1, "parasite sur le câble : erreur détectée, trame réémise automatiquement");
  bus.inject_error = 1000;
  for (int i = 0; i < 40; i++) { can_send(&bus, moteur, &a); can_bus_step(&bus); }
  CHECK(bus.nodes[moteur].state == NODE_BUS_OFF, "erreurs répétées : le calculateur passe en BUS-OFF (TEC = %d) et se déconnecte", bus.nodes[moteur].tec);

  /* 7. Codes défauts */
  char s[6]; dtc_decode(dtc_encode("P0217"), s);
  char s2_[6]; dtc_decode(dtc_encode("U0100"), s2_);
  CHECK(dtc_encode("P0217") == 0x0217 && !strcmp(s, "P0217") && !strcmp(s2_, "U0100"), "codes défauts : P0217 ↔ 0x0217, U0100 ↔ 0xC100");

  /* 8. OBD-II mode 01 */
  vehicle_t v = { 1726, 87, 92, 34, 40, {0x0217}, 1, "VF1RFB00X12345678" };
  uint8_t req[] = { 0x01, 0x0C }, resp[64]; char txt[160];
  int rl = obd_handle(&v, req, 2, resp, sizeof resp);
  obd_describe(resp, rl, txt, sizeof txt);
  CHECK(rl == 4 && resp[0] == 0x41 && !strcmp(txt, "Regime moteur : 1726 tr/min"), "OBD 01 0C : réponse 41 0C %02X %02X → \"%s\"", resp[2], resp[3], txt);
  uint8_t r03[] = { 0x03 };
  rl = obd_handle(&v, r03, 1, resp, sizeof resp); obd_describe(resp, rl, txt, sizeof txt);
  CHECK(strstr(txt, "P0217") != NULL, "OBD 03 : \"%s\"", txt);

  /* 9. ISO-TP : VIN (20 octets) = First Frame + contrôle de flux + 2 Consecutive Frames */
  uint8_t r09[] = { 0x09, 0x02 };
  rl = obd_handle(&v, r09, 2, resp, sizeof resp);
  isotp_tx_t tx; isotp_rx_t rx = {0}; can_frame_t fr, fc; int send_fc, frames = 1, done = 0;
  isotp_tx_start(&tx, OBD_ECU_RESP_ID, resp, (uint16_t)rl, &fr);
  done = isotp_rx_feed(&rx, &fr, &fc, OBD_ECU_REQ_ID, &send_fc);
  if (send_fc) isotp_tx_flow_control(&tx, &fc);
  while (!done && isotp_tx_next(&tx, &fr)) { frames++; done = isotp_rx_feed(&rx, &fr, &fc, OBD_ECU_REQ_ID, &send_fc); }
  obd_describe(rx.buf, rx.len, txt, sizeof txt);
  CHECK(done && rl == 20 && frames == 3 && !strcmp(txt, "VIN : VF1RFB00X12345678"),
        "ISO-TP : réponse de %d octets découpée en %d trames + contrôle de flux → \"%s\"", rl, frames, txt);

  printf("\n%s : %d échec(s)\n", fails ? "ÉCHEC" : "TOUS LES TESTS PASSENT", fails);
  return fails != 0;
}
