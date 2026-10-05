/*
 * PC tests for the CAN network and OBD-II diagnostics (same code as on the ESP32)
 *   gcc -O2 -Wall -Wextra -Isrc -o t test/test_can.c src/can.c src/can_bus.c src/obd.c && ./t
 */
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "can_bus.h"
#include "obd.h"

static int fails = 0;
#define CHECK(c, ...) do { int ok_ = (c); printf("%s ", ok_ ? "[OK]   " : "[FAIL] "); printf(__VA_ARGS__); \
  printf("\n"); if (!ok_) fails++; } while (0)

int main(void) {
  uint8_t bits[CAN_MAX_BITS], stf[CAN_MAX_BITS];

  /* 1. Encode / decode round trip */
  can_frame_t f = { 0x0C0, 8, { 0x1A, 0xF8, 0x5A, 0x00, 0xFF, 0x00, 0x42, 0x7E } }, g;
  int n = can_encode(&f, bits, stf);
  CHECK(can_decode(bits, n, &g) == CAN_OK && g.id == f.id && g.dlc == 8 && !memcmp(g.data, f.data, 8),
        "frame 0x0C0 (8 bytes): encoded into %d bits then decoded identically", n);

  /* 2. Bit stuffing: never more than 5 identical bits from SOF to the end of the CRC */
  can_frame_t z = { 0x000, 8, { 0 } };
  n = can_encode(&z, bits, stf);
  int nst = 0, maxrun = 0, run = 0;
  for (int i = 0; i < n - 10; i++) { nst += stf[i]; run = (i && bits[i] == bits[i - 1]) ? run + 1 : 1; if (run > maxrun) maxrun = run; }
  CHECK(maxrun <= 5 && nst > 10, "all-zero frame: %d stuff bits inserted, never more than 5 identical bits", nst);

  /* 3. The CRC detects any single-bit error (anywhere between SOF and the end of the CRC) */
  n = can_encode(&f, bits, NULL);
  int detected = 0, total = n - 10;
  for (int i = 0; i < total; i++) {
    uint8_t b2[CAN_MAX_BITS]; memcpy(b2, bits, n); b2[i] ^= 1;
    if (can_decode(b2, n, &g) != CAN_OK) detected++;
  }
  CHECK(detected == total, "single-bit error injected at each of the %d positions: %d detected", total, detected);

  /* 4. Two wrong bits: always detected as well (CRC-15, Hamming distance 6) */
  int det2 = 0, tot2 = 0;
  for (int i = 0; i < total; i += 3) for (int j = i + 1; j < total; j += 5) {
    uint8_t b2[CAN_MAX_BITS]; memcpy(b2, bits, n); b2[i] ^= 1; b2[j] ^= 1; tot2++;
    if (can_decode(b2, n, &g) != CAN_OK) det2++;
  }
  CHECK(det2 == tot2, "double-bit errors: %d / %d detected", det2, tot2);

  /* 5. Arbitration: the lowest ID wins, bit by bit */
  can_frame_t a = { 0x0C0, 0, {0} }, b = { 0x1A0, 0, {0} }, c = { 0x7DF, 0, {0} };
  const can_frame_t *cand[] = { &b, &c, &a };
  int lost[3];
  int w = can_arbitrate(cand, 3, lost);
  CHECK(w == 2 && lost[0] == 2 && lost[1] == 0, "arbitration 0x1A0 / 0x7DF / 0x0C0: 0x0C0 wins (0x7DF loses at bit 0, 0x1A0 at bit 2)");

  /* 6. Bus: arbitration, filters, retransmission after an error, fault confinement */
  can_bus_t bus; can_bus_init(&bus);
  int engine = can_bus_add_node(&bus, "engine", 0x7DF, 0x7F0);
  int abs_ = can_bus_add_node(&bus, "abs", 0, 0x000);
  int dash = can_bus_add_node(&bus, "dashboard", 0x000, 0x000);
  can_send(&bus, abs_, &b); can_send(&bus, engine, &a);
  can_bus_step(&bus);
  can_frame_t r; int got = can_receive(&bus, dash, &r);
  CHECK(got && r.id == 0x0C0, "bus: engine frame 0x0C0 goes first, the instrument cluster receives it");
  CHECK(!can_receive(&bus, engine, &r) || r.id != 0x0C0, "acceptance filter: the engine ignores frames not addressed to it");
  can_bus_step(&bus);
  bus.inject_error = 1;
  can_send(&bus, engine, &a);
  int s1 = can_bus_step(&bus), s2 = can_bus_step(&bus);
  CHECK(s1 == -1 && s2 == 1 && bus.frames_err == 1, "noise on the wire: error detected, frame automatically retransmitted");
  bus.inject_error = 1000;
  for (int i = 0; i < 40; i++) { can_send(&bus, engine, &a); can_bus_step(&bus); }
  CHECK(bus.nodes[engine].state == NODE_BUS_OFF, "repeated errors: the ECU goes BUS-OFF (TEC = %d) and disconnects", bus.nodes[engine].tec);

  /* 7. Trouble codes */
  char s[6]; dtc_decode(dtc_encode("P0217"), s);
  char s2_[6]; dtc_decode(dtc_encode("U0100"), s2_);
  CHECK(dtc_encode("P0217") == 0x0217 && !strcmp(s, "P0217") && !strcmp(s2_, "U0100"), "trouble codes: P0217 ↔ 0x0217, U0100 ↔ 0xC100");

  /* 8. OBD-II mode 01 */
  vehicle_t v = { 1726, 87, 92, 34, 40, {0x0217}, 1, "VF1RFB00X12345678" };
  uint8_t req[] = { 0x01, 0x0C }, resp[64]; char txt[160];
  int rl = obd_handle(&v, req, 2, resp, sizeof resp);
  obd_describe(resp, rl, txt, sizeof txt);
  CHECK(rl == 4 && resp[0] == 0x41 && !strcmp(txt, "Engine speed: 1726 rpm"), "OBD 01 0C: response 41 0C %02X %02X → \"%s\"", resp[2], resp[3], txt);
  uint8_t r03[] = { 0x03 };
  rl = obd_handle(&v, r03, 1, resp, sizeof resp); obd_describe(resp, rl, txt, sizeof txt);
  CHECK(strstr(txt, "P0217") != NULL, "OBD 03: \"%s\"", txt);

  /* 9. ISO-TP: VIN (20 bytes) = First Frame + flow control + 2 Consecutive Frames */
  uint8_t r09[] = { 0x09, 0x02 };
  rl = obd_handle(&v, r09, 2, resp, sizeof resp);
  isotp_tx_t tx; isotp_rx_t rx = {0}; can_frame_t fr, fc; int send_fc, frames = 1, done = 0;
  isotp_tx_start(&tx, OBD_ECU_RESP_ID, resp, (uint16_t)rl, &fr);
  done = isotp_rx_feed(&rx, &fr, &fc, OBD_ECU_REQ_ID, &send_fc);
  if (send_fc) isotp_tx_flow_control(&tx, &fc);
  while (!done && isotp_tx_next(&tx, &fr)) { frames++; done = isotp_rx_feed(&rx, &fr, &fc, OBD_ECU_REQ_ID, &send_fc); }
  obd_describe(rx.buf, rx.len, txt, sizeof txt);
  CHECK(done && rl == 20 && frames == 3 && !strcmp(txt, "VIN: VF1RFB00X12345678"),
        "ISO-TP: %d-byte response split into %d frames + flow control → \"%s\"", rl, frames, txt);

  printf("\n%s: %d failure(s)\n", fails ? "FAILED" : "ALL TESTS PASSED", fails);
  return fails != 0;
}
