#include "can.h"
#include <string.h>

uint16_t can_crc15(const uint8_t *bits, int n) {
  uint16_t crc = 0;
  for (int i = 0; i < n; i++) {
    int nxt = bits[i] ^ ((crc >> 14) & 1);
    crc = (uint16_t)((crc << 1) & 0x7FFF);
    if (nxt) crc ^= 0x4599;
  }
  return crc;
}

static int put(uint8_t *b, int n, uint32_t v, int width) {
  for (int i = width - 1; i >= 0; i--) b[n++] = (v >> i) & 1;
  return n;
}

int can_encode(const can_frame_t *f, uint8_t *bits, uint8_t *stuff) {
  uint8_t raw[CAN_MAX_BITS];
  int n = 0, dlen = f->dlc > 8 ? 8 : f->dlc;
  raw[n++] = 0;                                    /* SOF */
  n = put(raw, n, f->id & 0x7FF, 11);
  raw[n++] = 0; raw[n++] = 0; raw[n++] = 0;        /* RTR, IDE, r0 */
  n = put(raw, n, f->dlc, 4);
  for (int i = 0; i < dlen; i++) n = put(raw, n, f->data[i], 8);
  n = put(raw, n, can_crc15(raw, n), 15);

  int m = 0, run = 0, last = -1;                   /* bit stuffing from SOF to the end of the CRC */
  for (int i = 0; i < n; i++) {
    bits[m] = raw[i]; if (stuff) stuff[m] = 0; m++;
    run = (raw[i] == last) ? run + 1 : 1; last = raw[i];
    if (run == 5) { bits[m] = !raw[i]; if (stuff) stuff[m] = 1; m++; last = !raw[i]; run = 1; }
  }
  static const uint8_t tail[] = {1, 0, 1, 1, 1, 1, 1, 1, 1, 1};   /* CRC delim, ACK (acknowledged), ACK delim, EOF */
  for (unsigned i = 0; i < sizeof tail; i++) { bits[m] = tail[i]; if (stuff) stuff[m] = 0; m++; }
  return m;
}

can_err_t can_decode(const uint8_t *bits, int n, can_frame_t *out) {
  uint8_t raw[CAN_MAX_BITS];
  int r = 0, i = 0, run = 0, last = -1, need = 19;           /* 19 = SOF..DLC */
  while (r < need) {
    if (i >= n) return CAN_ERR_FORM;
    int b = bits[i++];
    run = (b == last) ? run + 1 : 1; last = b;
    if (run == 6) return CAN_ERR_STUFF;                       /* 6 identical bits: forbidden */
    raw[r++] = (uint8_t)b;
    if (run == 5) {                                           /* the next bit must be a stuff bit */
      if (i >= n) return CAN_ERR_FORM;
      int s = bits[i++];
      if (s == b) return CAN_ERR_STUFF;
      last = s; run = 1;
    }
    if (r == 19) {
      int dlc = 0; for (int k = 15; k < 19; k++) dlc = dlc << 1 | raw[k];
      need = 19 + 8 * (dlc > 8 ? 8 : dlc) + 15;
    }
  }
  if (raw[0] != 0) return CAN_ERR_FORM;
  if (can_crc15(raw, need) != 0) return CAN_ERR_CRC;          /* CRC included: remainder is zero if intact */
  if (i + 10 > n || bits[i] != 1 || bits[i + 2] != 1) return CAN_ERR_FORM;
  for (int k = 3; k < 10; k++) if (bits[i + k] != 1) return CAN_ERR_FORM;

  int id = 0; for (int k = 1; k < 12; k++) id = id << 1 | raw[k];
  int dlc = 0; for (int k = 15; k < 19; k++) dlc = dlc << 1 | raw[k];
  out->id = (uint16_t)id; out->dlc = (uint8_t)dlc;
  memset(out->data, 0, 8);
  for (int b = 0; b < (dlc > 8 ? 8 : dlc); b++)
    for (int k = 0; k < 8; k++) out->data[b] = (uint8_t)(out->data[b] << 1 | raw[19 + 8 * b + k]);
  return CAN_OK;
}

const char *can_err_str(can_err_t e) {
  switch (e) {
    case CAN_OK: return "OK";
    case CAN_ERR_STUFF: return "stuff error (6 identical bits)";
    case CAN_ERR_CRC: return "CRC error";
    case CAN_ERR_FORM: return "form error";
  }
  return "?";
}

int can_arbitrate(const can_frame_t *const *c, int n, int *lost_at) {
  int alive[16], winner = -1;
  for (int i = 0; i < n; i++) { alive[i] = 1; if (lost_at) lost_at[i] = -1; }
  for (int bit = 10; bit >= 0; bit--) {                       /* ID sent from the most significant bit to the least */
    int bus = 1;
    for (int i = 0; i < n; i++) if (alive[i]) bus &= (c[i]->id >> bit) & 1;   /* wired AND */
    for (int i = 0; i < n; i++)
      if (alive[i] && ((c[i]->id >> bit) & 1) != bus) { alive[i] = 0; if (lost_at) lost_at[i] = 10 - bit; }
  }
  for (int i = 0; i < n; i++) if (alive[i]) { winner = i; break; }
  return winner;
}
