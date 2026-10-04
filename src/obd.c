#include "obd.h"
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 *  Codes défauts (DTC) : 2 octets. Les 2 premiers bits donnent la lettre
 *  (P moteur, C châssis, B carrosserie, U réseau), puis 4 chiffres.
 * ------------------------------------------------------------------------- */
uint16_t dtc_encode(const char *s) {
  static const char L[] = "PCBU";
  const char *p = strchr(L, s[0]);
  uint16_t v = (uint16_t)((p ? p - L : 0) << 14);
  v |= (uint16_t)((s[1] - '0') & 3) << 12;
  for (int i = 2; i < 5; i++) {
    char c = s[i];
    int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : 0;
    v |= (uint16_t)d << (4 * (4 - i));
  }
  return v;
}

void dtc_decode(uint16_t v, char out[6]) {
  static const char L[] = "PCBU";
  snprintf(out, 6, "%c%d%03X", L[v >> 14], (v >> 12) & 3, v & 0xFFF);
}

const char *dtc_description(uint16_t v) {
  switch (v) {
    case 0x0217: return "surchauffe du moteur";
    case 0x0480: return "commande du ventilateur 1 defaillante";
    case 0x0300: return "rates d'allumage detectes";
    case 0x0500: return "capteur de vitesse du vehicule";
    case 0xC100: return "perte de communication avec le calculateur moteur";
  }
  return "code constructeur";
}

/* ---------------------------------------------------------------------------
 *  Calculateur moteur : réponse aux requêtes OBD-II
 * ------------------------------------------------------------------------- */
int obd_handle(vehicle_t *v, const uint8_t *req, int len, uint8_t *r, int max) {
  if (len < 1 || max < 8) return 0;
  uint8_t mode = req[0];
  int n = 0;
  r[n++] = mode + 0x40;                                         /* réponse positive = mode + 0x40 */
  if (mode == 0x01 && len >= 2) {
    uint8_t pid = req[1];
    r[n++] = pid;
    switch (pid) {
      case 0x00: r[n++] = 0x18; r[n++] = 0x18; r[n++] = 0x80; r[n++] = 0x00; break;  /* PID supportés : 04 05 0C 0D 11 */
      case 0x04: r[n++] = (uint8_t)(v->load_pct * 255 / 100); break;
      case 0x05: r[n++] = (uint8_t)(v->coolant_c + 40); break;                       /* A - 40 °C */
      case 0x0C: { uint16_t x = (uint16_t)(v->rpm * 4); r[n++] = x >> 8; r[n++] = x & 0xFF; break; }  /* (256A+B)/4 */
      case 0x0D: r[n++] = (uint8_t)v->speed_kmh; break;
      case 0x11: r[n++] = (uint8_t)(v->throttle_pct * 255 / 100); break;
      default:   r[0] = 0x7F; r[1] = mode; r[2] = 0x12; return 3;                     /* PID non supporté */
    }
  } else if (mode == 0x03) {
    r[n++] = (uint8_t)v->n_dtc;
    for (int i = 0; i < v->n_dtc && n + 2 <= max; i++) { r[n++] = v->dtc[i] >> 8; r[n++] = v->dtc[i] & 0xFF; }
  } else if (mode == 0x04) {
    v->n_dtc = 0;                                               /* effacement des défauts */
  } else if (mode == 0x09 && len >= 2 && req[1] == 0x02) {
    r[n++] = 0x02; r[n++] = 0x01;                               /* 1 VIN */
    for (int i = 0; i < 17; i++) r[n++] = (uint8_t)v->vin[i];
  } else {
    r[0] = 0x7F; r[1] = mode; r[2] = 0x11; return 3;            /* service non supporté */
  }
  return n;
}

void obd_describe(const uint8_t *r, int len, char *out, int max) {
  if (len >= 3 && r[0] == 0x7F) { snprintf(out, max, "refus du calculateur (code 0x%02X)", r[2]); return; }
  if (len >= 3 && r[0] == 0x41) {
    switch (r[1]) {
      case 0x0C: snprintf(out, max, "Regime moteur : %d tr/min", (r[2] * 256 + r[3]) / 4); return;
      case 0x0D: snprintf(out, max, "Vitesse : %d km/h", r[2]); return;
      case 0x05: snprintf(out, max, "Temperature liquide de refroidissement : %d C", r[2] - 40); return;
      case 0x11: snprintf(out, max, "Position papillon : %d %%", r[2] * 100 / 255); return;
      case 0x04: snprintf(out, max, "Charge moteur : %d %%", r[2] * 100 / 255); return;
      case 0x00: snprintf(out, max, "PID supportes : %02X %02X %02X %02X", r[2], r[3], r[4], r[5]); return;
    }
  }
  if (len >= 2 && r[0] == 0x43) {
    int n = r[1], w = snprintf(out, max, "%d code(s) defaut", n);
    for (int i = 0; i < n && 2 + 2 * i + 1 < len && w < max; i++) {
      char c[6]; uint16_t v = (uint16_t)(r[2 + 2 * i] << 8 | r[3 + 2 * i]);
      dtc_decode(v, c);
      w += snprintf(out + w, max - w, " | %s %s", c, dtc_description(v));
    }
    return;
  }
  if (len >= 1 && r[0] == 0x44) { snprintf(out, max, "Codes defaut effaces, voyant moteur eteint"); return; }
  if (len >= 20 && r[0] == 0x49 && r[1] == 0x02) { snprintf(out, max, "VIN : %.17s", (const char *)r + 3); return; }
  snprintf(out, max, "reponse inconnue");
}

/* ---------------------------------------------------------------------------
 *  ISO-TP : SF (simple, ≤ 7 octets) | FF (1re trame, longueur) + CF (suites)
 *  Le récepteur répond à la FF par un contrôle de flux (FC) avant les CF.
 * ------------------------------------------------------------------------- */
void isotp_tx_start(isotp_tx_t *t, uint16_t id, const uint8_t *data, uint16_t len, can_frame_t *out) {
  t->id = id; t->len = len > sizeof t->buf ? sizeof t->buf : len;
  memcpy(t->buf, data, t->len);
  memset(out, 0, sizeof *out);
  out->id = id; out->dlc = 8;
  if (t->len <= 7) {                                         /* Single Frame */
    out->data[0] = (uint8_t)t->len;
    memcpy(out->data + 1, t->buf, t->len);
    t->pos = t->len; t->waiting_fc = 0;
  } else {                                                   /* First Frame */
    out->data[0] = 0x10 | (uint8_t)(t->len >> 8);
    out->data[1] = t->len & 0xFF;
    memcpy(out->data + 2, t->buf, 6);
    t->pos = 6; t->sn = 1; t->waiting_fc = 1;
  }
}

void isotp_tx_flow_control(isotp_tx_t *t, const can_frame_t *fc) {
  if ((fc->data[0] & 0xF0) == 0x30 && (fc->data[0] & 0x0F) == 0) t->waiting_fc = 0;   /* « continue » */
}

int isotp_tx_next(isotp_tx_t *t, can_frame_t *out) {
  if (t->waiting_fc || t->pos >= t->len) return 0;
  memset(out, 0, sizeof *out);
  out->id = t->id; out->dlc = 8;
  out->data[0] = 0x20 | (t->sn & 0x0F);                      /* Consecutive Frame + numéro */
  int k = t->len - t->pos < 7 ? t->len - t->pos : 7;
  memcpy(out->data + 1, t->buf + t->pos, k);
  t->pos += k; t->sn++;
  return 1;
}

int isotp_rx_feed(isotp_rx_t *r, const can_frame_t *in, can_frame_t *fc, uint16_t fc_id, int *send_fc) {
  *send_fc = 0;
  uint8_t type = in->data[0] >> 4;
  if (type == 0) {                                           /* Single Frame */
    r->len = in->data[0] & 0x0F;
    if (r->len > 7) return 0;
    memcpy(r->buf, in->data + 1, r->len); r->active = 0;
    return 1;
  }
  if (type == 1) {                                           /* First Frame */
    r->len = (uint16_t)((in->data[0] & 0x0F) << 8 | in->data[1]);
    if (r->len > sizeof r->buf) r->len = sizeof r->buf;
    memcpy(r->buf, in->data + 2, 6); r->got = 6; r->sn = 1; r->active = 1;
    memset(fc, 0, sizeof *fc);
    fc->id = fc_id; fc->dlc = 8; fc->data[0] = 0x30;         /* FC : continue, sans limite, sans délai */
    *send_fc = 1;
    return 0;
  }
  if (type == 2 && r->active) {                              /* Consecutive Frame */
    if ((in->data[0] & 0x0F) != (r->sn & 0x0F)) { r->active = 0; return 0; }   /* trame perdue */
    int k = r->len - r->got < 7 ? r->len - r->got : 7;
    memcpy(r->buf + r->got, in->data + 1, k);
    r->got += k; r->sn++;
    if (r->got >= r->len) { r->active = 0; return 1; }
  }
  return 0;
}
