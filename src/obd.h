/*
 * OBD-II diagnostics (SAE J1979) over CAN with ISO-TP transport (ISO 15765-2)
 *  - functional request 0x7DF, engine ECU response 0x7E8
 *  - mode 01: live data (RPM, speed, coolant temperature, throttle, load)
 *  - mode 03: read trouble codes        - mode 04: clear trouble codes
 *  - mode 09: vehicle identification number (VIN, 17 characters → ISO-TP multi-frame)
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

/* Trouble codes: "P0217" <-> 0x0217 */
uint16_t dtc_encode(const char *s);
void     dtc_decode(uint16_t v, char out[6]);
const char *dtc_description(uint16_t v);

/* ECU side: handles an OBD request (without the ISO-TP length byte) */
int  obd_handle(vehicle_t *v, const uint8_t *req, int len, uint8_t *resp, int max);
/* Diagnostic tool side: response → human-readable text */
void obd_describe(const uint8_t *resp, int len, char *out, int max);

/* --- ISO-TP: segmentation of messages longer than 7 bytes --- */
typedef struct { uint16_t id; uint8_t buf[64]; uint16_t len, pos; uint8_t sn; int waiting_fc; } isotp_tx_t;
typedef struct { uint8_t buf[64]; uint16_t len, got; uint8_t sn; int active; } isotp_rx_t;

/* Sender: first frame (Single Frame or First Frame) */
void isotp_tx_start(isotp_tx_t *t, uint16_t id, const uint8_t *data, uint16_t len, can_frame_t *out);
/* Sender: next frame (Consecutive Frame); 1 if a frame was produced, 0 if done or waiting for flow control */
int  isotp_tx_next(isotp_tx_t *t, can_frame_t *out);
void isotp_tx_flow_control(isotp_tx_t *t, const can_frame_t *fc);
/* Receiver: 1 = message complete; *send_fc = 1 if a flow control frame (fc) must be sent */
int  isotp_rx_feed(isotp_rx_t *r, const can_frame_t *in, can_frame_t *fc, uint16_t fc_id, int *send_fc);

#ifdef __cplusplus
}
#endif
