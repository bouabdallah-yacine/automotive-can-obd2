/*
 * ============================================================================
 *  Automotive CAN network + OBD-II diagnostics (ESP32 + FreeRTOS, Wokwi)
 * ============================================================================
 *  4 nodes on a virtual 500 kbit/s CAN bus, each in its own FreeRTOS task:
 *
 *   Engine ECU   0x0C0 / 50 ms: RPM, coolant temperature, throttle, check-engine light
 *                answers OBD-II diagnostics (0x7DF / 0x7E0 → 0x7E8)
 *   ABS ECU      0x1A0 / 50 ms: vehicle speed, brake, ABS active
 *   Instrument   receives everything, shows it on the OLED, detects loss of
 *   cluster      communication with the engine
 *   Diagnostic   OBD commands typed in the serial monitor
 *   tool
 *
 *  The bus transmits every frame BIT BY BIT (CRC-15, bit stuffing, arbitration,
 *  error frames, TEC/REC counters, bus-off): portable C code tested on a PC.
 *
 *  Web dashboard served by the ESP32 (http://localhost:8181 with Wokwi): gauges,
 *  warning lights, live bus sniffer, ECU status, OBD-II diagnostics.
 *
 *  Simulated hardware: potentiometer = accelerator pedal, red button = brake,
 *  yellow button = cooling fan failure.
 * ============================================================================
 */
#include <Arduino.h>
#include <stdarg.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "can.h"
#include "can_bus.h"
#include "obd.h"
#include "web_page.h"

#define PIN_PEDAL   34
#define PIN_BRAKE   26
#define PIN_FANFAIL 27
#define PIN_LED_MIL 2      // check-engine light (orange)
#define PIN_LED_ABS 4      // ABS warning light (yellow)

#define ID_ENGINE   0x0C0
#define ID_ABS      0x1A0

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

can_bus_t bus;
SemaphoreHandle_t busMutex;
int N_ENGINE, N_ABS, N_DASH, N_DIAG;
volatile bool sniff = false;
QueueHandle_t sniffQ;                       // frames to print (without blocking the bus)
uint32_t busStartMs;

static int busSend(int node, const can_frame_t *f) {
  xSemaphoreTake(busMutex, portMAX_DELAY); int r = can_send(&bus, node, f); xSemaphoreGive(busMutex); return r;
}
static int busRecv(int node, can_frame_t *f) {
  xSemaphoreTake(busMutex, portMAX_DELAY); int r = can_receive(&bus, node, f); xSemaphoreGive(busMutex); return r;
}

// ---------------------------------------------------------------------------
//  The bus: transmits pending frames (≈ 4 frames max per ms at 500 kbit/s)
// ---------------------------------------------------------------------------
struct SniffItem { can_frame_t f; uint8_t sender; int8_t status; uint32_t t; };
#define WEB_FRAMES 40
SniffItem webFrames[WEB_FRAMES];             // latest frames for the web dashboard (protected by busMutex)
uint32_t webSeq = 0;
WebServer server(80);
volatile int webPedal = -1;                  // pedal driven from the web page (-1 = potentiometer)
volatile uint32_t webBrakeUntil = 0;
SemaphoreHandle_t diagMutex;                 // only one diagnostic request at a time

void taskBus(void *) {
  uint32_t busOffSince[CAN_MAX_NODES] = {0};
  for (;;) {
    for (int k = 0; k < 4; k++) {
      xSemaphoreTake(busMutex, portMAX_DELAY);
      int r = can_bus_step(&bus);
      SniffItem it = { bus.last_frame, (uint8_t)bus.last_sender, (int8_t)r, (uint32_t)millis() };
      if (r != 0) webFrames[webSeq++ % WEB_FRAMES] = it;
      // Automatic recovery of a bus-off node after 3 s
      for (int i = 0; i < bus.n_nodes; i++) {
        if (bus.nodes[i].state == NODE_BUS_OFF) {
          if (!busOffSince[i]) {
            busOffSince[i] = millis();
            if (i == bus.inject_node) bus.inject_error = 0;        // the faulty node is isolated
            Serial.printf("# %s: TEC = %u -> BUS-OFF, the ECU disconnects itself to protect the network\n", bus.nodes[i].name, bus.nodes[i].tec);
          } else if (millis() - busOffSince[i] > 3000) {
            bus.nodes[i].tec = bus.nodes[i].rec = 0; bus.nodes[i].state = NODE_ERROR_ACTIVE; busOffSince[i] = 0;
            Serial.printf("# %s: recovered from BUS-OFF, back on the network\n", bus.nodes[i].name);
          }
        }
      }
      xSemaphoreGive(busMutex);
      if (r == 0) break;
      if (sniff) xQueueSend(sniffQ, &it, 0);
    }
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------
//  Engine ECU: simplified physics + 0x0C0 frame + OBD-II server
// ---------------------------------------------------------------------------
vehicle_t veh = { 800, 0, 25, 0, 15, {0}, 0, "VF1PFE2026EMB0042" };
volatile bool fanFail = false;
volatile bool engineLostComm = false;

static void addDtc(uint16_t code) {
  for (int i = 0; i < veh.n_dtc; i++) if (veh.dtc[i] == code) return;
  if (veh.n_dtc < 8) veh.dtc[veh.n_dtc++] = code;
}

void taskEngine(void *) {
  TickType_t last = xTaskGetTickCount();
  isotp_tx_t tx; bool txActive = false;
  uint32_t k = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    // Physics (100 Hz)
    float pedal = webPedal >= 0 ? (float)webPedal : analogRead(PIN_PEDAL) / 4095.0f * 100.0f;
    veh.throttle_pct = pedal;
    float rpmTarget = 800 + pedal * 55;
    veh.rpm += (rpmTarget - veh.rpm) * 0.05f;
    veh.load_pct = 15 + pedal * 0.8f;
    float tTarget = fanFail ? 60 + veh.rpm / 40 : fminf(90, 25 + veh.rpm / 60);   // thermostat at 90 °C
    veh.coolant_c += (tTarget - veh.coolant_c) * (fanFail ? 0.004f : 0.002f);
    if (fanFail) addDtc(0x0480);
    if (veh.coolant_c > 110) addDtc(0x0217);
    bool mil = veh.n_dtc > 0;
    digitalWrite(PIN_LED_MIL, mil);

    // Periodic 0x0C0 frame every 50 ms
    if (++k % 5 == 0) {
      can_frame_t f = { ID_ENGINE, 6, {0} };
      uint16_t r = (uint16_t)veh.rpm;
      f.data[0] = r >> 8; f.data[1] = r & 0xFF;
      f.data[2] = (uint8_t)(veh.coolant_c + 40);
      f.data[3] = (uint8_t)(veh.throttle_pct * 2.55f);
      f.data[4] = mil;
      f.data[5] = (uint8_t)veh.n_dtc;
      busSend(N_ENGINE, &f);
    }

    // Diagnostics: 0x7DF / 0x7E0 requests, 0x7E8 responses (ISO-TP)
    can_frame_t in;
    while (busRecv(N_ENGINE, &in)) {
      if (in.id == ID_ABS) veh.speed_kmh = (in.data[0] << 8 | in.data[1]) / 100.0f;
      else if ((in.id == OBD_REQ_ID || in.id == OBD_ECU_REQ_ID) && (in.data[0] >> 4) == 0) {
        uint8_t resp[64];
        int n = obd_handle(&veh, in.data + 1, in.data[0] & 0x0F, resp, sizeof resp);
        if (n > 0) { can_frame_t out; isotp_tx_start(&tx, OBD_ECU_RESP_ID, resp, n, &out); busSend(N_ENGINE, &out); txActive = true; }
      } else if (in.id == OBD_ECU_REQ_ID && (in.data[0] >> 4) == 3 && txActive) {
        isotp_tx_flow_control(&tx, &in);                     // flow control received → send the remaining frames
      }
    }
    can_frame_t cf;
    while (txActive && isotp_tx_next(&tx, &cf)) busSend(N_ENGINE, &cf);
  }
}

// ---------------------------------------------------------------------------
//  ABS ECU: vehicle speed, braking, ABS
// ---------------------------------------------------------------------------
volatile bool absActive = false;

void taskAbs(void *) {
  TickType_t last = xTaskGetTickCount();
  float speed = 0, throttle = 0; uint32_t k = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    can_frame_t in;
    while (busRecv(N_ABS, &in)) if (in.id == ID_ENGINE) throttle = in.data[3] / 2.55f;
    bool brake = digitalRead(PIN_BRAKE) == LOW || millis() < webBrakeUntil;
    float accel = throttle * 0.15f - 0.0004f * speed * speed - (brake ? 30.0f : 0.5f);   // km/h per second
    speed = fmaxf(0, speed + accel * 0.01f);
    absActive = brake && speed > 30;                                      // hard braking at high speed
    digitalWrite(PIN_LED_ABS, absActive && (millis() / 120) % 2);
    if (++k % 5 == 0) {
      can_frame_t f = { ID_ABS, 4, {0} };
      uint16_t s = (uint16_t)(speed * 100);
      f.data[0] = s >> 8; f.data[1] = s & 0xFF; f.data[2] = brake; f.data[3] = absActive;
      busSend(N_ABS, &f);
    }
  }
}

// ---------------------------------------------------------------------------
//  Instrument cluster: OLED display + network monitoring
// ---------------------------------------------------------------------------
struct DashState { float rpm, temp, speed; bool mil, absOn, brake, lost; int nDtc; };
volatile DashState dashState = {};           // what the instrument cluster read from the bus

void taskDash(void *) {
  float rpm = 0, temp = 0, speed = 0; bool mil = false, absOn = false, brake = false;
  uint32_t lastEngine = millis(); int nDtc = 0;
  for (;;) {
    can_frame_t in;
    while (busRecv(N_DASH, &in)) {
      if (in.id == ID_ENGINE) { rpm = in.data[0] << 8 | in.data[1]; temp = in.data[2] - 40.0f; mil = in.data[4]; nDtc = in.data[5]; lastEngine = millis(); }
      if (in.id == ID_ABS) { speed = (in.data[0] << 8 | in.data[1]) / 100.0f; brake = in.data[2]; absOn = in.data[3]; }
    }
    bool lost = millis() - lastEngine > 500;                  // no 0x0C0 for 500 ms
    if (lost && !engineLostComm) Serial.println("# INSTRUMENT CLUSTER: lost communication with the engine (U0100)");
    engineLostComm = lost;
    dashState.rpm = rpm; dashState.temp = temp; dashState.speed = speed; dashState.mil = mil;
    dashState.absOn = absOn; dashState.brake = brake; dashState.lost = lost; dashState.nDtc = nDtc;

    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2); oled.setCursor(0, 0); oled.printf("%3.0f", speed);
    oled.setTextSize(1); oled.setCursor(38, 7); oled.print("km/h");
    oled.setCursor(74, 0); oled.printf("%4.0f rpm", rpm);
    oled.setCursor(74, 9); oled.printf("%3.0f C", temp);
    oled.drawRect(0, 20, 128, 6, SSD1306_WHITE);                       // temperature gauge
    oled.fillRect(1, 21, (int)constrain((temp - 40) / 90.0f * 126, 0.0f, 126.0f), 4, SSD1306_WHITE);
    oled.setCursor(0, 30);
    if (lost) oled.print("ENGINE COMM LOST U0100");
    else if (mil) { oled.fillRect(0, 29, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); oled.printf(" CHECK ENGINE (%d)", nDtc); oled.setTextColor(SSD1306_WHITE); }
    oled.setCursor(0, 42); if (absOn) oled.print("ABS ACTIVE");
    xSemaphoreTake(busMutex, portMAX_DELAY);
    float elapsed = (millis() - busStartMs) / 1000.0f;
    float load = elapsed > 0 ? bus.bits_total / (elapsed * CAN_BITRATE) * 100 : 0;
    uint32_t err = bus.frames_err;
    xSemaphoreGive(busMutex);
    oled.setCursor(0, 56); oled.printf("CAN %.1f%%  err %lu", load, (unsigned long)err);
    oled.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---------------------------------------------------------------------------
//  Diagnostic tool: OBD-II commands typed in the serial monitor
// ---------------------------------------------------------------------------
static void printBits() {
  xSemaphoreTake(busMutex, portMAX_DELAY);
  int n = bus.last_nbits; uint8_t b[CAN_MAX_BITS], s[CAN_MAX_BITS]; memcpy(b, bus.last_bits, n); memcpy(s, bus.last_stuff, n);
  can_frame_t f = bus.last_frame;
  xSemaphoreGive(busMutex);
  Serial.printf("Last frame: ID 0x%03X, %d bytes, %d bits on the wire (stuff bits marked ^)\n", f.id, f.dlc, n);
  String l1, l2;
  for (int i = 0; i < n; i++) { l1 += (char)('0' + b[i]); l2 += s[i] ? '^' : ' '; }
  Serial.println(l1); Serial.println(l2);
  Serial.println("Order: SOF | ID (11 bits) | RTR IDE r0 | DLC (4) | data | CRC-15 | delim | ACK | delim | EOF (7 x 1)");
}

static void printStats() {
  xSemaphoreTake(busMutex, portMAX_DELAY);
  float elapsed = (millis() - busStartMs) / 1000.0f;
  Serial.printf("CAN bus 500 kbit/s: load %.1f %%, %lu frames OK, %lu errors, %lu arbitrations\n",
                bus.bits_total / (elapsed * CAN_BITRATE) * 100, (unsigned long)bus.frames_ok,
                (unsigned long)bus.frames_err, (unsigned long)bus.arbitrations);
  for (int i = 0; i < bus.n_nodes; i++)
    Serial.printf("  %-12s TEC %3u  REC %3u  state: %s\n", bus.nodes[i].name, bus.nodes[i].tec, bus.nodes[i].rec, can_state_str(bus.nodes[i].state));
  if (bus.arb_winner >= 0)
    Serial.printf("Last arbitration: 0x%03X (%s) won against 0x%03X (%s), which lost at identifier bit %d\n",
                  bus.arb_win_id, bus.nodes[bus.arb_winner].name, bus.arb_lose_id, bus.nodes[bus.arb_loser].name, bus.arb_lost_bit);
  xSemaphoreGive(busMutex);
}

// Result of a request (for the web page); the same lines are also printed to the serial monitor
struct ObdResult { char lines[8][72]; int n; char text[200]; };

static void obdLine(ObdResult *r, const char *fmt, ...) {
  char buf[96]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
  Serial.println(buf);
  if (r && r->n < 8) strlcpy(r->lines[r->n++], buf, sizeof r->lines[0]);
}

static void hexFrame(char *out, size_t n, const char *dir, uint16_t id, const uint8_t *d, int len) {
  int w = snprintf(out, n, "%s 0x%03X :", dir, id);
  for (int i = 0; i < len && w < (int)n - 4; i++) w += snprintf(out + w, n - w, " %02X", d[i]);
}

static void obdRequest(const uint8_t *req, int len, ObdResult *res = nullptr) {
  xSemaphoreTake(diagMutex, portMAX_DELAY);
  if (res) { res->n = 0; res->text[0] = 0; }
  can_frame_t f = { OBD_REQ_ID, 8, {0} };
  f.data[0] = (uint8_t)len; memcpy(f.data + 1, req, len);
  char line[96];
  hexFrame(line, sizeof line, "->", f.id, f.data, len + 1); obdLine(res, "%s", line);
  can_frame_t junk; while (busRecv(N_DIAG, &junk)) {}               // flush stale responses
  busSend(N_DIAG, &f);
  isotp_rx_t rx = {}; uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    can_frame_t in, fc; int sendFc;
    if (busRecv(N_DIAG, &in)) {
      hexFrame(line, sizeof line, "<-", in.id, in.data, in.dlc); obdLine(res, "%s", line);
      if (isotp_rx_feed(&rx, &in, &fc, OBD_ECU_REQ_ID, &sendFc)) {
        char txt[200]; obd_describe(rx.buf, rx.len, txt, sizeof txt);
        Serial.printf("   %s\n", txt);
        if (res) strlcpy(res->text, txt, sizeof res->text);
        xSemaphoreGive(diagMutex);
        return;
      }
      if (sendFc) { obdLine(res, "-> 0x7E0 : 30 00 00   (ISO-TP flow control)"); busSend(N_DIAG, &fc); }
    }
    vTaskDelay(2);
  }
  Serial.println("   no response (engine ECU offline?)");
  if (res) strlcpy(res->text, "no response (engine ECU offline?)", sizeof res->text);
  xSemaphoreGive(diagMutex);
}

// Human-readable shortcuts → OBD-II request
static int obdShortcut(const String &q, uint8_t *req) {
  if (q == "rpm") { req[0] = 1; req[1] = 0x0C; return 2; }
  if (q == "temp") { req[0] = 1; req[1] = 0x05; return 2; }
  if (q == "speed") { req[0] = 1; req[1] = 0x0D; return 2; }
  if (q == "throttle") { req[0] = 1; req[1] = 0x11; return 2; }
  if (q == "dtc") { req[0] = 3; return 1; }
  if (q == "clear") { req[0] = 4; return 1; }
  if (q == "vin") { req[0] = 9; req[1] = 2; return 2; }
  return 0;
}

static void help() {
  Serial.println("\nOBD-II diagnostic tool:");
  Serial.println("  rpm | temp | speed | throttle | dtc | clear | vin   (or in hex: 01 0C, 03, 09 02...)");
  Serial.println("  sniff (monitor the bus) | bits (last frame bit by bit) | stats | error <n> (inject noise)");
}

void taskDiag(void *) {
  String line;
  help();
  for (;;) {
    SniffItem it;
    while (xQueueReceive(sniffQ, &it, 0) == pdTRUE) {
      Serial.printf("[%7.3f] 0x%03X %-8s %d ", it.t / 1000.0f, it.f.id, bus.nodes[it.sender].name, it.f.dlc);
      for (int i = 0; i < it.f.dlc && i < 8; i++) Serial.printf(" %02X", it.f.data[i]);
      Serial.println(it.status < 0 ? "   <- ERROR detected, retransmitting" : "");
    }
    while (Serial.available()) {
      char c = Serial.read();
      if (c != '\n' && c != '\r') { if (line.length() < 40) line += c; continue; }
      line.trim(); line.toLowerCase();
      if (!line.length()) continue;
      uint8_t req[8]; int n = obdShortcut(line, req);
      if (n) {}
      else if (line == "sniff") { sniff = !sniff; Serial.printf("# bus sniffer: %s\n", sniff ? "ON" : "OFF"); }
      else if (line == "bits") printBits();
      else if (line == "stats") printStats();
      else if (line.startsWith("error")) {
        int k = line.length() > 6 ? line.substring(6).toInt() : 1;
        xSemaphoreTake(busMutex, portMAX_DELAY); bus.inject_error = k; bus.inject_node = N_ENGINE; xSemaphoreGive(busMutex);
        Serial.printf("# %d noise burst(s) on the engine ECU frames (try error 40 for BUS-OFF)\n", k);
      }
      else {                                                   // bytes in hexadecimal: "01 0c"
        char buf[48]; line.toCharArray(buf, sizeof buf);
        for (char *t = strtok(buf, " "); t && n < 7; t = strtok(nullptr, " ")) req[n++] = (uint8_t)strtol(t, nullptr, 16);
        if (!n) help();
      }
      if (n) obdRequest(req, n);
      line = "";
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ---------------------------------------------------------------------------
void taskButtons(void *) {
  bool lastFan = HIGH;
  for (;;) {
    bool b = digitalRead(PIN_FANFAIL);
    if (b == LOW && lastFan == HIGH) {
      fanFail = !fanFail;
      Serial.printf("# Cooling fan: %s\n", fanFail ? "FAILED" : "repaired");
    }
    lastFan = b;
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ---------------------------------------------------------------------------
//  Web dashboard (served by the ESP32)
// ---------------------------------------------------------------------------
static void webState() {
  static char buf[6144];
  uint32_t since = server.arg("since").toInt();
  DashState d; memcpy(&d, (const void *)&dashState, sizeof d);
  xSemaphoreTake(busMutex, portMAX_DELAY);
  float elapsed = (millis() - busStartMs) / 1000.0f;
  int n = snprintf(buf, sizeof buf,
    "{\"speed\":%.1f,\"rpm\":%.0f,\"temp\":%.1f,\"mil\":%d,\"ndtc\":%d,\"abs\":%d,\"brake\":%d,\"lost\":%d,"
    "\"pedal\":%d,\"load\":%.2f,\"ok\":%lu,\"err\":%lu,\"arb\":%lu,\"nodes\":[",
    d.speed, d.rpm, d.temp, d.mil, d.nDtc, d.absOn, d.brake, d.lost, (int)webPedal,
    elapsed > 0 ? bus.bits_total / (elapsed * CAN_BITRATE) * 100 : 0.0f,
    (unsigned long)bus.frames_ok, (unsigned long)bus.frames_err, (unsigned long)bus.arbitrations);
  for (int i = 0; i < bus.n_nodes; i++)
    n += snprintf(buf + n, sizeof buf - n, "%s{\"name\":\"%s\",\"tec\":%u,\"rec\":%u,\"state\":\"%s\"}", i ? "," : "",
                  bus.nodes[i].name, bus.nodes[i].tec, bus.nodes[i].rec, can_state_str(bus.nodes[i].state));
  n += snprintf(buf + n, sizeof buf - n, "],\"lastArb\":\"");
  if (bus.arb_winner >= 0)
    n += snprintf(buf + n, sizeof buf - n, "Last arbitration: 0x%03X (%s) won against 0x%03X (%s), lost at bit %d",
                  bus.arb_win_id, bus.nodes[bus.arb_winner].name, bus.arb_lose_id, bus.nodes[bus.arb_loser].name, bus.arb_lost_bit);
  n += snprintf(buf + n, sizeof buf - n, "\",\"frames\":[");
  uint32_t first = webSeq > WEB_FRAMES ? webSeq - WEB_FRAMES : 0;
  if (since + 1 > first) first = since + 1 > webSeq ? webSeq : since;
  bool comma = false;
  for (uint32_t q = first; q < webSeq && n < (int)sizeof buf - 200; q++) {
    const SniffItem &it = webFrames[q % WEB_FRAMES];
    char data[32] = ""; int w = 0;
    for (int i = 0; i < it.f.dlc && i < 8; i++) w += snprintf(data + w, sizeof data - w, "%s%02X", i ? " " : "", it.f.data[i]);
    n += snprintf(buf + n, sizeof buf - n, "%s{\"seq\":%lu,\"t\":%lu,\"id\":%u,\"from\":\"%s\",\"dlc\":%u,\"data\":\"%s\",\"err\":%d}",
                  comma ? "," : "", (unsigned long)(q + 1), (unsigned long)it.t, it.f.id, bus.nodes[it.sender].name, it.f.dlc, data, it.status < 0);
    comma = true;
  }
  xSemaphoreGive(busMutex);
  snprintf(buf + n, sizeof buf - n, "]}");
  server.send(200, "application/json", buf);
}

static void webObd() {
  uint8_t req[8];
  int n = obdShortcut(server.arg("q"), req);
  if (!n) { server.send(400, "application/json", "{\"log\":[],\"text\":\"unknown request\"}"); return; }
  static ObdResult r;
  obdRequest(req, n, &r);
  static char buf[1024];
  int w = snprintf(buf, sizeof buf, "{\"log\":[");
  for (int i = 0; i < r.n; i++) w += snprintf(buf + w, sizeof buf - w, "%s\"%s\"", i ? "," : "", r.lines[i]);
  snprintf(buf + w, sizeof buf - w, "],\"text\":\"%s\"}", r.text);
  server.send(200, "application/json", buf);
}

static void webCmd() {
  String c = server.arg("c"), v = server.arg("v");
  if (c == "pedal") webPedal = constrain(v.toInt(), -1, 100);
  else if (c == "brake") webBrakeUntil = millis() + 2000;
  else if (c == "fan") { fanFail = !fanFail; Serial.printf("# Cooling fan (web page): %s\n", fanFail ? "FAILED" : "repaired"); }
  else if (c == "error") {
    xSemaphoreTake(busMutex, portMAX_DELAY); bus.inject_error = v.toInt(); bus.inject_node = N_ENGINE; xSemaphoreGive(busMutex);
    Serial.printf("# %d noise bursts on the engine frames (web page)\n", (int)v.toInt());
  }
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BRAKE, INPUT_PULLUP); pinMode(PIN_FANFAIL, INPUT_PULLUP);
  pinMode(PIN_LED_MIL, OUTPUT); pinMode(PIN_LED_ABS, OUTPUT);
  Wire.begin(21, 22);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  can_bus_init(&bus);
  N_ENGINE = can_bus_add_node(&bus, "engine", 0x000, 0x000);
  N_ABS    = can_bus_add_node(&bus, "abs", 0x000, 0x000);
  N_DASH   = can_bus_add_node(&bus, "dashboard", 0x000, 0x000);
  N_DIAG   = can_bus_add_node(&bus, "diag", OBD_ECU_RESP_ID, 0x7FF);   // filter: 0x7E8 only
  busMutex = xSemaphoreCreateMutex();
  sniffQ = xQueueCreate(64, sizeof(SniffItem));
  diagMutex = xSemaphoreCreateMutex();
  busStartMs = millis();

  Serial.println("\n=== Automotive CAN network (500 kbit/s): engine 0x0C0, ABS 0x1A0, instrument cluster, diagnostics ===");
  xTaskCreatePinnedToCore(taskBus,     "bus",     4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskEngine,  "engine",  4096, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(taskAbs,     "abs",     3072, nullptr, 4, nullptr, 1);
  xTaskCreatePinnedToCore(taskDash,    "dash",    4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskDiag,    "diag",    6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskButtons, "buttons", 2048, nullptr, 1, nullptr, 0);

  WiFi.begin("Wokwi-GUEST", "", 6);
  for (int k = 0; k < 40 && WiFi.status() != WL_CONNECTED; k++) delay(250);
  server.on("/", []() { server.send(200, "text/html; charset=utf-8", WEB_PAGE); });
  server.on("/api/state", webState);
  server.on("/api/obd", webObd);
  server.on("/api/cmd", webCmd);
  server.begin();
  Serial.printf("# Web dashboard: http://localhost:8181 (Wi-Fi %s)\n", WiFi.status() == WL_CONNECTED ? "OK" : "not connected");
}

void loop() {
  server.handleClient();
  delay(2);
}
