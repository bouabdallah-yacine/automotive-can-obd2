# 🚗 Automotive CAN network + OBD-II diagnostics (ESP32 + FreeRTOS)

[![Tests](https://github.com/bouabdellah-yacine/automotive-can-obd2/actions/workflows/ci.yml/badge.svg)](https://github.com/bouabdellah-yacine/automotive-can-obd2/actions/workflows/ci.yml)

A modern car contains dozens of ECUs talking to each other over a **CAN bus**. This project
simulates such a network with 4 nodes (engine, ABS, instrument cluster, diagnostic tool) exchanging
frames **bit by bit**: CRC-15, bit stuffing, arbitration, error frames and bus-off of a faulty ECU.
The diagnostic tool reads live data and **OBD-II trouble codes**, just like a mechanic's scan tool.

> ✅ Simulated on **Wokwi** (VS Code). The CAN / OBD-II core is portable C, tested on a PC.

## The network

| Node | Transmits | Role |
|---|---|---|
| Engine ECU | `0x0C0` every 50 ms | RPM, coolant temperature, throttle, check-engine light; answers diagnostics on `0x7E8` |
| ABS ECU | `0x1A0` every 50 ms | vehicle speed, brake, ABS active |
| Instrument cluster | — | OLED display, detects loss of communication with the engine |
| Diagnostic tool | `0x7DF`, `0x7E0` | OBD-II requests (acceptance filter: receives `0x7E8` only) |

## What the project demonstrates

- **Bit-accurate CAN 2.0A frames**: SOF, 11-bit ID, DLC, data, **CRC-15** (polynomial 0x4599),
  **bit stuffing**, ACK, EOF. The `bits` command prints the last frame exactly as it travels on the wire.
- **Non-destructive arbitration**: dominant 0 beats recessive 1, the lowest ID wins.
- **Error handling**: corruption detected, error frame, automatic retransmission; **TEC / REC**
  counters, *active → passive → bus-off* states: a faulty ECU disconnects itself.
- **OBD-II (SAE J1979)**: mode 01 (RPM, speed, temperature, throttle), modes 03/04 (read / clear
  trouble codes **P0217**, **P0480**), mode 09 (VIN).
- **ISO-TP (ISO 15765-2)**: the VIN (20 bytes) is split into a *First Frame* + flow control +
  *Consecutive Frames*.
- **FreeRTOS**: one ECU per task, bus protected by a mutex.
- **Web dashboard served by the ESP32**: speedometer and tachometer, warning lights, **live bus
  sniffer** (like a CAN analyser), error counters for each ECU, OBD-II diagnostic buttons.

## Test results (`test/test_can.c`, 13 tests)

- Single-bit error injected at **each of the 106 positions** of a frame: **106 / 106 detected**.
- Double-bit errors: **392 / 392 detected**.
- Arbitration, filters, retransmission, bus-off transition (TEC ≥ 256), trouble codes, OBD-II, ISO-TP.

## Running the demo (Wokwi in VS Code)

1. Open this folder in VS Code → PlatformIO **Build** → **F1 › Wokwi: Start Simulator**.
2. Open **http://localhost:8181**: the dashboard (gauges, warning lights, live frames, diagnostics).
   Turn the potentiometer (**accelerator**) or the pedal on the web page: RPM and speed go up.
3. In the serial monitor, type:
   - `rpm`, `temp`, `speed`, `vin`: OBD-II requests, with the raw frames;
   - `sniff`: monitors all bus traffic, `bits`: the last frame bit by bit;
   - `stats`: bus load, errors, TEC/REC counters, last arbitration.
4. Press the **yellow “Fan failure”** button, then accelerate: the temperature exceeds 110 °C and the
   **CHECK ENGINE** light turns on. Type `dtc`: **P0480** and **P0217**. Then `clear` to erase them.
5. Type `error 40`: noise on the engine's frames, its TEC climbs until **BUS-OFF**, and the instrument cluster
   shows **ENGINE COMM LOST U0100**. The engine rejoins the network 3 s later.
6. Press the **red “Brake”** button above 30 km/h: the **ABS** light flashes.

> If port forwarding does not work in your Wokwi version, everything else still works:
> OLED screen, warning lights and serial monitor commands.

## Tests

```bash
gcc -O2 -Wall -Wextra -Isrc -o t test/test_can.c src/can.c src/can_bus.c src/obd.c && ./t
```

## License

© 2026 Yacine — all rights reserved. Code published for viewing only (see [`LICENSE`](LICENSE)).
