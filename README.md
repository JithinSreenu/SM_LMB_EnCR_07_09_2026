# Arduino-Uno-IDEAL — Uno port of STM32B-U585IIOTA02_IDEAL (ADCONY_ARD)

Direct Arduino Uno port of `STM32B-U585IIOTA02_IDEAL - CopyADCONY_ARD\STM32B-U585IIOTA02_IDEAL`
(`main.c`, `app_freertos.c`, `smart_protocol.c`, `adc.c`, `adc_manager.c`,
`state_machine.c/.h`, `thresholds.c/.h`, `coefficients.c/.h`, `slave_comm.c/.h`).

Sketch: `Arduino-Uno-IDEAL.ino` (single file, no extra libraries besides built-in
`SoftwareSerial` + `EEPROM`).

An older SIM-only build already exists at `..\ARDUINO-UNO-TELEMETRY-SIM`
(valve simulated, no slave UART, no FORCE_KNEE / mmode). This build adds:
real slave UART on D10/D11 with median-of-3 + freshness, full
FORCE/MOMENT/FORCE_KNEE state machine, DIFF/ABSDIFF/FORCE_ARM moment modes,
all `set k*` CLI commands, and `thr` alias.

## 1. Wiring

| Uno pin | Connect to | Notes |
|---|---|---|
| A0 | load cell 1 amp out (0–3.3 V) | force |
| A1 | load cell 2 amp out (0–3.3 V) | moment |
| A2 | knee sensor (0–3.3 V) | pot wiper / Hall |
| A3 | valve pot wiper (0–VREF) | fallback when slave stale; 0–100 % |
| A4 | battery divider out (≤3.3 V!) | 3.3k/8.2k divider for 4.2 V pack |
| A5 | spare | — |
| D12 | red LED + 220 Ω → GND | active HIGH |
| D13 | green LED (on-board) | active HIGH |
| D10 | slave TX | 115200 8N1, keep wires <30 cm |
| D11 | slave RX | 115200 8N1 |
| GND | slave GND + sensor GND | common ground |

Battery divider (same as STM32): `4.2V ─ 3.3k ─ [A4] ─ 8.2k ─ GND`.

## 2. Flash

1. Open `Arduino-Uno-IDEAL.ino` in Arduino IDE.
2. Board: Arduino Uno, correct COM port. No extra board packages needed.
3. Upload → open Serial Monitor @ **115200**.
4. Banner + `CMD> ` appears; binary telemetry streams @ 100 Hz immediately
   in `MODE_REAL`. With floating inputs type `mode demo` for clean gait.

To disable the slave port (pure A3 fallback): set `#define USE_SLAVE_UART 0`
at the top and re-upload.

## 3. Protocol (identical to STM32)

Telemetry TX (25 B, 100 Hz): `AA 55 seq | force i16×100 LE | moment i16×100 LE |
knee i16×100 LE | valve u16 | batt u16×100 LE | state u8 | fault u16 LE |
7×00 | CRC16-MODBUS(0..22) LE`.

Slave RX (7 B): `A5 01 02 lo hi crc_lo crc_hi` (angle×100, CRC over 0..4,
bounds 0–10000, median-of-3, 25 ms freshness). Test frame 45%:
`A5 01 02 94 11 66 E9`.

Slave TX (6 B, on state change): `A5 02 01 state crc_lo crc_hi`, state 0=SWING 1=STANCE.

Threshold RX (45 B): `A5 31 28 + 10×f32 LE + CRC`. Coefficient RX (53 B):
`A5 32 30 + 12×f32 LE + CRC`.

## 4. CLI

Same as STM32 `main.c`, minus BLE/crypto (stubbed as unsupported):
`red/green/all on|off`, `status`, `show`, `data`, `crc`, `dmatest`, `smart`,
`set force|moment|knee|valve|batt|state|fault x`, `adc`, `adc debug`,
`set cycles|lmode|arm|stance|stmax|swmin|swmax|psmin|psmax|lomin|lomax|msmin|msmax`,
`set sinput|mmode`, `set kswmin|kswmax|kpsmin|kpsmax|klo_min|klo_max|kmsmin|kmsmax|ktermmin|ktermmax|kpremin|kpremax`,
`sm`, `slave`, `halt on|off|halt`, `demo on|off`, `mode demo|real`,
`get thresholds` (`thr`), `get coefficients`, `help`.

## 5. Uno-specific notes

- ADC 10-bit: default `adc_max=1023`, `adc_vref=5.0` (AVcc). STM32 uses 4095/3.3.
  Set via Coefficient panel if you use external AREF.
- REAL force = SM window volts × `force_scale` (default 300 → N);
  moment = SM volts×arm × `force_scale`. Set scales to 1.0 for raw volts.
- Window max 100 (STM32: 255) — SRAM limit. Default 10.
- Coefficients persist in EEPROM 0..55 (magic `COEF` + CRC + v1).
- `SoftwareSerial` RX @115200 is marginal; shorten wires or drop both ends
  to 57600/9600 (`SLAVE_BAUD`) if you see `slave`-stale / valve stuck at A3.
