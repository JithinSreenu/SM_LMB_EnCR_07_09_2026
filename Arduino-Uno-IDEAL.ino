/*
 * ============================================================================
 *  Arduino Uno port of STM32B-U585IIOTA02_IDEAL (ADCONY_ARD build)
 * ============================================================================
 *  Reference firmware:
 *    D:\ONLY test\STM32B-U585IIOTA02_IDEAL - CopyADCONY_ARD\STM32B-U585IIOTA02_IDEAL
 *    (Core/Src/main.c, app_freertos.c, smart_protocol.c, adc.c, adc_manager.c,
 *     state_machine.c/.h, thresholds.c/.h, coefficients.c/.h, slave_comm.c/.h)
 *
 *  Location: D:\ONLY test\Arduino-Uno-IDEAL\Arduino-Uno-IDEAL.ino
 *  Board: Arduino Uno (ATmega328P, 16 MHz, 2 KB SRAM, 32 KB flash)
 *  IDE: Arduino IDE 1.8+/2.x — open this file, select "Arduino Uno", Upload.
 *  Console: Serial (USB) 115200 8N1 — same as STM32 USART1.
 *
 *  What is ported (behaviour-compatible, byte-for-byte protocol):
 *    - SmartProtocol 25-byte telemetry frame @ 100 Hz, auto-started at boot,
 *      MODE_REAL default (live ADC). CRC-16/MODBUS over bytes 0..22.
 *      [AA 55][seq][force i16 x100 LE][moment i16 x100 LE][knee i16 x100 LE]
 *      [valve u16][batt u16 x100 LE][state u8][fault u16 LE][7x zero][CRC LE]
 *    - ADC: A0 force cell1, A1 moment cell2, A2 knee, A3 valve fallback,
 *      A4 battery, A5 extra. voltage = raw * vref / adc_max.
 *    - Gait state machine: N-cycle window, SUM/AVG/MAX/MIN combine,
 *      moment modes FORCE_ARM/DIFF/ABSDIFF, inputs FORCE/MOMENT/FORCE_KNEE,
 *      per-phase voltage bands + knee-degree bands + mid-seen sequence flag.
 *    - Slave link: 7-byte RX (A5 01 02 lo hi crc) with carry parser,
 *      CRC + bounds check, median-of-3, 25 ms freshness gate;
 *      6-byte TX state command (A5 02 01 state crc) on every transition.
 *    - Thresholds packet RX (A5 31 len=40 + 10x f32 LE + CRC) -> fault bits.
 *    - Coefficients packet RX (A5 32 len=48 + 12x f32 LE + CRC) -> scales/
 *      offsets/vref/max, persisted to EEPROM (magic "COEF" + CRC + version).
 *    - Full text CLI (see help): leds, show/data/smart/crc/dmatest,
 *      set force/moment/knee/valve/batt/state/fault, adc, adc debug,
 *      set cycles/lmode/arm/stance/stmax/swmin/swmax/psmin/psmax/
 *      lomin/lomax/msmin/msmax/sinput/mmode/kswmin/kswmax/kpsmin/kpsmax/
 *      klo_min/klo_max/kmsmin/kmsmax/ktermmin/ktermmax/kpremin/kpremax,
 *      sm, slave, halt on/off/halt, demo on/off, mode demo/real,
 *      get thresholds, get coefficients, help.
 *    - DEMO mode: 6-phase clinical gait, same formulas as DemoStreamCallback.
 *    - HALT: freezes telemetry + slave TX (coeff packet auto-halts).
 *
 *  Wiring:
 *    A0 -> load cell 1 amp out (0-3.3V)      A1 -> load cell 2 amp out
 *    A2 -> knee sensor (pot/Hall, 0-3.3V)    A3 -> valve pot (0-VREF = 0-100%)
 *    A4 -> battery divider out (<=3.3V!)     A5 -> spare
 *    D12 -> RED led (external + 220R to GND, mirrors STM32 RED, active HIGH)
 *    D13 -> GREEN led (on-board, mirrors STM32 GREEN, active HIGH)
 *    D10 -> SLAVE RX (slave TX -> D10 via divider/level shift if slave is 3.3V/5V)
 *    D11 -> SLAVE TX (D11 -> slave RX)
 *    GND -> common ground with slave + sensors
 *    Slave link baud 115200 8N1 (same as STM32 USART2/3).
 *
 *  Uno vs STM32 differences (must read):
 *    - Uno ADC is 10-bit (0..1023), STM32 is 12-bit (0..4095).
 *      Default adc_max = 1023 here. PC Coefficient panel must use 1023.
 *    - Uno ADC reference defaults to AVcc (5V). Default adc_vref = 5.0 here
 *      so pin volts read correctly. STM32 VREF = 3.3V. Sensors must still
 *      never exceed 3.3V at the pin (safe under 5V reference, just scaled).
 *      If you feed AREF with 3.3V + analogReference(EXTERNAL), set vref=3.3.
 *    - Single HW UART: console uses Serial (USB). Slave uses SoftwareSerial
 *      on D10/D11. SoftwareSerial RX at 115200 is marginal — keep slave wires
 *      short (<30 cm). If you get framing errors, lower BOTH sides to 57600
 *      or 9600 (change SLAVE_BAUD below AND re-flash the slave).
 *      Set USE_SLAVE_UART 0 to disable the slave port (pure A3 fallback,
 *      exactly like the ARDUINO-UNO-TELEMETRY-SIM build).
 *    - No RTOS/DMA/BLE/crypto: telemetry is a millis() 10 ms poll in loop().
 *      `dmatest` prints DMA TX OK, `ble ...` / `crypto ...` print
 *      "not supported on Uno". Encrypted frames are never emitted.
 *    - State window capped at 100 (STM32 allows 255) to fit 2 KB SRAM.
 *      100 x float = 400 bytes. Do not raise this.
 *    - Coefficients persist in AVR EEPROM bytes 0..55 (same 56-byte layout
 *      as STM32 flash record: magic u32 + crc u16 + ver u8 + rsv u8 +
 *      12x f32 LE). Boot prints "Coefficients restored from flash" if valid.
 *
 *  Bench test (no sensors needed):
 *    1. Upload, open Serial Monitor @ 115200. Banner + CMD> appears,
 *       telemetry streams at 100 Hz (binary — looks like garbage, normal).
 *    2. Type `mode demo` + Enter -> clean simulated gait for the PC app.
 *    3. Type `show` -> readable values. Type `sm` -> state machine status.
 *    4. Slave test: wire slave TX->D10, or send via 2nd USB-TTL:
 *       TX->D10, GND->GND, send `A5 01 02 94 11 66 E9` (=45%) at 115200.
 *    5. PC app (Prosthetic Telemetry Studio): select Uno COM port, 115200,
 *       Connect. Threshold/Coefficient panels work as on STM32.
 * ============================================================================
 */

#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

/* ============================ build switches ============================ */
#define USE_SLAVE_UART   1        /* 1 = real slave on D10/D11, 0 = A3 fallback only */
#define SLAVE_RX_PIN     10
#define SLAVE_TX_PIN     11
#define SLAVE_BAUD       115200   /* must match slave firmware */
#define CONSOLE_BAUD     115200

/* ============================ protocol constants ========================= */
#define SMART_PACKET_LEN        25U
#define SMART_CRC_INPUT_LEN     23U

#define THR_PACKET_SYNC         0xA5
#define THR_PACKET_TYPE         0x31
#define THR_PACKET_MIN_LEN      45U
#define THR_NUM_FIELDS          10U
#define THR_PAYLOAD_LEN         40U
#define THR_IDX_FORCE_MIN   0
#define THR_IDX_FORCE_MAX   1
#define THR_IDX_MOMENT_MIN  2
#define THR_IDX_MOMENT_MAX  3
#define THR_IDX_KNEE_MIN    4
#define THR_IDX_KNEE_MAX    5
#define THR_IDX_VALVE_MIN   6
#define THR_IDX_VALVE_MAX   7
#define THR_IDX_BATT_MIN    8
#define THR_IDX_BATT_MAX    9

#define COEFF_PACKET_TYPE       0x32
#define COEFF_PACKET_MIN_LEN    53U
#define COEFF_NUM_FIELDS        12U
#define COEFF_PAYLOAD_LEN       48U
#define COEFF_IDX_FORCE_SCALE   0
#define COEFF_IDX_MOMENT_SCALE  1
#define COEFF_IDX_KNEE_SCALE    2
#define COEFF_IDX_BATT_SCALE    3
#define COEFF_IDX_ADC_VREF      4
#define COEFF_IDX_ADC_MAX       5
#define COEFF_IDX_FORCE_OFFSET  6
#define COEFF_IDX_MOMENT_OFFSET 7
#define COEFF_IDX_KNEE_OFFSET   8
#define COEFF_IDX_BATT_OFFSET   9
#define COEFF_IDX_ADC_SMP       10
#define COEFF_IDX_ADC_CLK       11

/* Fault bits (thresholds.h) */
#define STATUS_FORCE_OVER_MAX       0x0001
#define STATUS_FORCE_UNDER_MIN      0x0002
#define STATUS_FORCE_SENSOR_FAIL    0x0004
#define STATUS_MOMENT_OVER_MAX      0x0008
#define STATUS_MOMENT_UNDER_MIN     0x0010
#define STATUS_KNEE_OVER_MAX        0x0020
#define STATUS_KNEE_UNDER_MIN       0x0040
#define STATUS_KNEE_SENSOR_FAIL     0x0080
#define STATUS_VALVE_OVER_MAX       0x0100
#define STATUS_VALVE_UNDER_MIN      0x0200
#define STATUS_VALVE_SENSOR_FAIL    0x0400
#define STATUS_BATT_OVER_MAX        0x0800
#define STATUS_BATT_UNDER_MIN       0x1000
#define STATUS_BATT_SENSOR_FAIL     0x2000
#define STATUS_ADC_STUCK_ZERO       0x4000
#define STATUS_ADC_STUCK_VREF       0x8000

/* Gait states (state_machine.h — match PC app STATE_LABELS) */
#define SM_STATE_SWING              0
#define SM_STATE_STANCE             1
#define SM_STATE_LOADING            2
#define SM_STATE_MID_STANCE         3
#define SM_STATE_TERMINAL_STANCE    4
#define SM_STATE_PRE_SWING          5

#define SM_COMBINE_SUM      0
#define SM_COMBINE_AVG      1
#define SM_COMBINE_MAX      2
#define SM_COMBINE_MIN      3

#define SM_INPUT_FORCE      0
#define SM_INPUT_MOMENT     1
#define SM_INPUT_FORCE_KNEE 2

#define SM_MOMENT_FORCE_ARM 0
#define SM_MOMENT_DIFF      1
#define SM_MOMENT_ABSDIFF   2

#define SM_DEFAULT_WINDOW       10
#define SM_DEFAULT_COMBINE_MODE SM_COMBINE_SUM
#define SM_DEFAULT_MOMENT_ARM   0.2f
#define SM_DEFAULT_INPUT        SM_INPUT_FORCE
#define SM_DEFAULT_SWING_MIN    0.00f
#define SM_DEFAULT_SWING_MAX    0.20f
#define SM_DEFAULT_PRESWING_MIN 0.20f
#define SM_DEFAULT_PRESWING_MAX 0.45f
#define SM_DEFAULT_LOAD_MIN     0.45f
#define SM_DEFAULT_LOAD_MAX     0.80f
#define SM_DEFAULT_MID_MIN      0.80f
#define SM_DEFAULT_MID_MAX      3.40f
#define SM_DEFAULT_STANCE_MIN   0.50f
#define SM_DEFAULT_STANCE_MAX   3.40f
#define SM_DEFAULT_KNEE_SWING_MIN 30.0f
#define SM_DEFAULT_KNEE_SWING_MAX 90.0f
#define SM_DEFAULT_KNEE_PSW_MIN   10.0f
#define SM_DEFAULT_KNEE_PSW_MAX   40.0f
#define SM_DEFAULT_KNEE_LOAD_MIN  5.0f
#define SM_DEFAULT_KNEE_LOAD_MAX  25.0f
#define SM_DEFAULT_KNEE_MID_MIN   0.0f
#define SM_DEFAULT_KNEE_MID_MAX   15.0f
#define SM_DEFAULT_KNEE_TERM_MIN  5.0f
#define SM_DEFAULT_KNEE_TERM_MAX  25.0f
#define SM_DEFAULT_KNEE_PRE_MIN   15.0f
#define SM_DEFAULT_KNEE_PRE_MAX   50.0f

#define SM_MAX_WINDOW           100   /* STM32 allows 255; Uno SRAM cap */

/* Slave link (slave_comm.h) */
#define SLAVE_SYNC_BYTE      0xA5
#define SLAVE_TYPE_SPOOL     0x01
#define SLAVE_PAYLOAD_LEN    2
#define SLAVE_FRAME_SIZE     7
#define SLAVE_FRESH_MS       25
#define SLAVE_ANGLE_MIN      0
#define SLAVE_ANGLE_MAX      10000

/* Coefficient EEPROM record (same layout as STM32 flash record) */
#define COEFF_FLASH_MAGIC     0x434F4546UL
#define COEFF_FLASH_VERSION   1U
#define COEFF_EEPROM_ADDR     0U
#define COEFF_RECORD_LEN      56U

/* Pins */
#define PIN_LED_GREEN   13
#define PIN_LED_RED     12
#define CH_FORCE        A0
#define CH_MOMENT       A1
#define CH_KNEE         A2
#define CH_VALVE        A3
#define CH_BATTERY      A4
#define CH_EXTRA        A5
#define ADC_CH_FORCE_IDX   0
#define ADC_CH_MOMENT_IDX  1
#define ADC_CH_KNEE_IDX    2
#define ADC_CH_VALVE_IDX   3
#define ADC_CH_BATTERY_IDX 4
#define ADC_CH_EXTRA_IDX   5

#define MODE_DEMO       0
#define MODE_REAL       1
#define MODE_COMMAND    0
#define MODE_WAIT_FORCE 1
#define MODE_WAIT_MOMENT 2
#define MODE_WAIT_KNEE  3
#define MODE_WAIT_VALVE 4
#define MODE_WAIT_BATT  5
#define MODE_WAIT_STATE 6
#define MODE_WAIT_FAULT 7

/* ============================ data structures ============================ */
typedef struct {
  float force_N, moment_Nm, kneeAngle_deg, valvePosition_percent, batteryVoltage_V;
  uint8_t systemState;
  uint16_t faultCode;
} TelemetryValues_t;

typedef struct {
  float force_min, force_max, moment_min, moment_max, knee_min, knee_max;
  float valve_min, valve_max, batt_min, batt_max;
} Thresholds_t;

typedef struct {
  float force_scale, moment_scale, knee_scale, batt_scale;
  float adc_vref, adc_max;
  float force_offset, moment_offset, knee_offset, batt_offset;
  float adc_smp, adc_clk;
} Coefficients_t;

/* ============================ globals ==================================== */
static TelemetryValues_t g_values;
static uint8_t  g_sequence = 0;
static uint8_t  g_mode = MODE_REAL;
static uint8_t  g_streaming = 1;
static volatile uint8_t g_systemHalt = 0;
static uint8_t  g_thresholdsActive = 0;
static Thresholds_t g_thresholds;
static uint8_t  g_coefficientsActive = 0;
static Coefficients_t g_coefficients;
static uint8_t  g_termMode = MODE_COMMAND;
static uint8_t  g_txFrame[SMART_PACKET_LEN];
static uint8_t  g_lastSentState = 0xFF;

#if USE_SLAVE_UART
static SoftwareSerial slaveSerial(SLAVE_RX_PIN, SLAVE_TX_PIN);
#endif
static volatile uint16_t g_slaveSpoolAngle_x100 = 0;
static volatile uint8_t  g_slaveDataValid = 0;
static volatile uint32_t g_slaveLastValidTick = 0;
/* median-of-3 ring */
static uint16_t s_angleHist[3];
static uint8_t  s_histCount = 0, s_histIdx = 0;
/* carry parser (handles frames split across reads) */
static uint8_t s_carry[SLAVE_FRAME_SIZE];
static uint8_t s_carryLen = 0;

/* forward decls */
static uint8_t coeffSaveToFlash(void);
static void printUint(uint32_t v);
static void printFloat3(float v);

/* ============================ CRC + LE helpers =========================== */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}
static void put_u16_le(uint8_t *b, uint16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }
static void put_u32_le(uint8_t *b, uint32_t v) {
  b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}
static void put_i16_le(uint8_t *b, int16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }
static void put_f32_le(uint8_t *b, float v) {
  union { uint32_t u; float f; } c; c.f = v; put_u32_le(b, c.u);
}
static float read_f32_le(const uint8_t *b) {
  union { uint32_t u; float f; } c;
  c.u = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  return c.f;
}

/* ============================ smart protocol ============================= */
static void buildTelemetryFrame(uint8_t *buf, uint16_t *outLen) {
  memset(buf, 0, SMART_PACKET_LEN);
  int16_t  fx = (int16_t)(g_values.force_N * 100.0f);
  int16_t  mx = (int16_t)(g_values.moment_Nm * 100.0f);
  int16_t  kx = (int16_t)(g_values.kneeAngle_deg * 100.0f);
  uint16_t vx = (uint16_t)(g_values.valvePosition_percent);
  uint16_t bx = (uint16_t)(g_values.batteryVoltage_V * 100.0f);
  buf[0] = 0xAA; buf[1] = 0x55; buf[2] = g_sequence;
  put_i16_le(&buf[3], fx); put_i16_le(&buf[5], mx); put_i16_le(&buf[7], kx);
  put_u16_le(&buf[9], vx); put_u16_le(&buf[11], bx);
  buf[13] = g_values.systemState;
  put_u16_le(&buf[14], g_values.faultCode);
  uint16_t crc = crc16_modbus(buf, SMART_CRC_INPUT_LEN);
  buf[23] = crc & 0xFF; buf[24] = (crc >> 8) & 0xFF;
  *outLen = SMART_PACKET_LEN;
}

/* ============================ thresholds ================================= */
static uint8_t processThresholdPacket(const uint8_t *buf, uint16_t len) {
  if (len < THR_PACKET_MIN_LEN) return 0;
  if (buf[0] != THR_PACKET_SYNC || buf[1] != THR_PACKET_TYPE) return 0;
  if (buf[2] != THR_PAYLOAD_LEN) return 0;
  uint16_t rx = buf[3 + buf[2]] | ((uint16_t)buf[4 + buf[2]] << 8);
  if (crc16_modbus(buf, 3 + buf[2]) != rx) return 0;
  const uint8_t *p = buf + 3;
  g_thresholds.force_min = read_f32_le(p + 0 * 4); g_thresholds.force_max = read_f32_le(p + 1 * 4);
  g_thresholds.moment_min = read_f32_le(p + 2 * 4); g_thresholds.moment_max = read_f32_le(p + 3 * 4);
  g_thresholds.knee_min = read_f32_le(p + 4 * 4); g_thresholds.knee_max = read_f32_le(p + 5 * 4);
  g_thresholds.valve_min = read_f32_le(p + 6 * 4); g_thresholds.valve_max = read_f32_le(p + 7 * 4);
  g_thresholds.batt_min = read_f32_le(p + 8 * 4); g_thresholds.batt_max = read_f32_le(p + 9 * 4);
  g_thresholdsActive = 1;
  return 1;
}
static uint16_t checkTelemetry(const TelemetryValues_t *v) {
  if (!g_thresholdsActive) return 0;
  uint16_t s = 0;
  if (v->force_N < g_thresholds.force_min) s |= STATUS_FORCE_UNDER_MIN;
  if (v->force_N > g_thresholds.force_max) s |= STATUS_FORCE_OVER_MAX;
  if (v->moment_Nm < g_thresholds.moment_min) s |= STATUS_MOMENT_UNDER_MIN;
  if (v->moment_Nm > g_thresholds.moment_max) s |= STATUS_MOMENT_OVER_MAX;
  if (v->kneeAngle_deg < g_thresholds.knee_min) s |= STATUS_KNEE_UNDER_MIN;
  if (v->kneeAngle_deg > g_thresholds.knee_max) s |= STATUS_KNEE_OVER_MAX;
  if (v->valvePosition_percent < g_thresholds.valve_min) s |= STATUS_VALVE_UNDER_MIN;
  if (v->valvePosition_percent > g_thresholds.valve_max) s |= STATUS_VALVE_OVER_MAX;
  if (v->batteryVoltage_V < g_thresholds.batt_min) s |= STATUS_BATT_UNDER_MIN;
  if (v->batteryVoltage_V > g_thresholds.batt_max) s |= STATUS_BATT_OVER_MAX;
  return s;
}

/* ============================ coefficients =============================== */
static uint8_t processCoeffPacket(const uint8_t *buf, uint16_t len) {
  if (len < COEFF_PACKET_MIN_LEN) return 0;
  if (buf[0] != THR_PACKET_SYNC || buf[1] != COEFF_PACKET_TYPE) return 0;
  if (buf[2] != COEFF_PAYLOAD_LEN) return 0;
  uint16_t rx = buf[3 + buf[2]] | ((uint16_t)buf[4 + buf[2]] << 8);
  if (crc16_modbus(buf, 3 + buf[2]) != rx) return 0;
  const uint8_t *p = buf + 3;
  g_coefficients.force_scale = read_f32_le(p + 0 * 4);
  g_coefficients.moment_scale = read_f32_le(p + 1 * 4);
  g_coefficients.knee_scale = read_f32_le(p + 2 * 4);
  g_coefficients.batt_scale = read_f32_le(p + 3 * 4);
  g_coefficients.adc_vref = read_f32_le(p + 4 * 4);
  g_coefficients.adc_max = read_f32_le(p + 5 * 4);
  g_coefficients.force_offset = read_f32_le(p + 6 * 4);
  g_coefficients.moment_offset = read_f32_le(p + 7 * 4);
  g_coefficients.knee_offset = read_f32_le(p + 8 * 4);
  g_coefficients.batt_offset = read_f32_le(p + 9 * 4);
  g_coefficients.adc_smp = read_f32_le(p + 10 * 4);
  g_coefficients.adc_clk = read_f32_le(p + 11 * 4);
  if (g_coefficients.adc_vref <= 0) g_coefficients.adc_vref = 5.0f;
  if (g_coefficients.adc_max <= 0) g_coefficients.adc_max = 1023.0f;
  g_coefficientsActive = 1;
  coeffSaveToFlash();
  return 1;
}
static uint8_t coeffSaveToFlash(void) {
  uint8_t rec[COEFF_RECORD_LEN], stored[COEFF_RECORD_LEN];
  float vals[COEFF_NUM_FIELDS];
  vals[0] = g_coefficients.force_scale; vals[1] = g_coefficients.moment_scale;
  vals[2] = g_coefficients.knee_scale; vals[3] = g_coefficients.batt_scale;
  vals[4] = g_coefficients.adc_vref; vals[5] = g_coefficients.adc_max;
  vals[6] = g_coefficients.force_offset; vals[7] = g_coefficients.moment_offset;
  vals[8] = g_coefficients.knee_offset; vals[9] = g_coefficients.batt_offset;
  vals[10] = g_coefficients.adc_smp; vals[11] = g_coefficients.adc_clk;
  uint16_t crc = crc16_modbus((uint8_t *)vals, sizeof(vals));
  put_u32_le(&rec[0], COEFF_FLASH_MAGIC); put_u16_le(&rec[4], crc);
  rec[6] = COEFF_FLASH_VERSION; rec[7] = 0;
  for (uint8_t i = 0; i < COEFF_NUM_FIELDS; i++) put_f32_le(&rec[8 + i * 4], vals[i]);
  for (uint8_t i = 0; i < COEFF_RECORD_LEN; i++) stored[i] = EEPROM.read(COEFF_EEPROM_ADDR + i);
  if (!memcmp(stored, rec, COEFF_RECORD_LEN)) return 1;
  for (uint8_t i = 0; i < COEFF_RECORD_LEN; i++) EEPROM.update(COEFF_EEPROM_ADDR + i, rec[i]);
  return 1;
}
static uint8_t coeffLoadFromFlash(void) {
  uint8_t rec[COEFF_RECORD_LEN];
  float vals[COEFF_NUM_FIELDS];
  for (uint8_t i = 0; i < COEFF_RECORD_LEN; i++) rec[i] = EEPROM.read(COEFF_EEPROM_ADDR + i);
  uint32_t magic = rec[0] | ((uint32_t)rec[1] << 8) | ((uint32_t)rec[2] << 16) | ((uint32_t)rec[3] << 24);
  if (magic != COEFF_FLASH_MAGIC || rec[6] != COEFF_FLASH_VERSION) return 0;
  for (uint8_t i = 0; i < COEFF_NUM_FIELDS; i++) vals[i] = read_f32_le(&rec[8 + i * 4]);
  if ((rec[4] | ((uint16_t)rec[5] << 8)) != crc16_modbus((uint8_t *)vals, sizeof(vals))) return 0;
  g_coefficients.force_scale = vals[0]; g_coefficients.moment_scale = vals[1];
  g_coefficients.knee_scale = vals[2]; g_coefficients.batt_scale = vals[3];
  g_coefficients.adc_vref = vals[4]; g_coefficients.adc_max = vals[5];
  g_coefficients.force_offset = vals[6]; g_coefficients.moment_offset = vals[7];
  g_coefficients.knee_offset = vals[8]; g_coefficients.batt_offset = vals[9];
  g_coefficients.adc_smp = vals[10]; g_coefficients.adc_clk = vals[11];
  g_coefficientsActive = 1;
  return 1;
}

/* ============================ ADC layer ================================== */
static uint16_t g_raw[6];
static void adcPoll(void) {
  g_raw[0] = analogRead(CH_FORCE); g_raw[1] = analogRead(CH_MOMENT);
  g_raw[2] = analogRead(CH_KNEE); g_raw[3] = analogRead(CH_VALVE);
  g_raw[4] = analogRead(CH_BATTERY); g_raw[5] = analogRead(CH_EXTRA);
}
static float adcVoltage(uint8_t ch) {
  if (ch > 5) return 0;
  return (float)g_raw[ch] * g_coefficients.adc_vref / g_coefficients.adc_max;
}
static float adcGetKneeAngle(void) {
  return (adcVoltage(ADC_CH_KNEE_IDX) + g_coefficients.knee_offset) * g_coefficients.knee_scale;
}
static float adcGetBattery(void) {
  return (adcVoltage(ADC_CH_BATTERY_IDX) + g_coefficients.batt_offset) * g_coefficients.batt_scale;
}
static float adcGetValvePercent(void) {
  if (g_slaveDataValid && (millis() - g_slaveLastValidTick) <= SLAVE_FRESH_MS)
    return (float)g_slaveSpoolAngle_x100 / 100.0f;
  if (g_coefficients.adc_vref <= 0) return 0;
  return adcVoltage(ADC_CH_VALVE_IDX) / g_coefficients.adc_vref * 100.0f;
}
static uint16_t adcCheckStuck(void) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 6; i++) sum += g_raw[i];
  uint16_t avg = sum / 6;
  if (avg < 2) return STATUS_ADC_STUCK_ZERO;
  if (avg > (uint16_t)(g_coefficients.adc_max - 2)) return STATUS_ADC_STUCK_VREF;
  return 0;
}

/* ============================ slave link ================================= */
static uint16_t slaveMedian(void) {
  if (s_histCount == 0) return 0;
  if (s_histCount == 1) return s_angleHist[(s_histIdx + 2) % 3];
  if (s_histCount == 2) return (s_angleHist[0] + s_angleHist[1]) / 2;
  uint16_t a = s_angleHist[0], b = s_angleHist[1], c = s_angleHist[2], t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}
static void slaveParseFrame(uint8_t *f) {
  if (f[0] != SLAVE_SYNC_BYTE || f[1] != SLAVE_TYPE_SPOOL || f[2] != SLAVE_PAYLOAD_LEN) return;
  uint16_t rx = f[5] | ((uint16_t)f[6] << 8);
  if (crc16_modbus(f, 5) != rx) return;
  uint16_t angle = f[3] | ((uint16_t)f[4] << 8);
  if (angle > SLAVE_ANGLE_MAX) return;
  s_angleHist[s_histIdx] = angle; s_histIdx = (s_histIdx + 1) % 3;
  if (s_histCount < 3) s_histCount++;
  g_slaveSpoolAngle_x100 = slaveMedian();
  g_slaveLastValidTick = millis();
  g_slaveDataValid = 1;
}
/* Feed raw bytes (from SoftwareSerial or test injector) through carry parser */
static void slaveRxChunk(uint8_t *data, uint16_t size) {
  uint16_t i = 0;
  while (i < size) {
    while (s_carryLen < SLAVE_FRAME_SIZE && i < size) s_carry[s_carryLen++] = data[i++];
    if (s_carryLen < SLAVE_FRAME_SIZE) break;
    if (s_carry[0] == SLAVE_SYNC_BYTE && s_carry[1] == SLAVE_TYPE_SPOOL && s_carry[2] == SLAVE_PAYLOAD_LEN) {
      slaveParseFrame(s_carry); s_carryLen = 0;
    } else {
      memmove(s_carry, &s_carry[1], SLAVE_FRAME_SIZE - 1);
      s_carryLen = SLAVE_FRAME_SIZE - 1;
    }
  }
}
static void slavePoll(void) {
#if USE_SLAVE_UART
  while (slaveSerial.available()) {
    uint8_t b = (uint8_t)slaveSerial.read();
    slaveRxChunk(&b, 1);
  }
#endif
}
static void slaveSendState(uint8_t state) {
  g_lastSentState = state;   /* CLI `slave` always reflects this */
#if USE_SLAVE_UART
  uint8_t f[6];
  f[0] = 0xA5; f[1] = 0x02; f[2] = 0x01; f[3] = state;
  uint16_t crc = crc16_modbus(f, 4);
  f[4] = crc & 0xFF; f[5] = (crc >> 8) & 0xFF;
  slaveSerial.write(f, sizeof(f));  /* real 6-byte command to slave */
#endif
}

/* ============================ state machine ============================== */
static uint8_t g_window = SM_DEFAULT_WINDOW, g_smMode = SM_DEFAULT_COMBINE_MODE;
static float   g_arm = SM_DEFAULT_MOMENT_ARM;
static uint8_t g_sinput = SM_DEFAULT_INPUT, g_mmMode = SM_MOMENT_DIFF;
static float g_swMin = SM_DEFAULT_SWING_MIN, g_swMax = SM_DEFAULT_SWING_MAX;
static float g_psMin = SM_DEFAULT_PRESWING_MIN, g_psMax = SM_DEFAULT_PRESWING_MAX;
static float g_loMin = SM_DEFAULT_LOAD_MIN, g_loMax = SM_DEFAULT_LOAD_MAX;
static float g_msMin = SM_DEFAULT_MID_MIN, g_msMax = SM_DEFAULT_MID_MAX;
static float g_stMin = SM_DEFAULT_STANCE_MIN, g_stMax = SM_DEFAULT_STANCE_MAX;
static float g_kSwMin = SM_DEFAULT_KNEE_SWING_MIN, g_kSwMax = SM_DEFAULT_KNEE_SWING_MAX;
static float g_kPsMin = SM_DEFAULT_KNEE_PSW_MIN, g_kPsMax = SM_DEFAULT_KNEE_PSW_MAX;
static float g_kLoMin = SM_DEFAULT_KNEE_LOAD_MIN, g_kLoMax = SM_DEFAULT_KNEE_LOAD_MAX;
static float g_kMsMin = SM_DEFAULT_KNEE_MID_MIN, g_kMsMax = SM_DEFAULT_KNEE_MID_MAX;
static float g_kTermMin = SM_DEFAULT_KNEE_TERM_MIN, g_kTermMax = SM_DEFAULT_KNEE_TERM_MAX;
static float g_kPreMin = SM_DEFAULT_KNEE_PRE_MIN, g_kPreMax = SM_DEFAULT_KNEE_PRE_MAX;
static uint8_t g_midSeen = 0;
static float g_buf[SM_MAX_WINDOW];
static float g_sum1 = 0, g_sum2 = 0;
static uint8_t g_count = 0;
static float g_smForce = 0, g_smMoment = 0;
static uint8_t g_smState = SM_STATE_SWING;

static float smCombine(float a, float b) {
  switch (g_smMode) {
    case SM_COMBINE_AVG: return (a + b) * 0.5f;
    case SM_COMBINE_MAX: return a > b ? a : b;
    case SM_COMBINE_MIN: return a < b ? a : b;
    default: return a + b;
  }
}
static float smMoment(float v1, float v2, float force) {
  switch (g_mmMode) {
    case SM_MOMENT_DIFF: return (v1 - v2) * g_arm;
    case SM_MOMENT_ABSDIFF: return fabsf(v1 - v2) * g_arm;
    default: return force * g_arm;
  }
}
static void smInit(void) {
  g_count = 0; g_sum1 = g_sum2 = 0; g_midSeen = 0;
  g_smForce = g_smMoment = 0; g_smState = SM_STATE_SWING;
}
static void smRun(void) {
  float c1 = adcVoltage(ADC_CH_FORCE_IDX), c2 = adcVoltage(ADC_CH_MOMENT_IDX);
  g_buf[g_count] = smCombine(c1, c2);
  g_sum1 += c1; g_sum2 += c2; g_count++;
  if (g_count < g_window) return;
  float sum = 0;
  for (uint8_t i = 0; i < g_window; i++) sum += g_buf[i];
  g_smForce = sum / g_window;
  float a1 = g_sum1 / g_window, a2 = g_sum2 / g_window;
  g_smMoment = smMoment(a1, a2, g_smForce);
  float knee = adcGetKneeAngle();
  g_sum1 = g_sum2 = 0;
  if (g_sinput == SM_INPUT_FORCE_KNEE) {
    bool inSw = (g_smForce >= g_swMin && g_smForce <= g_swMax) && (knee >= g_kSwMin && knee <= g_kSwMax);
    bool inPs = (g_smForce >= g_psMin && g_smForce <= g_psMax) && (knee >= g_kPsMin && knee <= g_kPsMax);
    bool inLo = (g_smForce >= g_loMin && g_smForce <= g_loMax) && (knee >= g_kLoMin && knee <= g_kLoMax);
    bool inMs = (g_smForce >= g_msMin && g_smForce <= g_msMax) && (knee >= g_kMsMin && knee <= g_kMsMax);
    bool inTerm = (g_smForce >= g_loMin && g_smForce <= g_loMax) && (knee >= g_kTermMin && knee <= g_kTermMax);
    bool inPre = (g_smForce >= g_psMin && g_smForce <= g_psMax) && (knee >= g_kPreMin && knee <= g_kPreMax);
    bool inSt = (g_smForce >= g_stMin);
    if (inSw) { g_smState = SM_STATE_SWING; g_midSeen = 0; }
    else if (inPs) g_smState = g_midSeen ? SM_STATE_PRE_SWING : SM_STATE_SWING;
    else if (inLo) g_smState = g_midSeen ? SM_STATE_TERMINAL_STANCE : SM_STATE_LOADING;
    else if (inMs) { g_smState = SM_STATE_MID_STANCE; g_midSeen = 1; }
    else if (inTerm) g_smState = SM_STATE_TERMINAL_STANCE;
    else if (inPre) g_smState = SM_STATE_PRE_SWING;
    else if (inSt) g_smState = SM_STATE_STANCE;
  } else {
    float in = (g_sinput == SM_INPUT_MOMENT) ? g_smMoment : g_smForce;
    if (in >= g_swMin && in <= g_swMax) { g_smState = SM_STATE_SWING; g_midSeen = 0; }
    else if (in >= g_psMin && in <= g_psMax) g_smState = g_midSeen ? SM_STATE_PRE_SWING : SM_STATE_SWING;
    else if (in >= g_loMin && in <= g_loMax) g_smState = g_midSeen ? SM_STATE_TERMINAL_STANCE : SM_STATE_LOADING;
    else if (in >= g_msMin && in <= g_msMax) { g_smState = SM_STATE_MID_STANCE; g_midSeen = 1; }
    else if (in >= g_stMin) g_smState = SM_STATE_STANCE;
  }
  g_count = 0;
}

/* ============================ telemetry tick ============================= */
static void telemetryTick(void) {
  if (g_systemHalt) return;
  slavePoll();
  if (g_mode == MODE_REAL) {
    adcPoll();
    smRun();
    /* NOTE: STM32 multiplies SM volts by compile-time FORCE_SCALE (1.0 in
       ADCONY_ARD). Here we apply the LIVE runtime force_scale so the PC
       Coefficient panel actually affects REAL force/moment. Defaults
       (300/60) give engineering units; set scales to 1.0 for raw volts. */
    g_values.force_N = (g_smForce + 0.0f) * g_coefficients.force_scale;
    /* moment: SM already includes arm; scale with moment_scale/force_scale
       ratio to keep units sane: use force_scale like STM32 (M = V*m * N/V) */
    g_values.moment_Nm = g_smMoment * g_coefficients.force_scale;
    g_values.kneeAngle_deg = adcGetKneeAngle();
    g_values.valvePosition_percent = adcGetValvePercent();
    g_values.batteryVoltage_V = adcGetBattery();
    g_values.systemState = g_smState;
    g_values.faultCode = adcCheckStuck();
    uint8_t slaveState = (g_smState > 0) ? 1 : 0;
    static uint8_t lastSent = 0xFF;
    if (slaveState != lastSent) { lastSent = slaveState; slaveSendState(slaveState); }
  } else {
    float t = millis() / 1000.0f;
    float phase = fmodf(t, 1.1f) / 1.1f;
    if (phase < 0.04f) g_values.systemState = SM_STATE_SWING;
    else if (phase < 0.12f) g_values.systemState = SM_STATE_LOADING;
    else if (phase < 0.35f) g_values.systemState = SM_STATE_MID_STANCE;
    else if (phase < 0.58f) g_values.systemState = SM_STATE_TERMINAL_STANCE;
    else if (phase < 0.70f) g_values.systemState = SM_STATE_PRE_SWING;
    else g_values.systemState = SM_STATE_SWING;
    if (phase >= 0.10f && phase < 0.58f) {
      float p = (phase - 0.10f) / 0.48f;
      g_values.force_N = 720.0f * sinf(p * 3.14159265f);
      g_values.moment_Nm = 32.0f * sinf(p * 3.14159265f);
    } else {
      g_values.force_N = 12.0f + 10.0f * sinf(phase * 6.0f * 3.14159265f);
      g_values.moment_Nm = -2.0f + 1.5f * sinf(phase * 6.0f * 3.14159265f);
    }
    g_values.kneeAngle_deg = (phase > 0.6f) ? (10.0f + 50.0f * ((phase - 0.6f) / 0.4f)) : 10.0f;
    g_values.valvePosition_percent = 30.0f + 40.0f * phase;
    g_values.batteryVoltage_V = 4.20f - t * 0.0002f;
    g_values.faultCode = 0;
  }
  g_values.faultCode |= checkTelemetry(&g_values);
  uint16_t len;
  buildTelemetryFrame(g_txFrame, &len);
  Serial.write(g_txFrame, len);
  g_sequence++;
}

/* ============================ print helpers ============================== */
static void printUint(uint32_t v) { Serial.print(v, DEC); }
static void printFloat2(float v) {
  if (v < 0) { Serial.print('-'); v = -v; }
  uint32_t i = (uint32_t)v;
  uint32_t f = (uint32_t)((v - i) * 100.0f + 0.5f);
  if (f >= 100) { i++; f = 0; }
  printUint(i); Serial.print('.');
  if (f < 10) Serial.print('0');
  printUint(f);
}
static void printFloat3(float v) {
  if (v < 0) { Serial.print('-'); v = -v; }
  uint32_t i = (uint32_t)v;
  uint32_t f = (uint32_t)((v - i) * 1000.0f + 0.5f);
  if (f >= 1000) { i++; f = 0; }
  printUint(i); Serial.print('.');
  if (f < 100) Serial.print('0');
  if (f < 10) Serial.print('0');
  printUint(f);
}
static void printPadUint(uint32_t v, uint8_t w) {
  char b[12]; utoa(v, b, 10);
  uint8_t l = strlen(b);
  while (l++ < w) Serial.print(' ');
  Serial.print(b);
}
static void printHex16(uint16_t v) {
  if (v < 0x1000) Serial.print('0');
  if (v < 0x100) Serial.print('0');
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}
static void printCurrentTelemetry(void) {
  Serial.print(F("FORCE=")); printFloat2(g_values.force_N); Serial.print(F(" N "));
  Serial.print(F("MOMENT=")); printFloat2(g_values.moment_Nm); Serial.print(F(" Nm "));
  Serial.print(F("KNEE=")); printFloat2(g_values.kneeAngle_deg); Serial.print(F(" deg "));
  Serial.print(F("VALVE=")); printFloat2(g_values.valvePosition_percent); Serial.print(F(" % "));
  Serial.print(F("BATT=")); printFloat2(g_values.batteryVoltage_V); Serial.print(F(" V "));
  Serial.print(F("STATE=")); printUint(g_values.systemState); Serial.print(' ');
  Serial.print(F("FAULT=")); printUint(g_values.faultCode); Serial.print(F("\r\n"));
}
static void printThresholds(void) {
  Serial.print(F("Thresholds:\r\n Force: min=")); printFloat2(g_thresholds.force_min);
  Serial.print(F(" max=")); printFloat2(g_thresholds.force_max); Serial.print(F(" N\r\n Moment: min="));
  printFloat2(g_thresholds.moment_min); Serial.print(F(" max=")); printFloat2(g_thresholds.moment_max);
  Serial.print(F(" Nm\r\n Knee: min=")); printFloat2(g_thresholds.knee_min);
  Serial.print(F(" max=")); printFloat2(g_thresholds.knee_max); Serial.print(F(" deg\r\n Valve: min="));
  printFloat2(g_thresholds.valve_min); Serial.print(F(" max=")); printFloat2(g_thresholds.valve_max);
  Serial.print(F(" %\r\n Battery: min=")); printFloat2(g_thresholds.batt_min);
  Serial.print(F(" max=")); printFloat2(g_thresholds.batt_max); Serial.print(F(" V\r\n"));
}
static void printCoefficients(void) {
  if (!g_coefficientsActive) { Serial.print(F("Coefficients: not set\r\n")); return; }
  Serial.print(F("=== Coefficients ===\r\nForce Scale    : ")); printFloat3(g_coefficients.force_scale); Serial.print(F(" N/V\r\nMoment Scale   : ")); printFloat3(g_coefficients.moment_scale); Serial.print(F(" Nm/V\r\nKnee Scale     : ")); printFloat3(g_coefficients.knee_scale); Serial.print(F(" deg/V\r\nBattery Scale  : ")); printFloat3(g_coefficients.batt_scale); Serial.print(F(" V/V\r\nADC VREF       : ")); printFloat3(g_coefficients.adc_vref); Serial.print(F(" V\r\nADC Max Counts : ")); printUint((uint32_t)g_coefficients.adc_max); Serial.print(F("\r\nForce Offset   : ")); printFloat3(g_coefficients.force_offset); Serial.print(F(" N\r\nMoment Offset  : ")); printFloat3(g_coefficients.moment_offset); Serial.print(F(" Nm\r\nKnee Offset    : ")); printFloat3(g_coefficients.knee_offset); Serial.print(F(" deg\r\nBattery Offset : ")); printFloat3(g_coefficients.batt_offset); Serial.print(F(" V\r\nADC Sample Cyc : ")); printUint((uint32_t)g_coefficients.adc_smp); Serial.print(F(" (0-7)\r\nADC Clock Freq : ")); printUint((uint32_t)g_coefficients.adc_clk); Serial.print(F(" Hz\r\n"));
}
static void printPrompt(void) { Serial.print(F("CMD> ")); }
static void adcDebug(const char *tag) {
  adcPoll();
  Serial.print(F("\r\n=== ADC debug [")); Serial.print(tag); Serial.print(F("] ===\r\n"));
  Serial.print(F("ATmega328P 10-bit, VREF=")); printFloat3(g_coefficients.adc_vref);
  Serial.print(F("V Max=")); printUint((uint32_t)g_coefficients.adc_max);
  Serial.print(F("\r\nraw[A0..A5]: "));
  for (uint8_t i = 0; i < 6; i++) { printUint(g_raw[i]); Serial.print(' '); }
  Serial.print(F("\r\nvolts[A0..A5]: "));
  for (uint8_t i = 0; i < 6; i++) { printFloat3(adcVoltage(i)); Serial.print(' '); }
  Serial.print(F("\r\nSlave: valid=")); printUint(g_slaveDataValid);
  Serial.print(F(" angle_x100=")); printUint(g_slaveSpoolAngle_x100);
  Serial.print(F("\r\nStuck=0x")); printHex16(adcCheckStuck()); Serial.print(F("\r\n"));
}
static void smPrintStatus(void) {
  const char *mn = g_smMode == 1 ? "AVG" : g_smMode == 2 ? "MAX" : g_smMode == 3 ? "MIN" : "SUM";
  const char *in = g_sinput == 1 ? "MOMENT" : g_sinput == 2 ? "FORCE_KNEE" : "FORCE";
  const char *mm = g_mmMode == 1 ? "DIFF" : g_mmMode == 2 ? "ABSDIFF" : "FORCE_ARM";
  const char *st = g_smState == 2 ? "LOADING" : g_smState == 3 ? "MID-STANCE" : g_smState == 4 ? "TERMINAL-STANCE" : g_smState == 5 ? "PRE-SWING" : g_smState == 1 ? "STANCE" : "SWING";
  Serial.print(F("State=")); Serial.print(st);
  Serial.print(F(" input=")); Serial.print(in);
  Serial.print(F(" mmode=")); Serial.print(mm);
  Serial.print(F(" Force=")); printUint((uint32_t)(g_smForce * 1000)); Serial.print(F("mV Moment=")); printUint((uint32_t)(g_smMoment * 1000));
  Serial.print(F("mVxM window=")); printUint(g_count); Serial.print('/'); printUint(g_window);
  Serial.print(F(" mode=")); Serial.print(mn);
  Serial.print(F(" arm=")); printUint((uint32_t)(g_arm * 1000)); Serial.print(F("mM\r\nBands: SW["));
  printUint(g_swMin * 1000); Serial.print(F("mV..")); printUint(g_swMax * 1000); Serial.print(F("mV] PS["));
  printUint(g_psMin * 1000); Serial.print(F("mV..")); printUint(g_psMax * 1000); Serial.print(F("mV] LO/TS["));
  printUint(g_loMin * 1000); Serial.print(F("mV..")); printUint(g_loMax * 1000); Serial.print(F("mV] MS["));
  printUint(g_msMin * 1000); Serial.print(F("mV..")); printUint(g_msMax * 1000); Serial.print(F("mV] ST>="));
  printUint(g_stMin * 1000); Serial.print(F("mV\r\n"));
  if (g_sinput == SM_INPUT_FORCE_KNEE) {
    Serial.print(F("Knee: SW[")); printFloat2(g_kSwMin); Serial.print(' '); printFloat2(g_kSwMax);
    Serial.print(F("] PS[")); printFloat2(g_kPsMin); Serial.print(' '); printFloat2(g_kPsMax);
    Serial.print(F("] LO[")); printFloat2(g_kLoMin); Serial.print(' '); printFloat2(g_kLoMax);
    Serial.print(F("] MS[")); printFloat2(g_kMsMin); Serial.print(' '); printFloat2(g_kMsMax);
    Serial.print(F("] TS[")); printFloat2(g_kTermMin); Serial.print(' '); printFloat2(g_kTermMax);
    Serial.print(F("] PR[")); printFloat2(g_kPreMin); Serial.print(' '); printFloat2(g_kPreMax);
    Serial.print(F("]\r\n"));
  }
}

/* ============================ CLI helpers ================================ */
static int isFloatStr(const char *s) { char *e; if (!*s) return 0; strtod(s, &e); return *e == 0; }
static int isIntStr(const char *s) {
  if (!*s) return 0; if (*s == '+' || *s == '-') s++; if (!*s) return 0;
  while (*s) { if (!isdigit((unsigned char)*s)) return 0; s++; } return 1;
}
static void toLower(char *s) { while (*s) { *s = tolower((unsigned char)*s); s++; } }

/* ============================ CLI (mirror of main.c) ===================== */
static void processCommand(char *cmd) {
  if (g_termMode == MODE_WAIT_FORCE) {
    if (isFloatStr(cmd)) { g_values.force_N = strtod(cmd, 0); Serial.print(F("Enter moment (Nm):\r\n")); g_termMode = MODE_WAIT_MOMENT; }
    else Serial.print(F("Invalid force. Enter force (N):\r\n")); return;
  } else if (g_termMode == MODE_WAIT_MOMENT) {
    if (isFloatStr(cmd)) { g_values.moment_Nm = strtod(cmd, 0); Serial.print(F("Enter knee angle (deg):\r\n")); g_termMode = MODE_WAIT_KNEE; }
    else Serial.print(F("Invalid moment. Enter moment (Nm):\r\n")); return;
  } else if (g_termMode == MODE_WAIT_KNEE) {
    if (isFloatStr(cmd)) { g_values.kneeAngle_deg = strtod(cmd, 0); Serial.print(F("Enter valve position (%):\r\n")); g_termMode = MODE_WAIT_VALVE; }
    else Serial.print(F("Invalid knee angle. Enter knee angle (deg):\r\n")); return;
  } else if (g_termMode == MODE_WAIT_VALVE) {
    if (isFloatStr(cmd)) { g_values.valvePosition_percent = strtod(cmd, 0); Serial.print(F("Enter battery voltage (V):\r\n")); g_termMode = MODE_WAIT_BATT; }
    else Serial.print(F("Invalid valve position. Enter valve position (%):\r\n")); return;
  } else if (g_termMode == MODE_WAIT_BATT) {
    if (isFloatStr(cmd)) { g_values.batteryVoltage_V = strtod(cmd, 0); Serial.print(F("Enter system state:\r\n")); g_termMode = MODE_WAIT_STATE; }
    else Serial.print(F("Invalid battery voltage. Enter battery voltage (V):\r\n")); return;
  } else if (g_termMode == MODE_WAIT_STATE) {
    if (isIntStr(cmd)) { long v = atol(cmd); if (v < 0 || v > 255) Serial.print(F("Invalid system state range. Enter 0..255:\r\n")); else { g_values.systemState = v; Serial.print(F("Enter fault code:\r\n")); g_termMode = MODE_WAIT_FAULT; } }
    else Serial.print(F("Invalid system state. Enter system state:\r\n")); return;
  } else if (g_termMode == MODE_WAIT_FAULT) {
    if (isIntStr(cmd)) { long v = atol(cmd); if (v < 0 || v > 65535) Serial.print(F("Invalid fault code range. Enter 0..65535:\r\n")); else { g_values.faultCode = v; Serial.print(F("Telemetry updated:\r\n")); printCurrentTelemetry(); g_termMode = MODE_COMMAND; printPrompt(); } }
    else Serial.print(F("Invalid fault code. Enter fault code:\r\n")); return;
  }
  if (g_termMode != MODE_COMMAND) { Serial.print(F("Finish current data entry first.\r\n")); return; }
  toLower(cmd);

  if (!strcmp(cmd, "red on")) { digitalWrite(PIN_LED_RED, HIGH); Serial.print(F("Red LED ON\r\n")); }
  else if (!strcmp(cmd, "red off")) { digitalWrite(PIN_LED_RED, LOW); Serial.print(F("Red LED OFF\r\n")); }
  else if (!strcmp(cmd, "green on")) { digitalWrite(PIN_LED_GREEN, HIGH); Serial.print(F("Green LED ON\r\n")); }
  else if (!strcmp(cmd, "green off")) { digitalWrite(PIN_LED_GREEN, LOW); Serial.print(F("Green LED OFF\r\n")); }
  else if (!strcmp(cmd, "all on")) { digitalWrite(PIN_LED_RED, HIGH); digitalWrite(PIN_LED_GREEN, HIGH); Serial.print(F("All LEDs ON\r\n")); }
  else if (!strcmp(cmd, "all off")) { digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_GREEN, LOW); Serial.print(F("All LEDs OFF\r\n")); }
  else if (!strcmp(cmd, "status")) {
    Serial.print(F("RED:")); Serial.print(digitalRead(PIN_LED_RED) ? F("ON") : F("OFF"));
    Serial.print(F(" GREEN:")); Serial.print(digitalRead(PIN_LED_GREEN) ? F("ON") : F("OFF")); Serial.print(F("\r\n"));
  }
  else if (!strcmp(cmd, "data")) { Serial.print(F("Enter force (N):\r\n")); g_termMode = MODE_WAIT_FORCE; }
  else if (!strcmp(cmd, "show")) printCurrentTelemetry();
  else if (!strcmp(cmd, "crc")) {
    uint16_t len; buildTelemetryFrame(g_txFrame, &len);
    uint16_t c = g_txFrame[23] | ((uint16_t)g_txFrame[24] << 8);
    Serial.print(F("SEQ=")); Serial.print(g_sequence); Serial.print(F(" LEN=")); Serial.print(len);
    Serial.print(F(" CRC16=0x")); printHex16(c); Serial.print(F("\r\nFRAME: "));
    for (uint16_t i = 0; i < len; i++) { if (g_txFrame[i] < 16) Serial.print('0'); Serial.print(g_txFrame[i], HEX); Serial.print(' '); }
    Serial.print(F("\r\n"));
  }
  else if (!strcmp(cmd, "dmatest")) Serial.print(F("DMA TX OK\r\n"));
  else if (!strncmp(cmd, "set force ", 10)) { g_values.force_N = strtod(cmd + 10, 0); Serial.print(F("Force updated\r\n")); }
  else if (!strncmp(cmd, "set moment ", 11)) { g_values.moment_Nm = strtod(cmd + 11, 0); Serial.print(F("Moment updated\r\n")); }
  else if (!strncmp(cmd, "set knee ", 9)) { g_values.kneeAngle_deg = strtod(cmd + 9, 0); Serial.print(F("Knee angle updated\r\n")); }
  else if (!strncmp(cmd, "set valve ", 10)) { g_values.valvePosition_percent = strtod(cmd + 10, 0); Serial.print(F("Valve position updated\r\n")); }
  else if (!strncmp(cmd, "set batt ", 9)) { g_values.batteryVoltage_V = strtod(cmd + 9, 0); Serial.print(F("Battery voltage updated\r\n")); }
  else if (!strncmp(cmd, "set state ", 10)) {
    if (isIntStr(cmd + 10)) { long v = atol(cmd + 10); if (v >= 0 && v <= 255) { g_values.systemState = v; Serial.print(F("System state updated\r\n")); } else Serial.print(F("State must be 0..255\r\n")); }
    else Serial.print(F("Invalid state value\r\n"));
  }
  else if (!strncmp(cmd, "set fault ", 10)) {
    if (isIntStr(cmd + 10)) { long v = atol(cmd + 10); if (v >= 0 && v <= 65535) { g_values.faultCode = v; Serial.print(F("Fault code updated\r\n")); } else Serial.print(F("Fault must be 0..65535\r\n")); }
    else Serial.print(F("Invalid fault value\r\n"));
  }
  else if (!strcmp(cmd, "smart")) { uint16_t l; buildTelemetryFrame(g_txFrame, &l); Serial.write(g_txFrame, l); g_sequence++; }
  else if (!strcmp(cmd, "adc debug")) adcDebug("cmd");
  else if (!strcmp(cmd, "adc")) {
    Serial.print(F("\r\nLive ADC pins (A0-A5) - send any char to exit\r\n"));
    while (1) {
      adcPoll();
      Serial.print(F("A0=")); printPadUint(g_raw[0], 5); Serial.print(F(" (")); printPadUint(adcVoltage(0) * 1000, 4); Serial.print(F("mV) "));
      Serial.print(F("A1=")); printPadUint(g_raw[1], 5); Serial.print(F(" (")); printPadUint(adcVoltage(1) * 1000, 4); Serial.print(F("mV) "));
      Serial.print(F("A2=")); printPadUint(g_raw[2], 5); Serial.print(F(" (")); printPadUint(adcVoltage(2) * 1000, 4); Serial.print(F("mV) "));
      Serial.print(F("A3=")); printPadUint(g_raw[3], 5); Serial.print(F(" (")); printPadUint(adcVoltage(3) * 1000, 4); Serial.print(F("mV) "));
      Serial.print(F("A4=")); printPadUint(g_raw[4], 5); Serial.print(F(" (")); printPadUint(adcVoltage(4) * 1000, 4); Serial.print(F("mV) "));
      Serial.print(F("A5=")); printPadUint(g_raw[5], 5); Serial.print(F(" (")); printPadUint(adcVoltage(5) * 1000, 4); Serial.print(F("mV)\r\n"));
      if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
      delay(500);
    }
    Serial.print(F("Live ADC readout stopped\r\n"));
  }
  else if (!strncmp(cmd, "set cycles ", 11)) {
    if (isIntStr(cmd + 11)) { long v = atol(cmd + 11); if (v >= 1 && v <= SM_MAX_WINDOW) { g_window = v; g_count = 0; Serial.print(F("Window size set to ")); Serial.print(v); Serial.print(F(" cycles\r\n")); } else Serial.print(F("Cycles must be 1..100\r\n")); }
    else Serial.print(F("Invalid cycles value\r\n"));
  }
  else if (!strncmp(cmd, "set lmode ", 10)) {
    if (isIntStr(cmd + 10)) { long v = atol(cmd + 10); if (v >= 0 && v <= 3) { g_smMode = v; g_count = 0; Serial.print(F("Load-cell mode set (0=SUM 1=AVG 2=MAX 3=MIN)\r\n")); } else Serial.print(F("Mode must be 0..3 (0=SUM 1=AVG 2=MAX 3=MIN)\r\n")); }
    else Serial.print(F("Invalid mode value\r\n"));
  }
  else if (!strncmp(cmd, "set arm ", 8)) {
    if (isFloatStr(cmd + 8)) { float v = strtod(cmd + 8, 0); if (v > 0) { g_arm = v; Serial.print(F("Moment arm updated\r\n")); } else Serial.print(F("Arm must be > 0\r\n")); }
    else Serial.print(F("Invalid arm value\r\n"));
  }
  else if (!strncmp(cmd, "set stance ", 11)) {
    if (isFloatStr(cmd + 11)) { float v = strtod(cmd + 11, 0); if (v >= 0) { g_stMin = v; Serial.print(F("Stance min updated\r\n")); } else Serial.print(F("Threshold must be >= 0\r\n")); }
    else Serial.print(F("Invalid threshold value\r\n"));
  }
  else if (!strcmp(cmd, "sm")) smPrintStatus();
  else if (!strncmp(cmd, "set stmax ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0 && v >= g_stMin) { g_stMax = v; Serial.print(F("Stance range max updated\r\n")); } else Serial.print(F("Max must be >= 0 and >= stance min\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set swmin ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0) { g_swMin = v; Serial.print(F("Swing range min updated\r\n")); } else Serial.print(F("Min must be >= 0\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set swmax ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0 && v >= g_swMin) { g_swMax = v; Serial.print(F("Swing range max updated\r\n")); } else Serial.print(F("Max must be >= 0 and >= swing min\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set psmin ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0) { g_psMin = v; Serial.print(F("Pre-swing min updated\r\n")); } else Serial.print(F("Min must be >= 0\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set psmax ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0 && v >= g_psMin) { g_psMax = v; Serial.print(F("Pre-swing max updated\r\n")); } else Serial.print(F("Max must be >= 0 and >= pre-swing min\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set lomin ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0) { g_loMin = v; Serial.print(F("Loading/terminal min updated\r\n")); } else Serial.print(F("Min must be >= 0\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set lomax ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0 && v >= g_loMin) { g_loMax = v; Serial.print(F("Loading/terminal max updated\r\n")); } else Serial.print(F("Max must be >= 0 and >= loading min\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set msmin ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0) { g_msMin = v; Serial.print(F("Mid-stance min updated\r\n")); } else Serial.print(F("Min must be >= 0\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set msmax ", 10)) {
    if (isFloatStr(cmd + 10)) { float v = strtod(cmd + 10, 0); if (v >= 0 && v >= g_msMin) { g_msMax = v; Serial.print(F("Mid-stance max updated\r\n")); } else Serial.print(F("Max must be >= 0 and >= mid-stance min\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set sinput ", 11)) {
    if (isIntStr(cmd + 11)) { long v = atol(cmd + 11); if (v >= 0 && v <= 2) { g_sinput = v; Serial.print(v == 0 ? F("Decision input set (0=FORCE)\r\n") : v == 1 ? F("Decision input set (1=MOMENT)\r\n") : F("Decision input set (2=FORCE_KNEE)\r\n")); } else Serial.print(F("Input must be 0 (FORCE), 1 (MOMENT), or 2 (FORCE_KNEE)\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set mmode ", 10)) {
    if (isIntStr(cmd + 10)) { long v = atol(cmd + 10); if (v >= 0 && v <= 2) { g_mmMode = v; Serial.print(F("Moment mode set (0=FORCE_ARM 1=DIFF 2=ABSDIFF)\r\n")); } else Serial.print(F("Mode must be 0 (FORCE_ARM), 1 (DIFF), or 2 (ABSDIFF)\r\n")); }
    else Serial.print(F("Invalid value\r\n"));
  }
  else if (!strncmp(cmd, "set kswmin ", 11)) { g_kSwMin = strtod(cmd + 11, 0); Serial.print(F("Knee swing min set\r\n")); }
  else if (!strncmp(cmd, "set kswmax ", 11)) { g_kSwMax = strtod(cmd + 11, 0); Serial.print(F("Knee swing max set\r\n")); }
  else if (!strncmp(cmd, "set kpsmin ", 11)) { g_kPsMin = strtod(cmd + 11, 0); Serial.print(F("Knee pre-swing min set\r\n")); }
  else if (!strncmp(cmd, "set kpsmax ", 11)) { g_kPsMax = strtod(cmd + 11, 0); Serial.print(F("Knee pre-swing max set\r\n")); }
  else if (!strncmp(cmd, "set klo_min ", 11)) { g_kLoMin = strtod(cmd + 11, 0); Serial.print(F("Knee loading min set\r\n")); }
  else if (!strncmp(cmd, "set klo_max ", 11)) { g_kLoMax = strtod(cmd + 11, 0); Serial.print(F("Knee loading max set\r\n")); }
  else if (!strncmp(cmd, "set kmsmin ", 11)) { g_kMsMin = strtod(cmd + 11, 0); Serial.print(F("Knee mid-stance min set\r\n")); }
  else if (!strncmp(cmd, "set kmsmax ", 11)) { g_kMsMax = strtod(cmd + 11, 0); Serial.print(F("Knee mid-stance max set\r\n")); }
  else if (!strncmp(cmd, "set ktermmin ", 13)) { g_kTermMin = strtod(cmd + 13, 0); Serial.print(F("Knee terminal min set\r\n")); }
  else if (!strncmp(cmd, "set ktermmax ", 13)) { g_kTermMax = strtod(cmd + 13, 0); Serial.print(F("Knee terminal max set\r\n")); }
  else if (!strncmp(cmd, "set kpremin ", 12)) { g_kPreMin = strtod(cmd + 12, 0); Serial.print(F("Knee pre-swing min (alt) set\r\n")); }
  else if (!strncmp(cmd, "set kpremax ", 12)) { g_kPreMax = strtod(cmd + 12, 0); Serial.print(F("Knee pre-swing max (alt) set\r\n")); }
  else if (!strcmp(cmd, "slave")) {
    if (g_lastSentState == 1) Serial.print(F("Last sent to slave: STANCE (1) frame A5 02 01 01 xx xx\r\n"));
    else if (g_lastSentState == 0) Serial.print(F("Last sent to slave: SWING (0) frame A5 02 01 00 xx xx\r\n"));
    else Serial.print(F("Nothing sent to slave yet\r\n"));
  }
  else if (!strcmp(cmd, "halt on")) { g_systemHalt = 1; Serial.print(F("System HALTED - telemetry and valve commands stopped\r\n")); }
  else if (!strcmp(cmd, "halt off")) { g_systemHalt = 0; Serial.print(F("System resumed\r\n")); }
  else if (!strcmp(cmd, "halt")) Serial.print(g_systemHalt ? F("System is HALTED\r\n") : F("System is RUNNING\r\n"));
  else if (!strcmp(cmd, "demo on")) { g_streaming = 1; Serial.print(F("Demo stream ON (100 Hz)\r\n")); }
  else if (!strcmp(cmd, "demo off")) { g_streaming = 0; Serial.print(F("Demo stream OFF\r\n")); }
  else if (!strcmp(cmd, "mode demo")) { g_mode = MODE_DEMO; Serial.print(F("Switched to DEMO mode (simulated data)\r\n")); }
  else if (!strcmp(cmd, "mode real")) { g_mode = MODE_REAL; Serial.print(F("Switched to REAL mode (ADC sensor data)\r\n")); }
  else if (!strcmp(cmd, "get thresholds") || !strcmp(cmd, "thr")) printThresholds();
  else if (!strcmp(cmd, "get coefficients")) printCoefficients();
  else if (!strcmp(cmd, "ble status")) Serial.print(F("BLE: not supported on Uno\r\n"));
  else if (!strcmp(cmd, "ble provision")) Serial.print(F("BLE provision: not supported on Uno\r\n"));
  else if (!strncmp(cmd, "crypto ", 7) || !strcmp(cmd, "crypto")) Serial.print(F("Crypto: not supported on Uno - plaintext only\r\n"));
  else if (!strncmp(cmd, "set key ", 8)) Serial.print(F("Refused: no crypto on Uno\r\n"));
  else if (!strcmp(cmd, "help")) {
    Serial.print(F("Commands: red on, red off, green on, green off, all on, all off, status, show, data, crc, dmatest, set force x, set moment x, set knee x, set valve x, set batt x, set state x, set fault x, smart, adc, adc debug, set cycles N, set lmode N, set arm x, set stance x, set stmax x, set swmin x, set swmax x, set psmin x, set psmax x, set lomin x, set lomax x, set msmin x, set msmax x, set sinput N, set mmode N, set kswmin x, set kswmax x, set kpsmin x, set kpsmax x, set klo_min x, set klo_max x, set kmsmin x, set kmsmax x, set ktermmin x, set ktermmax x, set kpremin x, set kpremax x, sm, slave, halt on, halt off, halt, demo on, demo off, mode demo, mode real, get thresholds, thr, get coefficients, ble status, help\r\n"));
  }
  else Serial.print(F("Unknown command\r\n"));
  if (g_termMode == MODE_COMMAND) printPrompt();
}

/* ============================ RX front-end =============================== */
static char g_cmdBuf[128]; static uint32_t g_cmdIdx = 0;
static uint8_t g_binBuf[64]; static uint16_t g_binIdx = 0; static uint8_t g_inBinary = 0;
static void rxByte(uint8_t ch) {
  if (g_inBinary) {
    if (g_binIdx < sizeof(g_binBuf)) g_binBuf[g_binIdx++] = ch;
    if (g_binIdx >= THR_PACKET_MIN_LEN && g_binBuf[1] == THR_PACKET_TYPE) {
      Serial.print(processThresholdPacket(g_binBuf, g_binIdx) ? F("Thresholds updated from PC\r\n") : F("Threshold packet invalid (CRC/length)\r\n"));
      g_binIdx = 0; g_inBinary = 0; return;
    }
    if (g_binIdx >= COEFF_PACKET_MIN_LEN && g_binBuf[1] == COEFF_PACKET_TYPE) {
      g_systemHalt = 1;
      Serial.print(processCoeffPacket(g_binBuf, g_binIdx) ? F("Coefficients updated from PC\r\n") : F("Coefficient packet invalid (CRC/length)\r\n"));
      g_systemHalt = 0; g_binIdx = 0; g_inBinary = 0; return;
    }
    return;
  }
  if (g_cmdIdx == 0 && ch == THR_PACKET_SYNC) { g_inBinary = 1; g_binBuf[0] = ch; g_binIdx = 1; return; }
  if (ch == '\r' || ch == '\n') {
    if (g_cmdIdx > 0) { g_cmdBuf[g_cmdIdx] = 0; processCommand(g_cmdBuf); g_cmdIdx = 0; memset(g_cmdBuf, 0, sizeof(g_cmdBuf)); }
  } else if (ch == '\b' || ch == 127) { if (g_cmdIdx > 0) g_cmdBuf[--g_cmdIdx] = 0; }
  else { if (g_cmdIdx < sizeof(g_cmdBuf) - 1) g_cmdBuf[g_cmdIdx++] = ch; else { g_cmdIdx = 0; memset(g_cmdBuf, 0, sizeof(g_cmdBuf)); } }
}

/* ============================ setup / loop =============================== */
void setup(void) {
  pinMode(PIN_LED_RED, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_GREEN, LOW);
  Serial.begin(CONSOLE_BAUD);
#if USE_SLAVE_UART
  slaveSerial.begin(SLAVE_BAUD);
#endif
  g_coefficients.force_scale = 300.0f; g_coefficients.moment_scale = 60.0f;
  g_coefficients.knee_scale = 27.0f; g_coefficients.batt_scale = 1.41f;
  g_coefficients.adc_vref = 5.0f; g_coefficients.adc_max = 1023.0f;
  g_coefficients.force_offset = g_coefficients.moment_offset = 0;
  g_coefficients.knee_offset = g_coefficients.batt_offset = 0;
  g_coefficients.adc_smp = 0; g_coefficients.adc_clk = 4000000.0f;
  g_coefficientsActive = 1;
  Serial.print(F("\r\nUART Terminal Ready\r\n"));
  Serial.print(F("Commands: red on, red off, green on, green off, all on, all off, status, show, data, crc, dmatest, set force x, set moment x, set knee x, set valve x, set batt x, set state x, set fault x, smart, help\r\n"));
  if (coeffLoadFromFlash()) Serial.print(F("Coefficients restored from flash\r\n"));
#if USE_SLAVE_UART
  Serial.print(F("Slave UART on D10(RX)/D11(TX) @115200\r\n"));
#else
  Serial.print(F("Slave UART disabled - valve = A3 fallback\r\n"));
#endif
  printPrompt();
  smInit();
  adcPoll();
}

void loop(void) {
  static uint32_t lastTick = 0;
  slavePoll();
  uint32_t now = millis();
  if ((now - lastTick) >= 10) {  /* 100 Hz */
    lastTick = (now - lastTick) > 20 ? now : lastTick + 10;  /* no drift pile-up */
    if (g_streaming) telemetryTick();
  }
  while (Serial.available()) rxByte((uint8_t)Serial.read());
}
