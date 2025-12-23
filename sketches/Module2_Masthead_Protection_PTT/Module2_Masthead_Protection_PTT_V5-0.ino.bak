/*
  Module2_Masthead_Protection_PTT.ino
  Masthead PTT protection + telemetry node

  - Receives RSCP_ID_PTT_CMD from station:
      data0 = 1(on)/0(off)
      data1 = seq
      data2 = flags (optional, currently unused here)

  - Sends RSCP_ID_PTT_STATUS back:
      data0 = state (0/1)
      data1 = seq
      data2 = status_flags (local bitfield; see below)
      data3 = err_latched (1/0)
      data4..7 reserved (0)

  - Sends RSCP_ID_ERRORS (uint32 bitmask in data0..3) whenever latched changes.
  - Sends telemetry frames for voltage/vswr/drive/temp/hum when enabled.

  IMPORTANT:
  This sketch is written to match your RSCP header naming:
    - RSCP_ERR_OVERTIME (NOT RSCP_ERR_PTT_OVERTIME)
    - rscp_put_u16 / rscp_get_u16 (NOT rscp_le_to_u16)
*/

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "rscp_can_protocol_fixed.h"   // protocol header (single source of truth)

// ------------------------- Feature toggles -------------------------
// Primary environment sensor on D8:
//   Default = DHT11. To switch to DHT22, set DHT1_IS_DHT11 to 0.
// Secondary temperature sensor (optional, NOT enabled by default):
//   - DS18B20 on D9  (ENABLE_TEMP2_DS18B20=1)
//   - OR DHT11/DHT22 on D9 (ENABLE_TEMP2_DHTxx=1)
//
// Notes:
// - We keep the "fast loop" rule: PTT handled every loop, max one sensor task per loop.
// - Temp is sampled even when PTT is ON; humidity is sampled only when PTT is OFF.

#define ENABLE_TCS34725       0   // 1 to enable colour sensor (requires Adafruit_TCS34725 lib)
#define HAVE_PTT_CONFIRM      1   // 0 if you don't have the confirm sense wiring

// ---- Sensor #1 (default enabled) ----
#define ENABLE_DHT1           1   // 1 to enable primary DHT sensor on D8
#define DHT1_IS_DHT11         1   // 1=DHT11, 0=DHT22

// ---- Sensor #2 (default disabled) ----
#define ENABLE_TEMP2_DS18B20  0   // 1 to enable DS18B20 as Temp2 on D9
#define ENABLE_TEMP2_DHTxx    0   // 1 to enable DHT as Temp2 on D9 (temp only)
#define DHT2_IS_DHT11         1   // 1=DHT11, 0=DHT22 (only if ENABLE_TEMP2_DHTxx=1)

// ------------------------- Optional libraries -------------------------
#if ENABLE_TCS34725
  #include <Wire.h>
  #include <Adafruit_TCS34725.h>
#endif

#if ENABLE_DHT1 || ENABLE_TEMP2_DHTxx
  #include <DHT.h>
#endif

#if ENABLE_TEMP2_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

// ------------------------- Pin mapping -------------------------
// Aligned to Pinouts.md + Module2 pin table (Arduino Nano):
//   D10 CAN_CS, D2 CAN_INT, D7 PTT_OUT, D6 PTT_CONFIRM, D4 PTT_MIRROR_LED,
//   D8 DHT11/DHT22 (Temp+Hum), D9 optional Temp2 (DS18B20 OR DHT),
//   D5 INHIBIT_IN (optional), D3 spare (used for DHT if enabled),
//   A0 FWD_RAW, A1 REV_RAW, A2 VBAT_RAW, A4/A5 I2C for colour sensor.

#ifndef PIN_CAN_CS
  #define PIN_CAN_CS          10
#endif
#ifndef PIN_CAN_INT
  #define PIN_CAN_INT         2
#endif

#ifndef PIN_PTT_OUT
  #define PIN_PTT_OUT         7   // Keys transverter/linear via opto/relay
#endif

#ifndef PIN_PTT_CONFIRM
  #define PIN_PTT_CONFIRM     6   // Sense input confirming remote switching
#endif

#ifndef PIN_PTT_MIRROR_LED
  #define PIN_PTT_MIRROR_LED  4   // Mirrors PTT state locally
#endif

#ifndef PIN_PTT_INHIBIT
  #define PIN_PTT_INHIBIT     5   // Optional inhibit/interlock input (INPUT_PULLUP)
#endif

#ifndef PIN_DHT1
  #define PIN_DHT1            8   // DHT11/DHT22 primary sensor (TEMP+HUM)
#endif

#ifndef PIN_TEMP2
  #define PIN_TEMP2           9   // Optional Temp2 sensor pin (DS18B20 OR DHT)
#endif


#ifndef PIN_FWD_ADC
  #define PIN_FWD_ADC         A0  // FWD_RAW
#endif
#ifndef PIN_REV_ADC
  #define PIN_REV_ADC         A1  // REV_RAW
#endif
#ifndef PIN_VBAT_ADC
  #define PIN_VBAT_ADC        A2  // VBAT_RAW
#endif

// Back-compat aliases for older names used in this sketch:
#ifndef PIN_VSWR1_ADC
  #define PIN_VSWR1_ADC       PIN_FWD_ADC
#endif
#ifndef PIN_VSWR2_ADC
  #define PIN_VSWR2_ADC       PIN_REV_ADC
#endif

// Analog drive channels are NOT used with the colour sensor mapping.
// Keep placeholders on A3 (spare analog) so the sketch still compiles if referenced.
#ifndef PIN_DRV1_ADC
  #define PIN_DRV1_ADC        A3
#endif
#ifndef PIN_DRV2_ADC
  #define PIN_DRV2_ADC        A3
#endif

// ------------------------- CAN -------------------------
static MCP2515 mcp2515(PIN_CAN_CS);
static struct can_frame canRx;
static struct can_frame canTx;

// ------------------------- Local status_flags bits -------------------------
// These are NOT defined in the RSCP header, so define them locally.
// Station can decode these later if you want.
static const uint8_t PTTSTAT_CONFIRMED_BIT = (1u << 0);
static const uint8_t PTTSTAT_INHIBITED_BIT = (1u << 1);

// ------------------------- Settings received from station -------------------------
static uint32_t g_stop_ptt_err_mask = (RSCP_ERR_PTT_FAIL | RSCP_ERR_VSWR_HIGH | RSCP_ERR_VBAT_LOW | RSCP_ERR_TEMP_HIGH | RSCP_ERR_HUM_HIGH | RSCP_ERR_SENSOR_FAIL | RSCP_ERR_OVERTIME);
static uint32_t g_ptt_max_ms = 300000UL; // default 300s

// Thresholds / limits (received from station via CFG_SET)
static uint16_t g_vbat_low_raw   = 0;
static uint16_t g_vswr1_high_raw = 0;
static uint16_t g_vswr2_high_raw = 0;
static int16_t  g_temp1_high_x10 = 0;
static int16_t  g_temp2_high_x10 = 0;
static uint8_t  g_hum_high_pct   = 0;


// ------------------------- Runtime state -------------------------
static bool     ptt_state = false;
static uint8_t  last_ptt_seq = 0;
static uint8_t  g_status_flags = 0;
static uint32_t g_err_latched = RSCP_ERR_NONE;
static bool     g_err_any_latched = false;

static uint32_t ptt_on_ms = 0;

// ------------------------- Cached telemetry values -------------------------
static uint16_t vbat_raw_cache = 0;
static int16_t  temp1_x10_cache = 0;
static int16_t  temp2_x10_cache = 0;
static uint8_t  hum_cache = 0;
static uint8_t  vswr_state_cache = RSCP_VSWR_NA;
static uint8_t  drive_state_cache = RSCP_DRIVE_NA;

// ------------------------- Sensors (optional) -------------------------
#if ENABLE_TCS34725
static Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
static bool tcs_ok = false;
#endif

#if ENABLE_DHT1
  #if DHT1_IS_DHT11
    static DHT dht1(PIN_DHT1, DHT11);
  #else
    static DHT dht1(PIN_DHT1, DHT22);
  #endif
#endif

#if ENABLE_TEMP2_DHTxx
  #if DHT2_IS_DHT11
    static DHT dht2(PIN_TEMP2, DHT11);
  #else
    static DHT dht2(PIN_TEMP2, DHT22);
  #endif
#endif

#if ENABLE_TEMP2_DS18B20
  static OneWire ow2(PIN_TEMP2);
  static DallasTemperature ds2(&ow2);
#endif

// ------------------------- Helpers -------------------------

static void canSend(uint16_t id, const uint8_t *data, uint8_t dlc) {
  canTx.can_id  = id;
  canTx.can_dlc = dlc;
  for (uint8_t i = 0; i < 8; i++) canTx.data[i] = (i < dlc) ? data[i] : 0;
  mcp2515.sendMessage(&canTx);
}

static void sendPttStatus() {
  uint8_t d[8] = {0};
  d[0] = ptt_state ? 1 : 0;
  d[1] = last_ptt_seq;
  d[2] = g_status_flags;
  d[3] = g_err_any_latched ? 1 : 0;
  canSend(RSCP_ID_PTT_STATUS, d, 8);
}

static void sendErrors() {
  uint8_t d[8] = {0};
  d[0] = RSCP_MOD_MAST;
  d[1] = (g_err_latched == RSCP_ERR_NONE) ? RSCP_SEV_INFO : RSCP_SEV_ERROR;
  rscp_put_u32(&d[2], g_err_latched);
  canSend(RSCP_ID_ERR_STATUS, d, 8);
  g_err_any_latched = (g_err_latched != RSCP_ERR_NONE);
}

static void latchError(uint32_t errBit) {
  const uint32_t before = g_err_latched;
  g_err_latched |= errBit;
  g_err_any_latched = (g_err_latched != RSCP_ERR_NONE);

  if (g_err_latched != before) {
    sendErrors();
  }

  // stop PTT only if this error is configured to stop it
  if ((g_stop_ptt_err_mask & errBit) != 0) {
    // force PTT off
    ptt_state = false;
    digitalWrite(PIN_PTT_OUT, LOW);
      digitalWrite(PIN_PTT_MIRROR_LED, LOW);
    g_status_flags &= ~PTTSTAT_CONFIRMED_BIT;
    sendPttStatus();
  }
}

static void clearErrorsLocalOnly() {
  // NOTE: You said NO auto-clearing. This is here in case later you add a command.
  g_err_latched = RSCP_ERR_NONE;
  g_err_any_latched = false;
  sendErrors();
  sendPttStatus();
}

static bool pttConfirmActive() {
#if HAVE_PTT_CONFIRM
  // Define your polarity here; assuming LOW means “relay pulled in / successful switch”
  return digitalRead(PIN_PTT_CONFIRM) == LOW;
#else
  return true;
#endif
}

static bool pttInhibited() {
  // Optional inhibit/interlock input (HIGH = inhibited)
  return digitalRead(PIN_PTT_INHIBIT) == HIGH;
}

static void applyPtt(uint8_t wantOn) {
  if (wantOn) {
    if (pttInhibited()) {
      g_status_flags |= PTTSTAT_INHIBITED_BIT;
      latchError(RSCP_ERR_PTT_FAIL);   // treat inhibit as PTT fail for now
      ptt_state = false;
      digitalWrite(PIN_PTT_OUT, LOW);
      digitalWrite(PIN_PTT_MIRROR_LED, LOW);
      sendPttStatus();
      return;
    }

    g_status_flags &= ~PTTSTAT_INHIBITED_BIT;

    ptt_state = true;
    digitalWrite(PIN_PTT_OUT, HIGH);
    digitalWrite(PIN_PTT_MIRROR_LED, HIGH);
    ptt_on_ms = millis();

    // Confirm sensing (optional)
    if (pttConfirmActive()) g_status_flags |= PTTSTAT_CONFIRMED_BIT;
    else                    g_status_flags &= ~PTTSTAT_CONFIRMED_BIT;

    sendPttStatus();
  } else {
    ptt_state = false;
    digitalWrite(PIN_PTT_OUT, LOW);
      digitalWrite(PIN_PTT_MIRROR_LED, LOW);
    g_status_flags &= ~(PTTSTAT_CONFIRMED_BIT | PTTSTAT_INHIBITED_BIT);
    sendPttStatus();
  }
}

// Placeholder conversions (you asked: write functions now; scaling later)
static uint16_t readAdcRaw(uint8_t pin) {
  return (uint16_t)analogRead(pin);
}

// ------------------------- Setup / Loop -------------------------
void setup() {
  pinMode(PIN_PTT_OUT, OUTPUT);
    pinMode(PIN_PTT_MIRROR_LED, OUTPUT);
  digitalWrite(PIN_PTT_MIRROR_LED, LOW);
digitalWrite(PIN_PTT_OUT, LOW);
      digitalWrite(PIN_PTT_MIRROR_LED, LOW);

  pinMode(PIN_PTT_INHIBIT, INPUT_PULLUP);

#if HAVE_PTT_CONFIRM
  pinMode(PIN_PTT_CONFIRM, INPUT_PULLUP);
#endif

  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

#if ENABLE_TCS34725
  Wire.begin();
  tcs_ok = tcs.begin();
#endif


#if ENABLE_DHT1
  dht1.begin();
#endif

#if ENABLE_TEMP2_DHTxx
  dht2.begin();
#endif

#if ENABLE_TEMP2_DS18B20
  ds2.begin();
#endif

// announce boot
  {
    uint8_t d[8] = {0};
    d[0] = RSCP_MOD_MAST;  // “MAST” module id in your header enum
    canSend(RSCP_ID_BOOT, d, 8);
  }

  sendErrors();
  sendPttStatus();
}

static void handleCfgFrame(const struct can_frame &f) {
  // RSCP_ID_CFG_SET: data0=destModule, data1=key, data2..3 unused, data4..7 value (LE u32)
  // RSCP_ID_CFG_ACK: data0=module, data1=key, data2=status(0 ok else error)

  const uint8_t dest = f.data[0];
  if (dest != RSCP_MOD_MAST && dest != RSCP_MOD_BROADCAST) return;

  const uint8_t key = f.data[1];
  const uint32_t val = rscp_get_u32(&f.data[4]);

  uint8_t ack[8] = {0};
  ack[0] = RSCP_MOD_MAST;
  ack[1] = key;
  ack[2] = 0; // OK

  switch (key) {
    case RSCP_CFG_PTT_MAX_MS:
      g_ptt_max_ms = (val < 1000UL) ? 1000UL : val;
      break;

    case RSCP_CFG_STOP_PTT_ERR_MASK:
      g_stop_ptt_err_mask = val;
      break;

    case RSCP_CFG_VBAT_LOW_RAW:
      g_vbat_low_raw = (uint16_t)val;
      break;

    case RSCP_CFG_VSWR1_HIGH_RAW:
      g_vswr1_high_raw = (uint16_t)val;
      break;

    case RSCP_CFG_VSWR2_HIGH_RAW:
      g_vswr2_high_raw = (uint16_t)val;
      break;

    case RSCP_CFG_TEMP1_HIGH_X10:
      g_temp1_high_x10 = (int16_t)(uint16_t)val;
      break;

    case RSCP_CFG_TEMP2_HIGH_X10:
      g_temp2_high_x10 = (int16_t)(uint16_t)val;
      break;

    case RSCP_CFG_HUM_HIGH_PCT:
      g_hum_high_pct = (uint8_t)val;
      break;

    default:
      ack[2] = 1; // unknown key
      break;
  }

  canSend(RSCP_ID_CFG_ACK, ack, 8);
}

static void processCan() {
  if (mcp2515.readMessage(&canRx) != MCP2515::ERROR_OK) return;

  switch (canRx.can_id) {
    case RSCP_ID_EMERG_PTT_OFF:
        // Emergency stop from station
        setPtt(false, 0, true);
        break;

      case RSCP_ID_PTT_CMD: {
      const uint8_t want = canRx.data[0];
      last_ptt_seq = canRx.data[1];
      applyPtt(want);
    } break;

    case RSCP_ID_CFG_SET:
      handleCfgFrame(canRx);
      break;

    // Later: add query handlers (CFG_GET etc) if you want
    default:
      break;
  }
}

void loop() {
  const uint32_t now = millis();

  // Fast path: always process CAN quickly
  processCan();

  // If PTT is ON, enforce overtime
  if (ptt_state) {
    const uint32_t elapsed = now - ptt_on_ms;
    if (elapsed > g_ptt_max_ms) {
      latchError(RSCP_ERR_OVERTIME);
    }

#if HAVE_PTT_CONFIRM
    // Confirm sense drop-out check (optional)
    if (!pttConfirmActive()) {
      latchError(RSCP_ERR_PTT_FAIL);
    }
#endif
  }

  // ---------------- One-per-loop sensor / telemetry task ----------------
  static uint8_t phase = 0;
  static uint32_t last_env_ms = 0;
  static uint32_t last_rf_ms  = 0;

  // Stagger tasks; never block
  phase = (uint8_t)((phase + 1) % 5);

  // Always keep VBAT fairly current (used for safety checks)
  if (phase == 0) {
    vbat_raw_cache = readAdcRaw(PIN_VBAT_ADC);
    if (g_vbat_low_raw > 0 && vbat_raw_cache < g_vbat_low_raw) {
      latchError(RSCP_ERR_VBAT_LOW);
    }
  }

#if ENABLE_DHT1
  // DHT temp is useful always; humidity only when PTT is OFF (your rule).
  if (phase == 1) {
    const float t = dht1.readTemperature();
    if (!isnan(t)) temp1_x10_cache = (int16_t)lroundf(t * 10.0f);

    if (!ptt_state) {
      const float h = dht1.readHumidity();
      if (!isnan(h)) hum_cache = (uint8_t)lroundf(h);
      if (g_hum_high_pct > 0 && hum_cache > g_hum_high_pct) {
        latchError(RSCP_ERR_HUM_HIGH);
      }
    }

    if (g_temp1_high_x10 != 0 && temp1_x10_cache > g_temp1_high_x10) {
      latchError(RSCP_ERR_TEMP_HIGH);
    }
  }
#endif



#if ENABLE_TEMP2_DS18B20
  if (phase == 2) {
    ds2.requestTemperatures();
    const float t = ds2.getTempCByIndex(0);
    if (t > -100.0f && t < 150.0f) {
      temp2_x10_cache = (int16_t)lroundf(t * 10.0f);
    }
    if (g_temp2_high_x10 != 0 && temp2_x10_cache > g_temp2_high_x10) {
      latchError(RSCP_ERR_TEMP_HIGH);
    }
  }
#endif

#if ENABLE_TEMP2_DHTxx
  if (phase == 2) {
    const float t = dht2.readTemperature();
    if (!isnan(t)) temp2_x10_cache = (int16_t)lroundf(t * 10.0f);
    if (g_temp2_high_x10 != 0 && temp2_x10_cache > g_temp2_high_x10) {
      latchError(RSCP_ERR_TEMP_HIGH);
    }
  }
#endif

  // RF-related ADC reads only when PTT ON (per your performance rule)
  if (phase == 4 && ptt_state) {
    const uint16_t vswr1 = readAdcRaw(PIN_VSWR1_ADC);
    const uint16_t vswr2 = readAdcRaw(PIN_VSWR2_ADC);

    // Placeholder compare (raw threshold); VSWR calc can be added later
    if ((g_vswr1_high_raw > 0 && vswr1 > g_vswr1_high_raw) ||
        (g_vswr2_high_raw > 0 && vswr2 > g_vswr2_high_raw)) {
      vswr_state_cache = RSCP_VSWR_HIGH;
      latchError(RSCP_ERR_VSWR_HIGH);
    } else {
      vswr_state_cache = RSCP_VSWR_OK;
    }

    // Drive: if colour sensor disabled, report NA for now
#if ENABLE_TCS34725
    drive_state_cache = RSCP_DRIVE_OK; // TODO: map colour to low/high
#else
    drive_state_cache = RSCP_DRIVE_NA;
#endif
  }

  // ---------------- Periodic telemetry sends (non-blocking) -------------
  // ENV telemetry always (but you can choose to slow this down during PTT later)
  if ((now - last_env_ms) >= 1000UL) {
    sendEnvTelem(temp1_x10_cache, hum_cache, vbat_raw_cache);
    last_env_ms = now;
  }

  // RF telemetry only when PTT ON
  if (ptt_state && (now - last_rf_ms) >= 250UL) {
    sendRfTelem(vswr_state_cache, drive_state_cache);
    last_rf_ms = now;
  }
}