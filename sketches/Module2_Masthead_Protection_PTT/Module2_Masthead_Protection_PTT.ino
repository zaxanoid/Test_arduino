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

#include "rscp_can_protocol.h"   // from libraries/RSCP/src/...

// ------------------------- Feature toggles -------------------------
#define ENABLE_TCS34725   0   // set 1 to enable colour sensor (requires Adafruit_TCS34725 lib)
#define HAVE_PTT_CONFIRM  1   // set 0 if you don't have the confirm sense wiring
#define ENABLE_DHTxx      1   // set 1 to enable DHT sensor (DHT11/DHT22)
#define DHT_IS_DHT11      0   // 1=DHT11, 0=DHT22 (only used if ENABLE_DHTxx=1)
#define ENABLE_DS18B20_1  0   // set 1 to enable DS18B20 #1 (OneWire)
#define ENABLE_DS18B20_2  0   // set 1 to enable DS18B20 #2 (OneWire)

// ------------------------- Optional libraries -------------------------
#if ENABLE_TCS34725
  #include <Wire.h>
  #include <Adafruit_TCS34725.h>
#endif

#if ENABLE_DHTxx
  #include <DHT.h>
#endif

#if ENABLE_DS18B20_1 || ENABLE_DS18B20_2
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

// ------------------------- Pin mapping -------------------------
// NOTE: You told me “use pin mappings from remote_V5.2.ino”.
// I can’t see that file inside this chat right now, so these are placeholders.
// Replace these defines with the exact pin numbers from remote_V5.2.ino.

#ifndef PIN_CAN_CS
  #define PIN_CAN_CS      10
#endif
#ifndef PIN_CAN_INT
  #define PIN_CAN_INT     2
#endif

#ifndef PIN_PTT_OUT
  #define PIN_PTT_OUT     4   // output to opto/relay to key TX
#endif

#ifndef PIN_PTT_CONFIRM
  #define PIN_PTT_CONFIRM 5   // sense input showing PTT actually took effect (optional)
#endif

#ifndef PIN_PTT_INHIBIT
  #define PIN_PTT_INHIBIT 6   // optional inhibit input (interlock)
#endif

#ifndef PIN_DHT
  #define PIN_DHT         7
#endif

#ifndef PIN_DS18B20_1
  #define PIN_DS18B20_1   8
#endif

#ifndef PIN_DS18B20_2
  #define PIN_DS18B20_2   9
#endif

// Example analogs (adjust to your wiring)
#ifndef PIN_VBAT_ADC
  #define PIN_VBAT_ADC    A0
#endif
#ifndef PIN_VSWR1_ADC
  #define PIN_VSWR1_ADC   A1
#endif
#ifndef PIN_VSWR2_ADC
  #define PIN_VSWR2_ADC   A2
#endif
#ifndef PIN_DRV1_ADC
  #define PIN_DRV1_ADC    A3
#endif
#ifndef PIN_DRV2_ADC
  #define PIN_DRV2_ADC    A4
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
static uint16_t g_ptt_overtime_s = 300; // default

// ------------------------- Runtime state -------------------------
static bool     ptt_state = false;
static uint8_t  last_ptt_seq = 0;
static uint8_t  g_status_flags = 0;
static uint32_t g_err_latched = RSCP_ERR_NONE;
static bool     g_err_any_latched = false;

static uint32_t ptt_on_ms = 0;

// ------------------------- Sensors (optional) -------------------------
#if ENABLE_TCS34725
static Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
static bool tcs_ok = false;
#endif

#if ENABLE_DHTxx
  #if DHT_IS_DHT11
    static DHT dht(PIN_DHT, DHT11);
  #else
    static DHT dht(PIN_DHT, DHT22);
  #endif
#endif

#if ENABLE_DS18B20_1
static OneWire ow1(PIN_DS18B20_1);
static DallasTemperature ds1(&ow1);
#endif
#if ENABLE_DS18B20_2
static OneWire ow2(PIN_DS18B20_2);
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
  rscp_u32_to_le(&d[0], g_err_latched);
  canSend(RSCP_ID_ERRORS, d, 8);
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
      sendPttStatus();
      return;
    }

    g_status_flags &= ~PTTSTAT_INHIBITED_BIT;

    ptt_state = true;
    digitalWrite(PIN_PTT_OUT, HIGH);
    ptt_on_ms = millis();

    // Confirm sensing (optional)
    if (pttConfirmActive()) g_status_flags |= PTTSTAT_CONFIRMED_BIT;
    else                    g_status_flags &= ~PTTSTAT_CONFIRMED_BIT;

    sendPttStatus();
  } else {
    ptt_state = false;
    digitalWrite(PIN_PTT_OUT, LOW);
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
  digitalWrite(PIN_PTT_OUT, LOW);

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

#if ENABLE_DHTxx
  dht.begin();
#endif

#if ENABLE_DS18B20_1
  ds1.begin();
#endif
#if ENABLE_DS18B20_2
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
  const uint32_t val = rscp_le_to_u32(&f.data[4]);

  uint8_t ack[8] = {0};
  ack[0] = RSCP_MOD_MAST;
  ack[1] = key;
  ack[2] = 0; // OK

  switch (key) {
    case RSCP_CFG_PTT_OVERTIME_S:
      g_ptt_overtime_s = (uint16_t)min<uint32_t>(val, 3600UL);
      break;

    case RSCP_CFG_STOP_PTT_ERR_MASK:
      g_stop_ptt_err_mask = val;
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
    if (elapsed > (uint32_t)g_ptt_overtime_s * 1000UL) {
      latchError(RSCP_ERR_OVERTIME);
    }

    // Confirm sense drop-out check (optional)
#if HAVE_PTT_CONFIRM
    if (!pttConfirmActive()) {
      latchError(RSCP_ERR_PTT_FAIL);
    } else {
      g_status_flags |= PTTSTAT_CONFIRMED_BIT;
    }
#endif
  }

  // Lightweight periodic telemetry (only if not transmitting, as you requested earlier)
  static uint32_t lastTelemMs = 0;
  if (!ptt_state && (now - lastTelemMs) > 500UL) {
    lastTelemMs = now;

    // Example: send raw voltage ADC
    uint8_t d[8] = {0};
    const uint16_t vraw = readAdcRaw(PIN_VBAT_ADC);
    rscp_put_u16(&d[0], vraw);
    canSend(RSCP_ID_VBAT_RAW, d, 8);

    // Example: send VSWR raw channels (two)
    const uint16_t vswr1 = readAdcRaw(PIN_VSWR1_ADC);
    const uint16_t vswr2 = readAdcRaw(PIN_VSWR2_ADC);
    rscp_put_u16(&d[0], vswr1);
    rscp_put_u16(&d[2], vswr2);
    canSend(RSCP_ID_VSWR_RAW, d, 8);

    // Example: send Drive raw channels (two)
    const uint16_t drv1 = readAdcRaw(PIN_DRV1_ADC);
    const uint16_t drv2 = readAdcRaw(PIN_DRV2_ADC);
    rscp_put_u16(&d[0], drv1);
    rscp_put_u16(&d[2], drv2);
    canSend(RSCP_ID_DRIVE_RAW, d, 8);

#if ENABLE_DHTxx
    // DHT: send temp_x10 and humidity %
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (isnan(t) || isnan(h)) {
      latchError(RSCP_ERR_SENSOR_FAIL);
    } else {
      int16_t tx10 = (int16_t)lroundf(t * 10.0f);
      uint16_t hp = (uint16_t)lroundf(h);
      rscp_put_i16(&d[0], tx10);
      rscp_put_u16(&d[2], hp);
      canSend(RSCP_ID_ENV, d, 8);
    }
#endif

#if ENABLE_DS18B20_1
    ds1.requestTemperatures();
    float t1 = ds1.getTempCByIndex(0);
    if (t1 > -100.0f && t1 < 150.0f) {
      int16_t tx10 = (int16_t)lroundf(t1 * 10.0f);
      rscp_put_i16(&d[0], tx10);
      canSend(RSCP_ID_TEMP1_X10, d, 8);
    } else {
      latchError(RSCP_ERR_SENSOR_FAIL);
    }
#endif

#if ENABLE_DS18B20_2
    ds2.requestTemperatures();
    float t2 = ds2.getTempCByIndex(0);
    if (t2 > -100.0f && t2 < 150.0f) {
      int16_t tx10 = (int16_t)lroundf(t2 * 10.0f);
      rscp_put_i16(&d[0], tx10);
      canSend(RSCP_ID_TEMP2_X10, d, 8);
    } else {
      latchError(RSCP_ERR_SENSOR_FAIL);
    }
#endif

#if ENABLE_TCS34725
    if (tcs_ok) {
      // Read colour sensor as needed; for now just prove it compiles.
      uint16_t r,g,b,c;
      tcs.getRawData(&r,&g,&b,&c);
      // pack something later if you want
    }
#endif
  }
}
