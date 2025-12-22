// -----------------------------------------------------------------------------
// Module 2: Masthead Protection + PTT
// CANbus + sensors (VBAT, FWD, REV, DHT, optional TCS34725)
// Safety: latched errors, fail-safe PTT drop, priority CAN IDs for PTT/emerg
//
// Pin mapping (from your remote_V5.2 intent / earlier versions):
//   LED=4, DHT=8, PSU_Volts=A1, FWD=A2, REV=A3, MCP2515 CS=10
//
// Notes:
// - ENABLE_TCS34725 default 0 => compiles without Adafruit library installed.
// - If you want the colour sensor, set ENABLE_TCS34725=1 and install Adafruit_TCS34725.
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mcp2515.h>
#include <Wire.h>

#include <rscp_can_protocol.h>

// ------------------------------- Feature flags ------------------------------
#define RSCP_USE_DHT
#define ENABLE_TCS34725 0  // set to 1 if you have Adafruit_TCS34725 library + sensor fitted
// #define RSCP_SERIAL_DEBUG

#ifdef RSCP_USE_DHT
  #include <DHT.h>
#endif

#if ENABLE_TCS34725
  #include <Adafruit_TCS34725.h>
#endif

// ------------------------------- Pinout -------------------------------------
static const uint8_t PIN_CAN_CS      = 10; // MCP2515 CS

static const uint8_t PIN_LED_DEBUG   = 4;  // existing LED
static const uint8_t PIN_PTT_OUT     = 7;  // OUTPUT to opto/relay to key PA/transverter

// Optional confirm input (if you have it wired):
static const uint8_t PIN_PTT_CONFIRM = 2;  // INPUT confirm TX engaged (LOW = confirmed)
static const bool    HAVE_PTT_CONFIRM = false; // set true if wired/used

// Sensors
static const uint8_t PIN_DHT         = 8;  // DHTxx data
static const uint8_t PIN_VBAT        = A1; // PSU_VolTS ADC
static const uint8_t PIN_FWD         = A2; // forward power ADC
static const uint8_t PIN_REV         = A3; // reverse power ADC

// ------------------------------- DHT config ---------------------------------
#ifdef RSCP_USE_DHT
  // Change to DHT11 or DHT22 depending on your sensor
  #define DHTTYPE DHT11
  DHT dht(PIN_DHT, DHTTYPE);
#endif

#if ENABLE_TCS34725
  Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
#endif

// ------------------------------- CAN ----------------------------------------
MCP2515 mcp2515(PIN_CAN_CS);
static struct can_frame rx;
static struct can_frame tx;

// ------------------------------- State --------------------------------------
static uint8_t  g_ptt_cmd     = RSCP_PTT_OFF;
static bool     ptt_active    = false; // true when PTT output asserted

static uint8_t  g_status_flags = 0;
static uint32_t g_err_code     = RSCP_ERR_NONE;

static uint32_t g_last_ptt_rx_ms  = 0;
static uint32_t g_last_env_ms     = 0;
static uint32_t g_last_rf_ms      = 0;

static uint8_t  g_task_rr      = 0;

// ------------------------------- Settings -----------------------------------
struct Settings2 {
  uint32_t ptt_max_ms;
  uint32_t stop_ptt_err_mask;  // which errors are allowed to stop PTT (bitmask)
};

static Settings2 settings2;

// ------------------------------- Helpers ------------------------------------
static inline void canSend(uint16_t can_id, const uint8_t *d, uint8_t len) {
  tx.can_id  = can_id;
  tx.can_dlc = len;
  for (uint8_t i = 0; i < len; i++) tx.data[i] = d[i];
  mcp2515.sendMessage(&tx);
}

static void sendBoot() {
  uint8_t d[8] = {0};
  d[0] = RSCP_PROTO_VER_MAJOR;
  d[1] = RSCP_PROTO_VER_MINOR;
  d[2] = RSCP_MOD_MAST;
  d[3] = 0; // reset reason placeholder
  canSend(RSCP_ID_BOOT, d, 8);
}

static void sendHeartbeat() {
  uint8_t d[8] = {0};
  d[0] = RSCP_MOD_MAST;
  d[1] = (uint8_t)(ptt_active ? 1 : 0);
  d[2] = (uint8_t)(g_err_code != RSCP_ERR_NONE ? 1 : 0);
  canSend(RSCP_ID_HEARTBEAT, d, 8);
}

static bool pttConfirmActive() {
  if (!HAVE_PTT_CONFIRM) return true;
  // LOW means confirmed
  return (digitalRead(PIN_PTT_CONFIRM) == LOW);
}

static void sendPttStatus() {
  uint8_t d[8] = {0};
  d[0] = (uint8_t)(ptt_active ? RSCP_PTT_ON : RSCP_PTT_OFF);
  d[1] = (uint8_t)(g_status_flags);
  d[2] = (uint8_t)(g_err_code != RSCP_ERR_NONE ? 1 : 0);
  d[3] = RSCP_MOD_MAST;
  canSend(RSCP_ID_PTT_STATUS, d, 8);
}

static void latchError(uint32_t err_code) {
  if (g_err_code == RSCP_ERR_NONE) {
    g_err_code = err_code;
  }
}

static void clearErrorManual() {
  g_err_code = RSCP_ERR_NONE;
}

static void applyPtt(uint8_t want_on) {
  if (want_on) {
    // If latched error exists, inhibit
    if (g_err_code != RSCP_ERR_NONE) {
      g_status_flags |= RSCP_PTTSTAT_INHIBITED;
      g_ptt_cmd = RSCP_PTT_OFF;
      ptt_active = false;
      digitalWrite(PIN_PTT_OUT, LOW);
      sendPttStatus();
      return;
    }

    // engage
    g_ptt_cmd = RSCP_PTT_ON;
    ptt_active = true;
    digitalWrite(PIN_PTT_OUT, HIGH);

    // confirm if available
    if (pttConfirmActive()) g_status_flags |= RSCP_PTTSTAT_CONFIRMED;
    else g_status_flags &= ~RSCP_PTTSTAT_CONFIRMED;

    sendPttStatus();
  } else {
    g_ptt_cmd = RSCP_PTT_OFF;
    ptt_active = false;
    digitalWrite(PIN_PTT_OUT, LOW);
    g_status_flags &= ~(RSCP_PTTSTAT_CONFIRMED | RSCP_PTTSTAT_INHIBITED);
    sendPttStatus();
  }
}

// ------------------------------- Telemetry ---------------------------------
static void sendEnvTelem() {
  uint8_t d[8] = {0};

#ifdef RSCP_USE_DHT
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) {
    int16_t tx10 = (int16_t)(t * 10.0f);
    rscp_put_u16(&d[0], (uint16_t)tx10);
  }
  if (!isnan(h)) {
    d[2] = (uint8_t)(h + 0.5f);
  }
#else
  rscp_put_u16(&d[0], 0);
  d[2] = 0;
#endif

  d[3] = 0; // status flags placeholder
  d[7] = RSCP_MOD_MAST;

  canSend(RSCP_ID_ENV_TELEM, d, 8);
  g_last_env_ms = millis();
}

static void sendRfTelem() {
  uint8_t d[8] = {0};

  uint16_t vraw = (uint16_t)analogRead(PIN_VBAT);
  uint16_t fwd  = (uint16_t)analogRead(PIN_FWD);
  uint16_t rev  = (uint16_t)analogRead(PIN_REV);

  rscp_put_u16(&d[0], vraw);
  d[2] = 0; // VSWR state placeholder (0 OK, 1 High, 2 ERR)
  d[3] = 0; // Drive state placeholder (0 OK, 1 Low, 2 High)

  // pack raw fwd/rev if you want later (currently spare)
  (void)fwd; (void)rev;

  d[7] = RSCP_MOD_MAST;

  canSend(RSCP_ID_RF_TELEM, d, 8);
  g_last_rf_ms = millis();
}

// ------------------------------- Settings RX -------------------------------
static void sendCfgAck(uint8_t key, uint8_t status) {
  uint8_t d[8] = {0};
  d[0] = RSCP_MOD_MAST;
  d[1] = status;   // 0 OK, E nonzero for error (your convention)
  d[2] = key;
  canSend(RSCP_ID_CFG_ACK, d, 8);
}

static void loadSettings() {
  EEPROM.get(0, settings2);
  if (settings2.ptt_max_ms == 0xFFFFFFFFUL || settings2.ptt_max_ms == 0) {
    settings2.ptt_max_ms = 300000UL;
    settings2.stop_ptt_err_mask = 0xFFFFFFFFUL;
    EEPROM.put(0, settings2);
  }
}

static void handleCfgSet(const struct can_frame &f) {
  const uint8_t key = f.data[0];
  const uint32_t val = rscp_le_to_u32(&f.data[4]);

  switch (key) {
    case RSCP_CFG_PTT_MAX_MS:
      settings2.ptt_max_ms = val;
      EEPROM.put(0, settings2);
      sendCfgAck(key, 0);
      break;

    case RSCP_CFG_STOP_PTT_ERR_MASK:
      settings2.stop_ptt_err_mask = val;
      EEPROM.put(0, settings2);
      sendCfgAck(key, 0);
      break;

    default:
      // ignore unknown keys but ACK with error
      sendCfgAck(key, 1);
      break;
  }
}

// ------------------------------- CAN RX -------------------------------------
static void handleCan(const struct can_frame &f) {
  if (f.can_id == RSCP_ID_EMERG_PTT_OFF) {
    applyPtt(0);
    return;
  }

  if (f.can_id == RSCP_ID_PTT_CMD) {
    const uint8_t want = f.data[0] ? 1 : 0;
    g_last_ptt_rx_ms = millis();
    applyPtt(want);
    return;
  }

  if (f.can_id == RSCP_ID_ERR_CLEAR) {
    // Manual clear only, no auto-clear
    clearErrorManual();
    sendPttStatus();
    return;
  }

  if (f.can_id == RSCP_ID_CFG_SET) {
    handleCfgSet(f);
    return;
  }
}

// ------------------------------- Tasks --------------------------------------
static void pollColourSensorTick() {
#if ENABLE_TCS34725
  // Only poll when not transmitting (RF)
  if (ptt_active) return;

  static uint32_t last = 0;
  const uint32_t now = millis();
  if ((now - last) < 1000UL) return;
  last = now;

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  // TODO: implement LED colour/status decoding
  (void)r; (void)g; (void)b; (void)c;
#endif
}

static void runOneTask() {
  switch (g_task_rr++ & 0x03) {
    case 0: sendEnvTelem(); break;
    case 1: sendRfTelem();  break;
    case 2: sendHeartbeat(); break;
    case 3: pollColourSensorTick(); break;
  }
}

// ------------------------------- Setup/Loop ---------------------------------
void setup() {
  pinMode(PIN_LED_DEBUG, OUTPUT);
  pinMode(PIN_PTT_OUT, OUTPUT);
  digitalWrite(PIN_PTT_OUT, LOW);

  if (HAVE_PTT_CONFIRM) pinMode(PIN_PTT_CONFIRM, INPUT_PULLUP);

#ifdef RSCP_USE_DHT
  dht.begin();
#endif

#if ENABLE_TCS34725
  // If sensor init fails, just disable polling by leaving ENABLE_TCS34725 at 0 in code
  tcs.begin();
#endif

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  loadSettings();
  sendBoot();
}

void loop() {
  // Priority: drain CAN RX every loop
  while (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    handleCan(rx);
  }

  // Priority: PTT overtime protection (drop PTT if stuck too long)
  if (ptt_active) {
    const uint32_t now = millis();
    if (settings2.ptt_max_ms > 0 && (now - g_last_ptt_rx_ms) > settings2.ptt_max_ms) {
      latchError(RSCP_ERR_PTT_OVERTIME);
      applyPtt(0);
    }
  }

  // Deterministic loop: run at most one extra task per iteration
  runOneTask();
}
