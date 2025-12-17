// -----------------------------------------------------------------------------
// Module 2: Masthead Protection + PTT (Arduino Nano/Teensy + MCP2515)
// -----------------------------------------------------------------------------
// Uses pin mappings based on remote_V5.2.ino (uploaded):
//   LED=4, DHTPIN=8, PSU_Volts=A1, FWD=A2, REV=A3, MCP2515 CS=10
//
// Adds optional DS18B20 (spare pin default D7).
//
// Safety model:
//   - Errors are latched until cleared by station (NO auto-clear)
//   - Which errors stop PTT is controlled by stopPttErrMask (EEPROM + station config)
//   - Any "stop" error immediately drops PTT and notifies station with RSCP_ID_ERR_STATUS
//   - If PTT sense/confirm is enabled and indicates failure -> latch RSCP_ERR_PTT_FAIL
//
// Deterministic loop:
//   - CAN RX + PTT logic every loop
//   - at most one "sensor task" per loop iteration
// -----------------------------------------------------------------------------

#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

#if ((USE_DS18B20_1 || USE_DS18B20_2)_1 || (USE_DS18B20_1 || USE_DS18B20_2)_2)
static OneWire ow1(PIN_DS18_1);
static DallasTemperature ds18_1(&ow1);
#if (USE_DS18B20_1 || USE_DS18B20_2)_2
static OneWire ow2(PIN_DS18_2);
static DallasTemperature ds18_2(&ow2);
#endif
#endif

#include <DHT.h>

// -------- Sensor selection (compile-time)
// Choose at most one of USE_DHT11 / USE_DHT22 (or none)
#define USE_DHT11 1
#define USE_DHT22 0
// DS18B20 sensors (0/1)
#define (USE_DS18B20_1 || USE_DS18B20_2)_1 1
#define (USE_DS18B20_1 || USE_DS18B20_2)_2 0

#if (USE_DS18B20_1 || USE_DS18B20_2)_1
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

#include <rscp_can_protocol.h>

// ------------------------- Pin mappings (from remote_V5.2.ino)
static const uint8_t PIN_LED         = 4;
static const uint8_t PIN_DHT         = 8;
static const uint8_t PIN_DS18_1      = 7;  // spare pin default
static const uint8_t PIN_DS18_2      = 6;  // spare pin default (optional)


static const uint8_t PIN_PTT_OUT     = 3;  // opto/relay to transverter/linear PTT
static const uint8_t PIN_PTT_SENSE   = 2;  // sense line to confirm PTT actually switched (optional)
static const bool    PTT_SENSE_ENABLE = true;
static const bool    PTT_SENSE_ACTIVE_LOW = true;

static const uint8_t PIN_CAN_CS      = 10;

// Analog
static const uint8_t PIN_VBAT        = A1; // PSU_Volts
static const uint8_t PIN_FWD         = A2;
static const uint8_t PIN_REV         = A3;
// Optional 2nd RF chain (if your Nano has A6/A7 or you use a Pico/Teensy)
#define ENABLE_RF2 0
#if ENABLE_RF2
static const uint8_t PIN_FWD2        = A6;
static const uint8_t PIN_REV2        = A7;
#endif


// DS18B20
#if (USE_DS18B20_1 || USE_DS18B20_2)_1
static const uint8_t PIN_DS18        = 7;  // spare pin
OneWire oneWire(PIN_DS18);
DallasTemperature ds18(&oneWire);
#endif

// DHT (optional)
#if USE_DHT11
static const uint8_t DHTTYPE = DHT11;
#define USE_DHT 1
#elif USE_DHT22
static const uint8_t DHTTYPE = DHT22;
#define USE_DHT 1
#else
#define USE_DHT 0
#endif
#if USE_DHT
DHT dht(PIN_DHT, DHTTYPE);
#endif
// ------------------------- CAN
MCP2515 mcp(PIN_CAN_CS);
struct can_frame rx;

// ------------------------- Settings in EEPROM
struct MastSettings {
  uint32_t magic;
  uint32_t stopPttErrMask; // RSCP_ERR_* bitmask
  uint32_t pttMaxMs;
  // Threshold placeholders (calibrate later)
  uint32_t vbat_low_raw;
  uint32_t vswr1_high_raw;
  uint32_t vswr2_high_raw;
  uint32_t temp1_high_x10;
  uint32_t temp2_high_x10;
  uint32_t hum_high_pct;
};
static const uint32_t MAST_MAGIC = 0x4D415354; // "MAST"
MastSettings cfg;

// ------------------------- Runtime state
static bool ptt_requested = false;
static uint8_t last_ptt_seq = 0;
static uint32_t ptt_on_ms = 0;

static uint32_t err_bits = 0; // latched
static bool err_changed = false;

// Telemetry
static int16_t temp_c_x10 = 0;
static uint8_t rh_pct = 0;

#if (USE_DS18B20_1 || USE_DS18B20_2)_1
static int16_t ds18_c_x10 = 0x7FFF;
#endif

static uint16_t vbat_raw = 0, fwd_raw = 0, rev_raw = 0;
static uint8_t drive_state = RSCP_DRIVE_OK;
static uint8_t vswr_state  = RSCP_VSWR_ERR;

// Timers
static uint32_t last_env_ms = 0;
static uint32_t last_rf_ms  = 0;
static uint32_t last_hb_ms  = 0;

// ------------------------- Helpers
static void loadCfg() {
  EEPROM.get(0, cfg);
  if (cfg.magic != MAST_MAGIC) {
    cfg.magic = MAST_MAGIC;
    cfg.stopPttErrMask = (RSCP_ERR_PTT_FAIL | RSCP_ERR_VSWR_HIGH | RSCP_ERR_OVERTIME);
    cfg.pttMaxMs = 300000UL;
    EEPROM.put(0, cfg);
  }
}

static void latchErr(uint32_t bit) {
  if ((err_bits & bit) == 0) {
    err_bits |= bit;
    err_changed = true;
  }
}

static void clearErrMask(uint32_t mask) {
  if (mask == 0xFFFFFFFFUL) {
    if (err_bits != 0) err_changed = true;
    err_bits = 0;
  } else {
    uint32_t before = err_bits;
    err_bits &= ~mask;
    if (err_bits != before) err_changed = true;
  }
}

static void sendHeartbeat() {
  struct can_frame f{};
  f.can_id = RSCP_ID_HEARTBEAT;
  f.can_dlc = 6;
  f.data[0] = RSCP_MOD_MAST;
  f.data[1] = 0;
  uint32_t up_s = millis() / 1000UL;
  rscp_u32_to_le(&f.data[2], up_s);
  mcp.sendMessage(&f);
}

static void sendBoot() {
  struct can_frame f{};
  f.can_id = RSCP_ID_BOOT;
  f.can_dlc = 4;
  f.data[0] = RSCP_MOD_MAST;
  f.data[1] = 0; // reset cause placeholder
  f.data[2] = RSCP_PROTO_VER_MAJOR;
  f.data[3] = RSCP_PROTO_VER_MINOR;
  mcp.sendMessage(&f);
}

static void sendPttStatus(bool confirmed) {
  struct can_frame f{};
  f.can_id = RSCP_ID_PTT_STATUS;
  f.can_dlc = 4;
  f.data[0] = ptt_requested ? RSCP_PTT_ON : RSCP_PTT_OFF;
  f.data[1] = last_ptt_seq;
  f.data[2] = confirmed ? 0x01 : 0x00; // status_flags bit0=confirmed
  f.data[3] = (err_bits != 0) ? 1 : 0;  // err_latched flag
  mcp.sendMessage(&f);
}

static void sendErrStatus(uint8_t severity) {
  struct can_frame f{};
  f.can_id = RSCP_ID_ERR_STATUS;
  f.can_dlc = 6;
  f.data[0] = RSCP_MOD_MAST;
  f.data[1] = severity;
  rscp_u32_to_le(&f.data[2], err_bits);
  mcp.sendMessage(&f);
}

static void sendEnvTelem() {
  struct can_frame f{};
  f.can_id = RSCP_ID_ENV_TELEM;
  f.can_dlc = 5;
  f.data[0] = (uint8_t)(temp_c_x10 & 0xFF);
  f.data[1] = (uint8_t)((temp_c_x10 >> 8) & 0xFF);
  f.data[2] = rh_pct;
#if (USE_DS18B20_1 || USE_DS18B20_2)_1
  f.data[3] = (uint8_t)(ds18_c_x10 & 0xFF);
  f.data[4] = (uint8_t)((ds18_c_x10 >> 8) & 0xFF);
#else
  f.data[3] = 0xFF; f.data[4] = 0x7F; // 0x7FFF
#endif
  mcp.sendMessage(&f);
}

static void sendRfTelem() {
  struct can_frame f{};
  f.can_id = RSCP_ID_RF_TELEM;
  f.can_dlc = 8;
  f.data[0] = (uint8_t)(vbat_raw & 0xFF);
  f.data[1] = (uint8_t)((vbat_raw >> 8) & 0xFF);
  f.data[2] = (uint8_t)(fwd_raw & 0xFF);
  f.data[3] = (uint8_t)((fwd_raw >> 8) & 0xFF);
  f.data[4] = (uint8_t)(rev_raw & 0xFF);
  f.data[5] = (uint8_t)((rev_raw >> 8) & 0xFF);
  f.data[6] = drive_state;
  f.data[7] = vswr_state;
  mcp.sendMessage(&f);
}

#if ENABLE_RF2
static void sendRf2Telem() {
  struct can_frame f{};
  f.can_id = RSCP_ID_RF2_TELEM;
  f.can_dlc = 6;
  f.data[0] = (uint8_t)(fwd2_raw & 0xFF);
  f.data[1] = (uint8_t)((fwd2_raw >> 8) & 0xFF);
  f.data[2] = (uint8_t)(rev2_raw & 0xFF);
  f.data[3] = (uint8_t)((rev2_raw >> 8) & 0xFF);
  f.data[4] = drive2_state;
  f.data[5] = vswr2_state;
  mcp.sendMessage(&f);
}
#endif


// Placeholder VSWR state calc for now (scaling/calibration later)
static uint8_t calcVswrState(uint16_t fwd, uint16_t rev) {
  // If ADCs look dead, mark ERR
  if (fwd == 0 && rev == 0) return RSCP_VSWR_ERR;
  // Very crude heuristic until calibrated:
  if (rev > (fwd / 2)) return RSCP_VSWR_HIGH;
  return RSCP_VSWR_OK;
}

// Placeholder drive state calc for now (can later be based on FWD power etc.)
static uint8_t calcDriveState(uint16_t fwd) {
  if (fwd < 50) return RSCP_DRIVE_LOW;
  if (fwd > 800) return RSCP_DRIVE_HIGH;
  return RSCP_DRIVE_OK;
}

static bool pttConfirmedNow() {
  if (!PTT_SENSE_ENABLE) return true;
  bool s = digitalRead(PIN_PTT_SENSE);
  return PTT_SENSE_ACTIVE_LOW ? (s == LOW) : (s == HIGH);
}

static void applyPttOut(bool on) {
  digitalWrite(PIN_PTT_OUT, on ? HIGH : LOW);
}

// ------------------------- CAN RX
static void processCan() {
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    switch (rx.can_id) {
      case RSCP_ID_EMERG_PTT_OFF:
        ptt_requested = false;
        applyPttOut(false);
        sendPttStatus(false);
        break;

      case RSCP_ID_PTT_CMD:
        if (rx.can_dlc >= 2) {
          bool on = rx.data[0] != 0;
          last_ptt_seq = rx.data[1];
          ptt_requested = on;
          if (on) ptt_on_ms = millis();
          applyPttOut(on);
          // Confirm check happens in main loop each iteration
        }
        break;

      case RSCP_ID_ERR_CLEAR:
        if (rx.can_dlc >= 6) {
          uint8_t target = rx.data[0];
          if (target == RSCP_MOD_MAST || target == 0) {
            uint32_t mask = rscp_le_to_u32(&rx.data[2]);
            clearErrMask(mask);
            // after clear, report status (manual clear only)
            sendErrStatus((err_bits == 0) ? RSCP_SEV_INFO : RSCP_SEV_ERROR);
          }
        }
        break;

      case RSCP_ID_CFG_SET:
        if (rx.can_dlc >= 8 && rx.data[0] == RSCP_MOD_MAST) {
          uint8_t key = rx.data[1];
          uint32_t val = rscp_le_to_u32(&rx.data[4]);
          if (key == RSCP_CFG_PTT_MAX_MS) cfg.pttMaxMs = val;
          else if (key == RSCP_CFG_STOP_PTT_ERR_MASK) cfg.stopPttErrMask = val;
          else if (key == RSCP_CFG_VBAT_LOW_RAW) cfg.vbat_low_raw = val;
          else if (key == RSCP_CFG_VSWR1_HIGH_RAW) cfg.vswr1_high_raw = val;
          else if (key == RSCP_CFG_VSWR2_HIGH_RAW) cfg.vswr2_high_raw = val;
          else if (key == RSCP_CFG_TEMP1_HIGH_X10) cfg.temp1_high_x10 = val;
          else if (key == RSCP_CFG_TEMP2_HIGH_X10) cfg.temp2_high_x10 = val;
          else if (key == RSCP_CFG_HUM_HIGH_PCT) cfg.hum_high_pct = val;
          EEPROM.put(0, cfg);

          // ACK
          struct can_frame f{};
          f.can_id = RSCP_ID_CFG_ACK;
          f.can_dlc = 4;
          f.data[0] = RSCP_MOD_MAST;
          f.data[1] = key;
          f.data[2] = 0; // ok
          f.data[3] = 0;
          mcp.sendMessage(&f);
        }
        break;

      default:
        break;
    }
  }
}

// ------------------------- Setup / Loop
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_PTT_OUT, OUTPUT);
  applyPttOut(false);

  pinMode(PIN_PTT_SENSE, INPUT_PULLUP);

#if (USE_DS18B20_1 || USE_DS18B20_2)_1
  ds18_1.begin();
#endif
  #if USE_DHT
  dht.begin();
#endif
#if ((USE_DS18B20_1 || USE_DS18B20_2)_1 || (USE_DS18B20_1 || USE_DS18B20_2)_2)
  ds18_1.setWaitForConversion(true);
  ds18_1.begin();
#if (USE_DS18B20_1 || USE_DS18B20_2)_2
  ds18_2.setWaitForConversion(true);
  ds18_2.begin();
#endif
#endif


  loadCfg();

  mcp.reset();
  mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp.setNormalMode();

  delay(20);
  sendBoot();
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  // 1) Always: CAN RX
  processCan();

  // 2) Always: PTT safety evaluation
  bool confirmed = pttConfirmedNow();

  // Overtime protection
  if (ptt_requested && cfg.pttMaxMs > 0 && (millis() - ptt_on_ms > cfg.pttMaxMs)) {
    latchErr(RSCP_ERR_OVERTIME);
    ptt_requested = false;
    applyPttOut(false);
  }

  // Sense/confirm failure
  if (ptt_requested && PTT_SENSE_ENABLE && !confirmed) {
    latchErr(RSCP_ERR_PTT_FAIL);
    // fail-safe drop
    ptt_requested = false;
    applyPttOut(false);
  }

  // If any configured STOP errors are set, drop PTT
  if (ptt_requested && (err_bits & cfg.stopPttErrMask)) {
    ptt_requested = false;
    applyPttOut(false);
  }

  // Push immediate error status if changed
  if (err_changed) {
    err_changed = false;
    sendErrStatus((err_bits & cfg.stopPttErrMask) ? RSCP_SEV_FATAL : RSCP_SEV_ERROR);
  }

  // Always report PTT status (cheap) on state changes only
  static bool prev_ptt = false;
  if (ptt_requested != prev_ptt) {
    prev_ptt = ptt_requested;
    sendPttStatus(confirmed && ptt_requested);
  }

  // 3) One non-PTT task per loop iteration
  static uint8_t rr = 0;
  rr = (rr + 1) % 5;

  uint32_t now = millis();

  if (rr == 0) {
    // Heartbeat every 1s
    if (now - last_hb_ms >= 1000) { last_hb_ms = now; sendHeartbeat(); }
  } else if (rr == 1) {
    // Env sensors at 2s
    if (now - last_env_ms >= 2000) {
      last_env_ms = now;
#if USE_DHT
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if (isnan(t) || isnan(h)) {
        latchErr(RSCP_ERR_SENSOR_FAIL);
      } else {
        temp_c_x10 = (int16_t)(t * 10.0f);
        rh_pct = (uint8_t)(h + 0.5f);
      }
#else
      temp_c_x10 = 0x7FFF;
      rh_pct = 0;
#endif
#if (USE_DS18B20_1 || USE_DS18B20_2)_1
      ds18_1.requestTemperatures();
      {
        float tds = ds18_1.getTempCByIndex(0);
        if (tds > -100 && tds < 150) ds18_c_x10 = (int16_t)(tds * 10.0f);
        else ds18_c_x10 = 0x7FFF;
      }
#else
      ds18_c_x10 = 0x7FFF;
#endif
      sendEnvTelem();
    }
  }
  } else if (rr == 2) {
    // reserved (was DS18 staggered)
  } else if (rr == 3) {
    // RF telem at 500ms
    if (now - last_rf_ms >= 500) {
      last_rf_ms = now;
      vbat_raw = analogRead(PIN_VBAT);
      fwd_raw  = analogRead(PIN_FWD);
      rev_raw  = analogRead(PIN_REV);

      vswr_state  = calcVswrState(fwd_raw, rev_raw);
      drive_state = calcDriveState(fwd_raw);

      // Map VSWR state into error bits if desired
      if (vswr_state == RSCP_VSWR_HIGH) latchErr(RSCP_ERR_VSWR_HIGH);

      sendRfTelem();
    }
  } else {
    // LED blink / spare
    static uint32_t lastBlink = 0;
    if (now - lastBlink > 2000) { lastBlink = now; digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }
  }
}
