#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h>
#include "rscp_can_protocol.h"

// ======================================================
// Module1: Station Controller (CAN bus master-ish UI)
//  - PTT-critical state machine
//  - Central CAN router
//  - Per-node heartbeat tracking
//  - LCD page system with auto-rotate (default ON, 2s)
//  - Config-over-CAN (push to remotes) + ACK tracking
// ======================================================

// ======================================================
// ===== DISPLAY SELECTION ===============================
// ======================================================
// Default = 20x4 I2C LCD (recommended for pages)
// Uncomment to force 16x2
// #define USE_LCD_16x2

#define LCD_I2C_ADDR 0x27

#include <LiquidCrystal_I2C.h>
#ifdef USE_LCD_16x2
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);
  static const uint8_t LCD_COLS = 16;
  static const uint8_t LCD_ROWS = 2;
#else
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);
  static const uint8_t LCD_COLS = 20;
  static const uint8_t LCD_ROWS = 4;
#endif


// --- forward declare to defeat Arduino auto-prototypes ---
struct NodeStatus;
static void markAlive(NodeStatus &n, uint8_t fwMaj, uint8_t fwMin, uint8_t statusBits);


// ======================================================
// ===== PIN DEFINITIONS =================================
// ======================================================
static const uint8_t PIN_PTT_IN   = 3;   // external PTT input (ACTIVE LOW, pullup)
static const uint8_t PIN_PTT_OUT  = 4;   // local "PTT confirmed" output (optional)
static const uint8_t PIN_PTT_LED  = 5;   // PTT status LED
static const uint8_t PIN_BTN      = 2;   // UI button (pullup) short=next page, long=toggle autorotate

// MCP2515
static const uint8_t CAN_CS_PIN   = 10;

// ======================================================
// ===== SETTINGS (EEPROM) ===============================
// ======================================================
struct Settings {
  uint32_t magic;

  uint32_t ptt_max_ms;            // sent to mast
  uint32_t stop_ptt_err_mask;     // sent to mast

  uint16_t compass_period_ms;     // future
  uint16_t rotator_period_ms;     // future

  // UI
  uint8_t  lcd_autorotate;        // 0/1
  uint16_t lcd_rotate_ms;         // default 2000

  // Module2 thresholds (placeholders)
  uint16_t vbat_low_raw;
  uint16_t vswr1_high_raw;
  uint16_t vswr2_high_raw;
  int16_t  temp1_high_x10;
  int16_t  temp2_high_x10;
  uint8_t  hum_high_pct;
};

static const uint32_t SETTINGS_MAGIC = 0x53544E31; // "STN1"
Settings settings;

static void saveSettings() { EEPROM.put(0, settings); }

static void loadSettings() {
  EEPROM.get(0, settings);
  if (settings.magic != SETTINGS_MAGIC) {
    settings.magic            = SETTINGS_MAGIC;
    settings.ptt_max_ms       = 300000UL;
    settings.stop_ptt_err_mask= (RSCP_ERR_PTT_FAIL | RSCP_ERR_VSWR_HIGH | RSCP_ERR_OVERTIME);

    settings.compass_period_ms = 500;
    settings.rotator_period_ms = 500;

    settings.lcd_autorotate    = 1;
    settings.lcd_rotate_ms     = 2000;

    settings.vbat_low_raw     = 0;
    settings.vswr1_high_raw   = 0;
    settings.vswr2_high_raw   = 0;
    settings.temp1_high_x10   = 0;
    settings.temp2_high_x10   = 0;
    settings.hum_high_pct     = 0;

    saveSettings();
  }
}

// ======================================================
// ===== STATE: PER NODE =================================
// ======================================================
struct NodeStatus {
  bool     alive;
  uint32_t lastSeenMs;
  uint8_t  fwMajor;
  uint8_t  fwMinor;
  uint8_t  statusBits;
};

static NodeStatus nodeMast{}, nodeCompass{}, nodeRot{};

static void markAlive(NodeStatus &n, uint8_t fwMaj, uint8_t fwMin, uint8_t statusBits) {
  n.alive = true;
  n.lastSeenMs = millis();
  n.fwMajor = fwMaj;
  n.fwMinor = fwMin;
  n.statusBits = statusBits;
}

static void updateAliveTimeouts() {
  const uint32_t now = millis();
  const uint32_t timeoutMs = 12000UL; // 12s without heartbeat => not alive
  if (nodeMast.alive   && (now - nodeMast.lastSeenMs   > timeoutMs)) nodeMast.alive = false;
  if (nodeCompass.alive&& (now - nodeCompass.lastSeenMs> timeoutMs)) nodeCompass.alive = false;
  if (nodeRot.alive    && (now - nodeRot.lastSeenMs    > timeoutMs)) nodeRot.alive = false;
}

// ======================================================
// ===== TELEMETRY STATE =================================
// ======================================================
struct MastRf {
  uint16_t vbat_raw = 0;
  uint16_t fwd_raw  = 0;
  uint16_t rev_raw  = 0;
  uint8_t  drive_state = RSCP_DRIVE_OK;
  uint8_t  vswr_state  = RSCP_VSWR_ERR;
} mastRf;

struct MastEnv {
  uint8_t  tempCount = 0;
  int16_t  t1_x10 = (int16_t)0x7FFF;
  int16_t  t2_x10 = (int16_t)0x7FFF;
  uint8_t  s1_id = 0;
  uint8_t  s2_id = 0;

  uint16_t rh_x10 = 0xFFFF;
  int16_t  dht_t_x10 = (int16_t)0x7FFF;
  uint8_t  humSensorType = 0;
} mastEnv;

struct Faults {
  uint32_t err_bits = 0;
  uint8_t  severity = RSCP_SEV_INFO;
  uint8_t  faultCode = 0;
  uint8_t  faultDetail = 0;
} faults;

// Compass/rotator values (separate from any rotator heading logic)
static uint16_t compass_bearing_x10 = 0;
static uint8_t  compass_quality = 0;

// ======================================================
// ===== PTT STATE MACHINE ===============================
// ======================================================
enum StationState : uint8_t {
  ST_IDLE = 0,
  ST_PTT_ACTIVE,
  ST_REMOTE_FAULT,
  ST_CAN_LOSS,
  ST_OVERTIME
};

static StationState stationState = ST_IDLE;

static bool     pttActive = false;     // local PTT input state (debounced)
static uint8_t  pttSeq = 0;
static uint32_t pttStartMs = 0;
static uint32_t pttAckMs = 0;
static uint16_t lastPttLatencyMs = 0;
static bool     remoteOvertimeDrop = false;

// Debounce
static bool rawPtt = false;
static bool debouncedPtt = false;
static uint32_t pttDebounceStart = 0;
static const uint16_t PTT_DEBOUNCE_MS = 15;

// ======================================================
// ===== CAN =============================================
// ======================================================
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canRx;
struct can_frame canTx;

// ======================================================
// ===== UI / LCD ========================================
// ======================================================
static uint8_t  lcdPage = 0;
static const uint8_t LCD_PAGE_COUNT =
#ifdef USE_LCD_16x2
  3;   // fewer pages on 16x2
#else
  5;
#endif

static uint32_t lastLcdUpdateMs = 0;
static uint32_t lastPageRotateMs = 0;

static bool btnDown = false;
static uint32_t btnDownMs = 0;

static void lcdPrintFixed(uint8_t col, uint8_t row, const char* s) {
  lcd.setCursor(col, row);
  lcd.print(s);
  // optional padding left to caller
}

// ======================================================
// ===== Utility formatting ==============================
// ======================================================
static const char* vswrText(uint8_t s) {
  switch (s) {
    case RSCP_VSWR_OK:   return "OK ";
    case RSCP_VSWR_HIGH: return "HIG";
    default:             return "ERR";
  }
}
static const char* driveText(uint8_t s) {
  switch (s) {
    case RSCP_DRIVE_OK:   return "OK ";
    case RSCP_DRIVE_LOW:  return "LOW";
    default:              return "HIG";
  }
}
static float vbatVoltsFromRaw(uint16_t raw) {
  // Placeholder scaling. Calibrate later.
  return raw * 0.01f;
}

static float tFromX10(int16_t t10) {
  if (t10 == (int16_t)0x7FFF) return NAN;
  return (float)t10 / 10.0f;
}

// ======================================================
// ===== CAN TX helpers ==================================
// ======================================================
static void sendCfg(uint8_t targetNode, uint8_t key, uint32_t value, uint8_t seq) {
  canTx.can_id  = RSCP_ID_CFG_SET;
  canTx.can_dlc = 8;
  canTx.data[0] = targetNode;
  canTx.data[1] = key;
  rscp_put_u32(&canTx.data[2], value);
  canTx.data[6] = 0;   // flags
  canTx.data[7] = seq;
  mcp2515.sendMessage(&canTx);
}

static void pushSettingsToMast() {
  uint8_t seq = (uint8_t)(millis() & 0xFF);

  sendCfg(RSCP_NODE_MAST, RSCP_CFG_PTT_MAX_MS,        settings.ptt_max_ms, seq);
  sendCfg(RSCP_NODE_MAST, RSCP_CFG_STOP_PTT_ERR_MASK, settings.stop_ptt_err_mask, seq);

  sendCfg(RSCP_NODE_MAST, RSCP_CFG_VBAT_LOW_RAW,   settings.vbat_low_raw, seq);
  sendCfg(RSCP_NODE_MAST, RSCP_CFG_VSWR1_HIGH_RAW, settings.vswr1_high_raw, seq);
  sendCfg(RSCP_NODE_MAST, RSCP_CFG_VSWR2_HIGH_RAW, settings.vswr2_high_raw, seq);
  sendCfg(RSCP_NODE_MAST, RSCP_CFG_TEMP1_HIGH_X10, (uint32_t)(int32_t)settings.temp1_high_x10, seq);
  sendCfg(RSCP_NODE_MAST, RSCP_CFG_TEMP2_HIGH_X10, (uint32_t)(int32_t)settings.temp2_high_x10, seq);
  sendCfg(RSCP_NODE_MAST, RSCP_CFG_HUM_HIGH_PCT,   settings.hum_high_pct, seq);
}

static void sendPttCmd(bool on) {
  pttSeq++;

  canTx.can_id  = RSCP_ID_PTT_CMD;
  canTx.can_dlc = 8;
  canTx.data[0] = on ? 1 : 0;
  canTx.data[1] = pttSeq;
  canTx.data[2] = 0; // flags (future: sensor suppress etc)
  canTx.data[3] = RSCP_NODE_STATION;
  rscp_put_u32(&canTx.data[4], pttStartMs);
  mcp2515.sendMessage(&canTx);
}

static void sendEmergPttOff() {
  canTx.can_id  = RSCP_ID_EMERG_PTT_OFF;
  canTx.can_dlc = 2;
  canTx.data[0] = RSCP_NODE_STATION;
  canTx.data[1] = ++pttSeq;
  mcp2515.sendMessage(&canTx);
}

// ======================================================
// ===== CAN RX router ===================================
// ======================================================
static void handleHeartbeat(const struct can_frame &f) {
  const uint8_t nodeId = f.data[0];
  const uint8_t statusBits = f.data[1];
  const uint8_t fwMaj = f.data[6];
  const uint8_t fwMin = f.data[7];

  if (nodeId == RSCP_NODE_MAST)    markAlive(nodeMast, fwMaj, fwMin, statusBits);
  if (nodeId == RSCP_NODE_COMPASS) markAlive(nodeCompass, fwMaj, fwMin, statusBits);
  if (nodeId == RSCP_NODE_ROTATOR) markAlive(nodeRot, fwMaj, fwMin, statusBits);
}

static void handleRfTelem(const struct can_frame &f) {
  mastRf.vbat_raw = rscp_get_u16(&f.data[0]);
  mastRf.fwd_raw  = rscp_get_u16(&f.data[2]);
  mastRf.rev_raw  = rscp_get_u16(&f.data[4]);
  mastRf.drive_state = f.data[6];
  mastRf.vswr_state  = f.data[7];
}

static void handleTempReport(const struct can_frame &f) {
  if (f.data[0] != RSCP_NODE_MAST) return;
  mastEnv.tempCount = f.data[1];
  mastEnv.s1_id = f.data[2];
  mastEnv.s2_id = f.data[3];
  mastEnv.t1_x10 = rscp_get_i16(&f.data[4]);
  mastEnv.t2_x10 = rscp_get_i16(&f.data[6]);
}

static void handleHumReport(const struct can_frame &f) {
  if (f.data[0] != RSCP_NODE_MAST) return;
  mastEnv.humSensorType = f.data[1];
  mastEnv.rh_x10 = rscp_get_u16(&f.data[2]);
  mastEnv.dht_t_x10 = rscp_get_i16(&f.data[4]);
}

static void handlePttAck(const struct can_frame &f) {
  const uint8_t state = f.data[0];
  const uint8_t seq   = f.data[1];
  const uint8_t result= f.data[2];
  const uint8_t overtimeDrop = f.data[3];
  const uint16_t latMs = rscp_get_u16(&f.data[4]);

  if (seq != pttSeq) return; // ignore stale
  pttAckMs = millis();
  lastPttLatencyMs = latMs;
  remoteOvertimeDrop = overtimeDrop != 0;

  // If remote reported failure, we treat as fault and force local PTT OFF
  if (result != 0) {
    stationState = ST_REMOTE_FAULT;
    if (pttActive) {
      sendEmergPttOff();
      pttActive = false;
    }
  }

  // Optional: confirm output
  digitalWrite(PIN_PTT_OUT, (state == RSCP_PTT_ON) ? HIGH : LOW);
}

static void handleErrStatus(const struct can_frame &f) {
  faults.severity = f.data[1];
  faults.err_bits = rscp_get_u32(&f.data[2]);
  faults.faultCode = f.data[6];
  faults.faultDetail = f.data[7];

  if (faults.severity >= RSCP_SEV_ERROR) stationState = ST_REMOTE_FAULT;
}

static void handleCompassBearing(const struct can_frame &f) {
  if (f.data[0] != RSCP_NODE_COMPASS) return;
  compass_quality = f.data[1];
  compass_bearing_x10 = rscp_get_u16(&f.data[2]);
}

static void processCan() {
  while (mcp2515.readMessage(&canRx) == MCP2515::ERROR_OK) {
    const uint16_t id = (uint16_t)(canRx.can_id & 0x7FF);

    switch (id) {
      case RSCP_ID_HEARTBEAT:       handleHeartbeat(canRx); break;
      case RSCP_ID_RF_TELEM:        handleRfTelem(canRx); break;
      case RSCP_ID_TEMP_REPORT:     handleTempReport(canRx); break;
      case RSCP_ID_HUM_REPORT:      handleHumReport(canRx); break;
      case RSCP_ID_PTT_ACK:         handlePttAck(canRx); break;
      case RSCP_ID_ERR_STATUS:      handleErrStatus(canRx); break;
      case RSCP_ID_COMPASS_BEARING: handleCompassBearing(canRx); break;
      case RSCP_ID_CFG_ACK:
        // currently not displayed; hook later if needed
        break;
      default:
        break;
    }
  }
}

// ======================================================
// ===== PTT input handling ==============================
// ======================================================
static bool readPttRaw() { return (digitalRead(PIN_PTT_IN) == LOW); }

static void updatePttDebounce() {
  const bool r = readPttRaw();
  const uint32_t now = millis();

  if (r != rawPtt) {
    rawPtt = r;
    pttDebounceStart = now;
  }

  if ((now - pttDebounceStart) >= PTT_DEBOUNCE_MS) {
    debouncedPtt = rawPtt;
  }
}

static void handlePttStateMachine() {
  updatePttDebounce();

  const uint32_t now = millis();

  // CAN loss fail-safe: if mast not alive, force PTT OFF
  if (!nodeMast.alive && pttActive) {
    stationState = ST_CAN_LOSS;
    sendEmergPttOff();
    pttActive = false;
  }

  // Transition: IDLE -> PTT_ACTIVE
  if (debouncedPtt && !pttActive) {
    pttActive = true;
    pttStartMs = now;
    stationState = ST_PTT_ACTIVE;
    sendPttCmd(true);
  }

  // Transition: PTT_ACTIVE -> IDLE
  if (!debouncedPtt && pttActive) {
    pttActive = false;
    stationState = ST_IDLE;
    sendPttCmd(false);
  }

  // Station-side overtime protection redundancy
  if (pttActive && settings.ptt_max_ms > 0 && (now - pttStartMs > settings.ptt_max_ms)) {
    stationState = ST_OVERTIME;
    sendEmergPttOff();
    pttActive = false;
  }

  digitalWrite(PIN_PTT_LED, pttActive ? HIGH : LOW);
}

// ======================================================
// ===== Button / LCD control ============================
// ======================================================
static bool btnPressed() { return (digitalRead(PIN_BTN) == LOW); }

static void handleButton() {
  const uint32_t now = millis();
  const bool down = btnPressed();

  if (down && !btnDown) {
    btnDown = true;
    btnDownMs = now;
  } else if (!down && btnDown) {
    // released
    const uint32_t held = now - btnDownMs;
    btnDown = false;

    if (held >= 700) {
      // long press: toggle auto-rotate
      settings.lcd_autorotate = settings.lcd_autorotate ? 0 : 1;
      saveSettings();
    } else {
      // short press: next page
      lcdPage = (uint8_t)((lcdPage + 1) % LCD_PAGE_COUNT);
      lastPageRotateMs = now;
    }
  }
}

static void maybeRotatePage() {
  const uint32_t now = millis();
  if (!settings.lcd_autorotate) return;

  // Freeze pages during PTT (always show page 0)
  if (pttActive) { lcdPage = 0; return; }

  if ((uint32_t)(now - lastPageRotateMs) >= (uint32_t)settings.lcd_rotate_ms) {
    lastPageRotateMs = now;
    lcdPage = (uint8_t)((lcdPage + 1) % LCD_PAGE_COUNT);
  }
}

static void renderPage0() {
  // Summary + PTT
  lcd.setCursor(0,0);
  lcd.print("PTT:");
  lcd.print(pttActive ? "ON " : "OFF");
  lcd.print(" Lat:");
  lcd.print(lastPttLatencyMs);
  lcd.print("ms ");
#ifndef USE_LCD_16x2
  lcd.setCursor(0,1);
  lcd.print("M2:");
  lcd.print(nodeMast.alive ? "OK " : "LOST");
  lcd.print(" Err:");
  lcd.print((faults.err_bits != 0) ? "Y " : "N ");
  lcd.print("Auto:");
  lcd.print(settings.lcd_autorotate ? "1" : "0");

  lcd.setCursor(0,2);
  lcd.print("VSWR:");
  lcd.print(vswrText(mastRf.vswr_state));
  lcd.print(" DRV:");
  lcd.print(driveText(mastRf.drive_state));

  lcd.setCursor(0,3);
  lcd.print("VBat:");
  lcd.print(vbatVoltsFromRaw(mastRf.vbat_raw), 1);
  lcd.print("V ");
  lcd.print(remoteOvertimeDrop ? "OT!" : "   ");
#else
  lcd.setCursor(0,1);
  lcd.print("M2:");
  lcd.print(nodeMast.alive ? "OK " : "LOST");
  lcd.print(" Err:");
  lcd.print((faults.err_bits != 0) ? "Y" : "N");
#endif
}

static void renderPage1() {
  // RF page
  lcd.setCursor(0,0);
  lcd.print("RF  VSWR:");
  lcd.print(vswrText(mastRf.vswr_state));
  lcd.print(" DRV:");
  lcd.print(driveText(mastRf.drive_state));
#ifndef USE_LCD_16x2
  lcd.setCursor(0,1);
  lcd.print("VBat:");
  lcd.print(vbatVoltsFromRaw(mastRf.vbat_raw), 2);
  lcd.print("V  F:");
  lcd.print(mastRf.fwd_raw);

  lcd.setCursor(0,2);
  lcd.print("REV:");
  lcd.print(mastRf.rev_raw);
  lcd.print("  Node:");
  lcd.print(nodeMast.fwMajor);
  lcd.print(".");
  lcd.print(nodeMast.fwMinor);

  lcd.setCursor(0,3);
  lcd.print("State:");
  lcd.print((int)stationState);
  lcd.print(" Seq:");
  lcd.print((int)pttSeq);
#else
  lcd.setCursor(0,1);
  lcd.print("VBat:");
  lcd.print(vbatVoltsFromRaw(mastRf.vbat_raw), 1);
  lcd.print("V");
#endif
}

static void renderPage2() {
  // Temps/Humidity
  lcd.setCursor(0,0);
  lcd.print("TEMP ");
  if (mastEnv.tempCount >= 1) {
    float t1 = tFromX10(mastEnv.t1_x10);
    if (isnan(t1)) lcd.print("--.-");
    else lcd.print(t1,1);
    lcd.print("C");
  } else {
    lcd.print("n/a ");
  }

#ifndef USE_LCD_16x2
  if (mastEnv.tempCount >= 2) {
    lcd.print(" T2:");
    float t2 = tFromX10(mastEnv.t2_x10);
    if (isnan(t2)) lcd.print("--.-");
    else lcd.print(t2,1);
    lcd.print("C");
  }

  lcd.setCursor(0,1);
  lcd.print("RH:");
  if (mastEnv.rh_x10 == 0xFFFF) lcd.print("--.-");
  else lcd.print((float)mastEnv.rh_x10/10.0f,1);
  lcd.print("%  DHT:");
  float td = tFromX10(mastEnv.dht_t_x10);
  if (isnan(td)) lcd.print("--.-");
  else lcd.print(td,1);
  lcd.print("C");

  lcd.setCursor(0,2);
  lcd.print("S1:");
  lcd.print(mastEnv.s1_id);
  lcd.print(" S2:");
  lcd.print(mastEnv.s2_id);
  lcd.print(" Type:");
  lcd.print(mastEnv.humSensorType);

  lcd.setCursor(0,3);
  lcd.print("Auto:");
  lcd.print(settings.lcd_autorotate ? "1" : "0");
  lcd.print(" ");
  lcd.print(settings.lcd_rotate_ms);
  lcd.print("ms");
#else
  lcd.setCursor(0,1);
  lcd.print("RH:");
  if (mastEnv.rh_x10 == 0xFFFF) lcd.print("--.-");
  else lcd.print((float)mastEnv.rh_x10/10.0f,1);
  lcd.print("%");
#endif
}

static void renderPage3() {
  // Bearings page (compass confidence)
  lcd.setCursor(0,0);
  lcd.print("Compass:");
  lcd.print((float)compass_bearing_x10/10.0f, 1);
  lcd.print((char)223);
  lcd.print(" Q:");
  lcd.print((int)compass_quality);
#ifndef USE_LCD_16x2
  lcd.setCursor(0,1);
  lcd.print("C Alive:");
  lcd.print(nodeCompass.alive ? "Y " : "N ");
  lcd.print("FW:");
  lcd.print(nodeCompass.fwMajor);
  lcd.print(".");
  lcd.print(nodeCompass.fwMinor);

  lcd.setCursor(0,2);
  lcd.print("R Alive:");
  lcd.print(nodeRot.alive ? "Y " : "N ");
  lcd.print("FW:");
  lcd.print(nodeRot.fwMajor);
  lcd.print(".");
  lcd.print(nodeRot.fwMinor);

  lcd.setCursor(0,3);
  lcd.print("PTT:");
  lcd.print(pttActive ? "ON " : "OFF");
  lcd.print(" Lat:");
  lcd.print(lastPttLatencyMs);
  lcd.print("ms");
#else
  lcd.setCursor(0,1);
  lcd.print("Alive:");
  lcd.print(nodeCompass.alive ? "Y" : "N");
#endif
}

static void renderPage4() {
  // Faults / link
  lcd.setCursor(0,0);
  lcd.print("FAULT Sev:");
  lcd.print((int)faults.severity);
  lcd.print(" Bits:");
  lcd.print((faults.err_bits != 0) ? "Y" : "N");
#ifndef USE_LCD_16x2
  lcd.setCursor(0,1);
  lcd.print("Err:");
  lcd.print((uint16_t)(faults.err_bits & 0xFFFF), HEX);
  lcd.print(" ");
  lcd.print((uint16_t)((faults.err_bits >> 16) & 0xFFFF), HEX);

  lcd.setCursor(0,2);
  lcd.print("M2:");
  lcd.print(nodeMast.alive ? "OK " : "LOST");
  lcd.print(" Last:");
  lcd.print((uint16_t)((millis() - nodeMast.lastSeenMs) / 1000UL));
  lcd.print("s");

  lcd.setCursor(0,3);
  lcd.print("Auto:");
  lcd.print(settings.lcd_autorotate ? "1" : "0");
  lcd.print(" Btn:Pg/Long");
#else
  lcd.setCursor(0,1);
  lcd.print("M2:");
  lcd.print(nodeMast.alive ? "OK " : "LOST");
#endif
}

static void renderLCD() {
  // Rate limit + avoid flicker
  const uint32_t now = millis();
  if (now - lastLcdUpdateMs < 250) return;
  lastLcdUpdateMs = now;

  lcd.clear();

  // Always force page0 during PTT
  if (pttActive) lcdPage = 0;

  switch (lcdPage) {
    case 0: renderPage0(); break;
    case 1: renderPage1(); break;
    case 2: renderPage2(); break;
    case 3: renderPage3(); break;
#ifndef USE_LCD_16x2
    case 4: renderPage4(); break;
#endif
    default: renderPage0(); break;
  }
}

// ======================================================
// ===== SETUP / LOOP ====================================
// ======================================================
void setup() {
  pinMode(PIN_PTT_IN, INPUT_PULLUP);
  pinMode(PIN_PTT_OUT, OUTPUT);
  pinMode(PIN_PTT_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  loadSettings();
  pushSettingsToMast();

  lastPageRotateMs = millis();
}

void loop() {
  // Priority tasks first
  processCan();
  updateAliveTimeouts();
  handlePttStateMachine();

  // UI
  handleButton();
  maybeRotatePage();
  renderLCD();
}
