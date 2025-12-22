#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h>
#include <rscp_can_protocol.h>

// ======================================================
// ===== DISPLAY SELECTION ===============================
// ======================================================

// Default = 20x4 I2C LCD
// Uncomment to force 16x2
// #define USE_LCD_16x2

#define LCD_I2C_ADDR 0x27

#ifdef USE_LCD_16x2
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);
#else
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);
#endif

// ======================================================
// ===== PIN DEFINITIONS =================================
// ======================================================

static const uint8_t PIN_PTT_IN   = 3;   // external PTT input
static const uint8_t PIN_PTT_OUT  = 4;   // PTT confirm output
static const uint8_t PIN_PTT_LED  = 5;   // PTT status LED
static const uint8_t PIN_BTN      = 2;   // UI button (pullup)

// MCP2515
static const uint8_t CAN_CS_PIN   = 10;

// ======================================================
// ===== SETTINGS STRUCT (FIXED) =========================
// ======================================================

struct Settings {
  uint32_t ptt_max_ms;
  uint16_t compass_period_ms;
  uint16_t rotator_period_ms;
  uint32_t stop_ptt_err_mask;

  // Module 2 thresholds (RAW / placeholder)
  uint16_t vbat_low_raw;
  uint16_t vswr1_high_raw;
  uint16_t vswr2_high_raw;
  int16_t  temp1_high_x10;
  int16_t  temp2_high_x10;
  uint8_t  hum_high_pct;
};

Settings settings;

// ======================================================
// ===== TELEMETRY STATE =================================
// ======================================================

struct MastState {
  uint16_t vbat_raw;
  uint8_t  vswr_state;
  uint8_t  drive_state;
  int16_t  temp_x10;
  uint8_t  humidity;
  uint32_t err_flags;
} mast;

uint16_t compass_deg = 0;
uint16_t rotator_deg = 0;

bool ptt_active = false;
bool ptt_confirmed = false;

// Heartbeats
bool alive_m2 = false;
bool alive_m3 = false;
bool alive_m4 = false;

// CFG ACK status
uint8_t cfgAckM2 = 0xFF;
uint8_t cfgAckM3 = 0xFF;
uint8_t cfgAckM4 = 0xFF;

// ======================================================
// ===== CAN =============================================
// ======================================================

MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canRx;
struct can_frame canTx;

// ======================================================
// ===== HELPERS =========================================
// ======================================================

static const char* vswrText(uint8_t s) {
  switch (s) {
    case 0: return "OK ";
    case 1: return "HIG";
    default:return "ERR";
  }
}

static const char* driveText(uint8_t s) {
  switch (s) {
    case 0: return "OK ";
    case 1: return "LOW";
    default:return "HIG";
  }
}

float vbatVoltsFromRaw(uint16_t raw) {
  // Placeholder – scaling later
  return raw * 0.01f;
}

// ======================================================
// ===== EEPROM ==========================================
// ======================================================

void saveSettings() {
  EEPROM.put(0, settings);
}

void loadSettings() {
  EEPROM.get(0, settings);

  if (settings.ptt_max_ms == 0xFFFFFFFFUL) {
    settings.ptt_max_ms        = 300000;
    settings.compass_period_ms = 500;
    settings.rotator_period_ms = 500;
    settings.stop_ptt_err_mask = 0x000000FF;

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
// ===== CAN TX ==========================================
// ======================================================

void sendCfg(uint8_t key, uint32_t value) {
  canTx.can_id  = RSCP_ID_CFG_SET;
  canTx.can_dlc = 8;
  canTx.data[0] = key;
  canTx.data[1] = 0;
  canTx.data[2] = 0;
  canTx.data[3] = 0;
  rscp_u32_to_le(&canTx.data[4], value);
  mcp2515.sendMessage(&canTx);
}

void pushSettingsToRemotes() {
  sendCfg(RSCP_CFG_PTT_MAX_MS,        settings.ptt_max_ms);
  //sendCfg(RSCP_CFG_STOP_PTT_MASK,     settings.stop_ptt_err_mask);
  sendCfg(RSCP_CFG_STOP_PTT_ERR_MASK, settings.stop_ptt_err_mask);


  sendCfg(RSCP_CFG_COMPASS_PERIOD_MS, settings.compass_period_ms);
  sendCfg(RSCP_CFG_ROTATOR_PERIOD_MS, settings.rotator_period_ms);

  sendCfg(RSCP_CFG_VBAT_LOW_RAW,   settings.vbat_low_raw);
  sendCfg(RSCP_CFG_VSWR1_HIGH_RAW, settings.vswr1_high_raw);
  sendCfg(RSCP_CFG_VSWR2_HIGH_RAW, settings.vswr2_high_raw);
  sendCfg(RSCP_CFG_TEMP1_HIGH_X10, settings.temp1_high_x10);
  sendCfg(RSCP_CFG_TEMP2_HIGH_X10, settings.temp2_high_x10);
  sendCfg(RSCP_CFG_HUM_HIGH_PCT,   settings.hum_high_pct);

  cfgAckM2 = cfgAckM3 = cfgAckM4 = 0xFF;
}

// ======================================================
// ===== CAN RX ==========================================
// ======================================================

void processCan() {
  while (mcp2515.readMessage(&canRx) == MCP2515::ERROR_OK) {
    switch (canRx.can_id) {

      case RSCP_ID_ENV_TELEM:
        mast.temp_x10  = (int16_t)rscp_get_u16(&canRx.data[0]);

        mast.humidity  = canRx.data[2];
        alive_m2 = true;
        break;

      case RSCP_ID_RF_TELEM:
        mast.vbat_raw   = rscp_get_u16(&canRx.data[0]);
        mast.vswr_state = canRx.data[2];
        mast.drive_state= canRx.data[3];
        break;

      case RSCP_ID_HEADING:
        compass_deg = rscp_get_u16(&canRx.data[0]);
        alive_m3 = true;
        break;

      case RSCP_ID_ROT_STATUS:
        rotator_deg = rotator_deg = rscp_get_u16(&canRx.data[0]);
        alive_m4 = true;
        break;

      case RSCP_ID_CFG_ACK:
        if (canRx.data[0] == RSCP_MOD_MAST)    cfgAckM2 = canRx.data[1];
        if (canRx.data[0] == RSCP_MOD_COMPASS) cfgAckM3 = canRx.data[1];
        if (canRx.data[0] == RSCP_MOD_ROTATOR) cfgAckM4 = canRx.data[1];

        break;
    }
  }
}

// ======================================================
// ===== UI ==============================================
// ======================================================

void renderLCD() {
  const uint32_t now = millis();
  const bool rfStale = (now - lastRfMs) > 4000UL;  // no RF telem in 4s
  const bool envStale = (now - lastEnvMs) > 6000UL; // no ENV telem in 6s

  lcd.setCursor(0,0);
  lcd.print("VSWR:");
  lcd.print(vswrText(mast.vswr_state));
  lcd.setCursor(9,0);
  lcd.print("DRV:");
  lcd.print(driveText(mast.drive_state));

  lcd.setCursor(0,1);
  lcd.print("Temp:");
  lcd.print(mast.temp_x10 / 10.0, 1);
  lcd.setCursor(9,1);
  lcd.print("V:");
  lcd.print(vbatVoltsFromRaw(mast.vbat_raw), 1);

#ifndef USE_LCD_16x2
  lcd.setCursor(0,2);
  lcd.print("Bear:");
  lcd.print(compass_deg);
  lcd.print(" Rot:");
  lcd.print(rotator_deg);

  lcd.setCursor(0,3);
  lcd.print("PTT:");
  lcd.print(ptt_active ? "ON " : "OFF");
  lcd.print(" E:");
  lcd.print(mast.err_flags ? "Y " : "N ");
  lcd.print(alive_m2 ? "2" : "-");
  lcd.print(alive_m3 ? "3" : "-");
  lcd.print(alive_m4 ? "4" : "-");
#endif
}

// ======================================================
// ===== PTT =============================================
// ======================================================

void handlePtt() {
  bool in = digitalRead(PIN_PTT_IN) == LOW;

  if (in && !ptt_active) {
    canTx.can_id  = RSCP_ID_PTT_CMD;
    canTx.can_dlc = 1;
    canTx.data[0] = 1;
    mcp2515.sendMessage(&canTx);
    ptt_active = true;
  }

  if (!in && ptt_active) {
    canTx.can_id  = RSCP_ID_PTT_CMD;
    canTx.can_dlc = 1;
    canTx.data[0] = 0;
    mcp2515.sendMessage(&canTx);
    ptt_active = false;
  }

  digitalWrite(PIN_PTT_LED, ptt_active);
  digitalWrite(PIN_PTT_OUT, ptt_confirmed);
}

// ======================================================
// ===== SETUP / LOOP ===================================
// ======================================================

void setup() {
  pinMode(PIN_PTT_IN, INPUT_PULLUP);
  pinMode(PIN_PTT_OUT, OUTPUT);
  pinMode(PIN_PTT_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);

  Wire.begin();
  lcd.init();
  lcd.backlight();

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();

  loadSettings();
  lcd.clear();
}

void loop() {
  processCan();   // priority
  handlePtt();    // priority
  renderLCD();    // one lightweight task
}
