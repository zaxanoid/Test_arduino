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
static uint32_t m2_last_seen_ms = 0;
static uint32_t m3_last_seen_ms = 0;
static uint32_t m4_last_seen_ms = 0;
static bool alive_m2 = false;
static bool alive_m3 = false;
static bool alive_m4 = false;

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
    const uint16_t id = (uint16_t)(canRx.can_id & 0x7FF);

    // Belt-and-braces "alive": mark M2 alive on ANY recognised M2-origin frame.
    auto markM2 = [&]() {
      alive_m2 = true;
      m2_last_seen_ms = millis();
    };
    auto markM3 = [&]() {
      alive_m3 = true;
      m3_last_seen_ms = millis();
    };
    auto markM4 = [&]() {
      alive_m4 = true;
      m4_last_seen_ms = millis();
    };

    switch (id) {

      case RSCP_ID_BOOT:
      case RSCP_ID_HEARTBEAT:
        if (canRx.data[0] == RSCP_NODE_MAST)    markM2();
        if (canRx.data[0] == RSCP_NODE_COMPASS) markM3();
        if (canRx.data[0] == RSCP_NODE_ROTATOR) markM4();
        break;

      case RSCP_ID_STATUS_SUMMARY:
        if (canRx.data[0] == RSCP_NODE_MAST) {
          markM2();
          // optional: canRx.data[1] statusBits
        }
        break;

      case RSCP_ID_ENV_TELEM:
        mast.temp_x10  = (int16_t)rscp_get_u16(&canRx.data[0]);
        mast.humidity  = canRx.data[2];
        markM2();
        break;

      case RSCP_ID_RF_TELEM:
        mast.vbat_raw   = rscp_get_u16(&canRx.data[0]);
        mast.vswr_state = canRx.data[6]; // per header: drive_state then vswr_state in last bytes? (depends). Keep legacy below.
        mast.drive_state= canRx.data[6]; // placeholder, your original used [2],[3]; keep original mapping:
        mast.vswr_state = canRx.data[2];
        mast.drive_state= canRx.data[3];
        markM2();
        break;

      case RSCP_ID_PTT_STATUS: {
        // M2 confirms switching here.
        bool state_on   = (canRx.data[0] == RSCP_PTT_ON);
        uint8_t seq     = canRx.data[1];
        bool confirmed  = (canRx.data[2] & 0x01) != 0;

        markM2();

        if (state_on && confirmed) {
          // Only treat as confirmed ON when mast says confirmed.
          ptt_confirmed = true;
          if (ptt_active && seq == ptt_seq && ptt_latency_ms == 0) {
            ptt_latency_ms = (uint16_t)min((uint32_t)(millis() - ptt_cmd_ms), (uint32_t)65535UL);
          }
        }
        if (!state_on) {
          ptt_confirmed = false;
        }
      } break;

      case RSCP_ID_HEADING:
        compass_deg = rscp_get_u16(&canRx.data[0]);
        markM3();
        break;

      case RSCP_ID_ROT_STATUS:
        rotator_deg = rscp_get_u16(&canRx.data[0]);
        markM4();
        break;

      case RSCP_ID_CFG_ACK:
        if (canRx.data[0] == RSCP_NODE_MAST)    cfgAckM2 = canRx.data[1];
        if (canRx.data[0] == RSCP_NODE_COMPASS) cfgAckM3 = canRx.data[1];
        if (canRx.data[0] == RSCP_NODE_ROTATOR) cfgAckM4 = canRx.data[1];
        break;
    }
  }

  // Timeout tracking
  const uint32_t now = millis();
  const uint32_t timeout_ms = 12000UL;
  if (alive_m2 && (now - m2_last_seen_ms) > timeout_ms) alive_m2 = false;
  if (alive_m3 && (now - m3_last_seen_ms) > timeout_ms) alive_m3 = false;
  if (alive_m4 && (now - m4_last_seen_ms) > timeout_ms) alive_m4 = false;
}


// ======================================================
// ===== UI ==============================================
// ======================================================

void renderLCD() {
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
  if (!ptt_active) {
    lcd.print("OFF ");
  } else if (!ptt_confirmed) {
    lcd.print("REQ ");
  } else {
    lcd.print("ON  ");
  }
  lcd.print("Lat:");
  if (ptt_confirmed) {
    lcd.print(ptt_latency_ms);
    lcd.print("ms ");
  } else {
    lcd.print("---- ");
  }
  lcd.print("M2:");
  if (alive_m2) {
    lcd.print("OK ");
    uint16_t s = (uint16_t)min((uint32_t)((millis()-m2_last_seen_ms)/1000UL), (uint32_t)999UL);
    lcd.print(s);
    lcd.print("s");
  } else {
    lcd.print("LOST");
  }
#endif
}


// ======================================================
// ===== PTT =============================================
// ======================================================

static uint8_t  ptt_seq = 0;
static uint32_t ptt_cmd_ms = 0;
static uint16_t ptt_latency_ms = 0;

// Hardware: external 10k pulldown on PIN_PTT_IN, ACTIVE HIGH when pressed
static inline bool readPttRaw() {
  return (digitalRead(PIN_PTT_IN) == HIGH);
}

static void sendPttCmd(bool on) {
  ptt_seq++;
  canTx.can_id  = RSCP_ID_PTT_CMD;
  canTx.can_dlc = 3;
  canTx.data[0] = on ? 1 : 0;
  canTx.data[1] = ptt_seq;
  canTx.data[2] = 0; // flags (reserved)
  mcp2515.sendMessage(&canTx);

  // Station should NOT show PTT:ON until mast confirms.
  ptt_active = on;        // requested state
  ptt_confirmed = false;  // becomes true on PTT_STATUS confirmed bit
  if (on) {
    ptt_cmd_ms = millis();
    ptt_latency_ms = 0;
  }
}

void handlePtt() {
  // Debounce (fast, low-latency). 10ms default.
  static bool last_raw = false;
  static bool stable = false;
  static uint32_t last_change_ms = 0;

  bool raw = readPttRaw();
  if (raw != last_raw) {
    last_raw = raw;
    last_change_ms = millis();
  }

  const uint32_t debounce_ms = 10;
  if ((millis() - last_change_ms) >= debounce_ms) {
    if (stable != raw) {
      stable = raw;
      sendPttCmd(stable);
    }
  }

  // Local indicators:
  digitalWrite(PIN_PTT_LED, ptt_confirmed ? HIGH : LOW); // LED indicates CONFIRMED TX
  digitalWrite(PIN_PTT_OUT, ptt_active ? HIGH : LOW);    // D4 "PTT enable" follows requested
}

// ======================================================
// ===== SETUP / LOOP ===================================
// ======================================================

void setup() {
  pinMode(PIN_PTT_IN, INPUT);  // external pulldown, active HIGH
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
