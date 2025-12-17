// -----------------------------------------------------------------------------
// Module 3: Masthead Independent Compass (RP2040 Pi Pico + BNO085 + MCP2515)
// -----------------------------------------------------------------------------
// - BNO085 (SPI) for heading
// - MCP2515 (SPI) for CAN
// - Sends heading periodically (default 500ms), but ONLY when PTT is OFF
// - Optional deadband: if heading hasn't changed by +/-deadband, don't send
// - Supports station config via CAN (period + deadband) and query
// - Provides a digital pin that can be used to de-power / RF-inhibit sensor circuitry
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <mcp2515.h>
#include <EEPROM.h>

#include <rscp_can_protocol.h>

// ------------------------------- Pins (adjust!) -----------------------------
// BNO085 on SPI0 (example Pico wiring)
// NOTE: Pico Arduino pin numbers often match GPIO numbers.
static const uint8_t PIN_BNO_CS    = 5;
static const uint8_t PIN_BNO_INT   = 6;
static const uint8_t PIN_BNO_RST   = 7;

// MCP2515 (can share SPI bus with BNO085; separate CS)
// If you already use MCP2515 on SPI0: keep this.
static const uint8_t PIN_CAN_CS    = 9;

// RF inhibit / sensor power control output (HIGH = enable sensor, LOW = inhibit)
static const uint8_t PIN_RF_ENABLE = 15;

// ------------------------------- Defaults -----------------------------------
static uint16_t g_period_ms = 500;      // send every 0.5s
static uint16_t g_deadband_x100 = 200;  // 2.00 degrees

// ------------------------------- EEPROM layout ------------------------------
static const uint16_t EEPROM_MAGIC = 0xA5C3;
static const int EEPROM_ADDR_MAGIC = 0;
static const int EEPROM_ADDR_PERIOD= 2;
static const int EEPROM_ADDR_DBAND = 4;
static const int EEPROM_SIZE       = 16;

// ------------------------------- Globals ------------------------------------
Adafruit_BNO08x bno08x(PIN_BNO_RST);
sh2_SensorValue_t sensorValue;

MCP2515 mcp2515(PIN_CAN_CS);
struct can_frame rx;
struct can_frame tx;

static uint32_t g_last_send = 0;
static uint16_t g_last_heading_x100 = 0;
static uint8_t  g_ptt_active = 0; // derived from RSCP_ID_PTT_CMD frames

static uint8_t  g_seq = 0;

// ------------------------------- CAN helpers --------------------------------
static void canSend(uint16_t can_id, const uint8_t *data, uint8_t dlc) {
  tx.can_id = can_id;
  tx.can_dlc = dlc;
  for (uint8_t i = 0; i < 8; i++) tx.data[i] = (i < dlc) ? data[i] : 0;
  (void)mcp2515.sendMessage(&tx);
}

static void sendBoot(uint8_t reset_cause) {
  uint8_t d[8] = {0};
  d[0] = RSCP_MOD_COMPASS;
  d[1] = reset_cause;
  d[2] = RSCP_PROTO_VER_MAJOR;
  d[3] = RSCP_PROTO_VER_MINOR;
  canSend(RSCP_ID_BOOT, d, 4);
}

static void sendHeartbeat(void) {
  uint8_t d[8] = {0};
  d[0] = RSCP_MOD_COMPASS;
  d[1] = (g_ptt_active ? 0x01 : 0x00);
  rscp_put_u32(&d[2], (uint32_t)(millis()/1000UL));
  canSend(RSCP_ID_HEARTBEAT, d, 6);
}

static void sendHeading(uint16_t heading_x100, uint8_t flags) {
  uint8_t d[8] = {0};
  rscp_put_u16(&d[0], heading_x100);
  d[2] = flags;
  canSend(RSCP_ID_HEADING, d, 3);
}

static void sendHeadingReply(uint8_t seq) {
  (void)seq;
  // For now reply with normal heading frame (station can treat as response)
  sendHeading(g_last_heading_x100, 0x80);
}

// ------------------------------- EEPROM -------------------------------------
static void loadConfig(void) {
  EEPROM.begin(EEPROM_SIZE);
  uint16_t magic = EEPROM.read(EEPROM_ADDR_MAGIC) | (EEPROM.read(EEPROM_ADDR_MAGIC+1) << 8);
  if (magic == EEPROM_MAGIC) {
    uint16_t p = EEPROM.read(EEPROM_ADDR_PERIOD) | (EEPROM.read(EEPROM_ADDR_PERIOD+1) << 8);
    uint16_t d = EEPROM.read(EEPROM_ADDR_DBAND)  | (EEPROM.read(EEPROM_ADDR_DBAND+1)  << 8);
    if (p >= 100 && p <= 5000) g_period_ms = p;
    if (d <= 3000) g_deadband_x100 = d;
  }
}

static void saveConfig(void) {
  EEPROM.write(EEPROM_ADDR_MAGIC, (uint8_t)(EEPROM_MAGIC & 0xFF));
  EEPROM.write(EEPROM_ADDR_MAGIC+1, (uint8_t)(EEPROM_MAGIC >> 8));
  EEPROM.write(EEPROM_ADDR_PERIOD, (uint8_t)(g_period_ms & 0xFF));
  EEPROM.write(EEPROM_ADDR_PERIOD+1, (uint8_t)(g_period_ms >> 8));
  EEPROM.write(EEPROM_ADDR_DBAND, (uint8_t)(g_deadband_x100 & 0xFF));
  EEPROM.write(EEPROM_ADDR_DBAND+1, (uint8_t)(g_deadband_x100 >> 8));
  EEPROM.commit();
}

// ------------------------------- Heading math --------------------------------
static bool readHeadingDeg(float &heading_deg) {
  // We enable rotation vector; this provides quaternion.
  if (!bno08x.getSensorEvent(&sensorValue)) return false;
  if (sensorValue.sensorId != SH2_ROTATION_VECTOR) return false;

  // Convert quaternion to yaw (heading)
  const float qi = sensorValue.un.rotationVector.i;
  const float qj = sensorValue.un.rotationVector.j;
  const float qk = sensorValue.un.rotationVector.k;
  const float qr = sensorValue.un.rotationVector.real;

  // yaw (Z) from quaternion
  float ys = 2.0f * (qr*qk + qi*qj);
  float yc = 1.0f - 2.0f * (qj*qj + qk*qk);
  float yaw = atan2f(ys, yc);

  float deg = yaw * 180.0f / PI;
  if (deg < 0) deg += 360.0f;
  heading_deg = deg;
  return true;
}

// ------------------------------- CAN RX -------------------------------------
static void sendCfgAck(uint8_t key, uint8_t status) {
  struct can_frame f{};
  f.can_id  = RSCP_ID_CFG_ACK;
  f.can_dlc = 4;
  f.data[0] = RSCP_MOD_COMPASS;
  f.data[1] = key;
  f.data[2] = status; // 0=ok
  f.data[3] = 0;
  mcp2515.sendMessage(&f);
}

// ------------------------------- CAN RX -------------------------------------
static void handleCan(const struct can_frame &f) {
  if (f.can_id == RSCP_ID_PTT_CMD) {
    g_ptt_active = f.data[0] ? 1 : 0;
    digitalWrite(PIN_RF_ENABLE, g_ptt_active ? LOW : HIGH); // inhibit during PTT
    return;
  }

  if (f.can_id == RSCP_ID_HEADING_QUERY) {
    sendHeadingReply(f.data[0]);
    return;
  }

  if (f.can_id == RSCP_ID_HEADING_CONFIG) {
    uint16_t p = rscp_get_u16(&f.data[0]);
    uint16_t d = rscp_get_u16(&f.data[2]);
    if (p >= 100 && p <= 5000) g_period_ms = p;
    if (d <= 3000) g_deadband_x100 = d;
    saveConfig();
    return;
  }

  if (f.can_id == RSCP_ID_CFG_SET) {
    if (f.can_dlc >= 8 && f.data[0] == RSCP_MOD_COMPASS) {
      uint8_t key = f.data[1];
      uint32_t val = rscp_get_u32(&f.data[4]);
      bool ok = true;
      if (key == RSCP_CFG_COMPASS_PERIOD_MS) {
        if (val >= 100 && val <= 60000) g_period_ms = (uint16_t)val;
        else ok = false;
      } else {
        ok = false; // unknown key
      }
      if (ok) saveConfig();
      sendCfgAck(key, ok ? 0 : 1);
    }
    return;
  }

  if (f.can_id == RSCP_ID_EMERG_PTT_OFF) {
    // doesn't directly mean PTT is off, but safe to enable sensor again
    g_ptt_active = 0;
    digitalWrite(PIN_RF_ENABLE, HIGH);
    return;
  }
}

static void canPollRx(void) {(void) {
  for (uint8_t i = 0; i < 3; i++) {
    if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) break;
    handleCan(rx);
  }
}

// ------------------------------- Setup/Loop ---------------------------------
void setup() {
  pinMode(PIN_RF_ENABLE, OUTPUT);
  digitalWrite(PIN_RF_ENABLE, HIGH); // enabled by default

  loadConfig();

  SPI.begin();

  // CAN
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // BNO085
  if (!bno08x.begin_SPI(PIN_BNO_CS, PIN_BNO_INT, &SPI)) {
    // Still send boot so station sees it's alive even if sensor missing
    sendBoot(RSCP_RESET_POWERON);
    while (1) { canPollRx(); delay(10); }
  }
  bno08x.enableReport(SH2_ROTATION_VECTOR, 5000); // sensor internal update (us) - adjust later

  sendBoot(RSCP_RESET_POWERON);
  sendHeartbeat();
  g_last_send = millis();
}

void loop() {
  canPollRx();

  // Heartbeat every 1s
  static uint32_t hb_last = 0;
  if (millis() - hb_last >= 1000) {
    hb_last = millis();
    sendHeartbeat();
  }

  // Only send heading when not in PTT / RF inhibit
  if (g_ptt_active) return;

  if (millis() - g_last_send < g_period_ms) return;

  float heading_deg = 0.0f;
  if (!readHeadingDeg(heading_deg)) return;

  uint16_t heading_x100 = (uint16_t)(heading_deg * 100.0f);

  // deadband check (wrap-aware simplified)
  uint16_t last = g_last_heading_x100;
  uint16_t diff = (heading_x100 > last) ? (heading_x100 - last) : (last - heading_x100);
  if (diff > 18000) diff = 36000 - diff; // wrap
  if (diff < g_deadband_x100) {
    g_last_send = millis();
    return;
  }

  g_last_heading_x100 = heading_x100;
  g_last_send = millis();
  sendHeading(heading_x100, 0);
}
