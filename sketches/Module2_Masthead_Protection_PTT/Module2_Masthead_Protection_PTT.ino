/*
  Module2_Masthead_Protection_PTT.ino
  Masthead protection + PTT execution node (Module 2)

  Design rules:
   - PTT/confirm/error evaluated every loop iteration
   - At most ONE sensor/telemetry task per loop iteration
   - Fail-safe: loss of CAN or watchdog reset => PTT OFF
   - No auto-clear of latched errors (station must clear)

  Hardware baseline (Arduino Nano + MCP2515):
    D10 CAN_CS, D2 CAN_INT, SPI D11/D12/D13
    D7  PTT_OUT (to opto/relay)
    D6  PTT_CONFIRM_IN (active LOW when PTT actually switched)
    D5  INHIBIT_IN (optional, active HIGH -> inhibit TX)
    D4  PTT_MIRROR_LED (optional)
    D8  DHT sensor (default DHT11; set DHT1_IS_DHT11=0 for DHT22)
    A0  FWD_ADC, A1 REV_ADC, A2 PSU_ADC (legacy name vbat in some docs)
    A4/A5 I2C reserved for colour sensor (optional; not implemented in this core)

  CAN protocol:
    include "rscp_can_protocol.h" (single source of truth)
*/

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "rscp_can_protocol.h"


// ---- Compatibility aliases (older/newer protocol headers)
#ifndef RSCP_NODE_MAST
#define RSCP_NODE_MAST RSCP_MOD_MAST
#endif
#ifndef RSCP_NODE_BROADCAST
// In v1.4 header broadcast module ID is not defined; we use 0 to mean "all modules"
#define RSCP_NODE_BROADCAST 0
#endif
#ifndef RSCP_CFG_DROP_MASK
// Older docs used DROP_MASK; protocol header uses STOP_PTT_ERR_MASK
#define RSCP_CFG_DROP_MASK RSCP_CFG_STOP_PTT_ERR_MASK
#endif
#ifndef RSCP_CFG_PTT_CONFIRM_ENABLE
#define RSCP_CFG_PTT_CONFIRM_ENABLE 16
#endif
// ---- Optional sensors ----
#include <DHT.h>

#define ENABLE_DHT1           1
#define DHT1_IS_DHT11         1   // 1=DHT11, 0=DHT22
#define DHT1_PIN              8

// Optional secondary temperature sensor (disabled by default)
#define ENABLE_TEMP2_DS18B20  0   // DS18B20 on D9
#define ENABLE_TEMP2_DHTxx    0   // DHTxx on D9 (temp only)
#define DHT2_IS_DHT11         1
#define TEMP2_PIN             9

#if ENABLE_TEMP2_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
  static OneWire oneWire2(TEMP2_PIN);
  static DallasTemperature ds2(&oneWire2);
#endif

// ---- Pins (Module 2) ----
static const uint8_t PIN_CAN_CS        = 10;
static const uint8_t PIN_CAN_INT       = 2;

static const uint8_t PIN_PTT_OUT       = 7;
static const uint8_t PIN_PTT_CONFIRM   = 6;
static const uint8_t PIN_PTT_INHIBIT   = 5;   // optional
static const uint8_t PIN_PTT_MIRROR_LED= 4;   // optional

static const uint8_t PIN_FWD_ADC       = A0;
static const uint8_t PIN_REV_ADC       = A1;
static const uint8_t PIN_PSU_ADC       = A2;

// ---- CAN ----
static MCP2515 mcp2515(PIN_CAN_CS);
static struct can_frame rxMsg;

// ---- Timing / loop scheduler ----
static uint32_t now_ms = 0;

static const uint16_t SENSOR_TICK_MS = 50;      // one sensor action every 50ms (adjust as needed)
static uint32_t next_sensor_tick_ms = 0;
static uint8_t sensor_phase = 0;

// Telemetry cadence (adjust via config later if desired)
static const uint16_t ENV_TELEM_MS = 1000;
static const uint16_t RF_TELEM_MS  = 250;

static uint32_t next_env_telem_ms = 0;
static uint32_t next_rf_telem_ms  = 0;

// ---- PTT state ----
static bool     ptt_requested = false;    // desired state from station
static bool     ptt_active_hw = false;    // what we are currently driving
static uint8_t  last_ptt_seq  = 0;
static uint32_t ptt_cmd_rx_ms = 0;
static bool     last_overtime_drop = false;

// overtime protection (set via config later)
static uint32_t max_ptt_ms = 300000UL;
static bool ptt_require_confirm = false; // LOCAL decision: require confirm input to be true when PTT ON
    // 300s default
static uint32_t ptt_on_start_ms = 0;

// ---- Error handling ----
static uint32_t err_bits = 0;
static uint32_t stop_ptt_err_mask = 0;            // which errors should drop PTT
static bool err_latched = false;

// error bit assignments (keep aligned with CAN_Messages.md)
enum {
  ERR_VSWR            = 1u << 0,
  ERR_DRIVE           = 1u << 1,
  ERR_TEMP            = 1u << 2,
  ERR_HUM             = 1u << 3,
  ERR_PSU             = 1u << 4,
  ERR_PTT_CONFIRM_FAIL= 1u << 5,
  ERR_INHIBIT_ACTIVE  = 1u << 6,
};

// ---- Cached telemetry ----
static int16_t  temp1_x10_cache = 0;
static uint8_t  hum_cache = 0;
static uint16_t psu_raw_cache = 0;
static uint16_t fwd_raw_cache = 0;
static uint16_t rev_raw_cache = 0;

static uint8_t  vswr_state_cache  = 0;  // 0 OK, 1 HIGH, 2 ERR
static uint8_t  drive_state_cache = 0;  // 0 OK, 1 LOW, 2 HIGH

// ---- DHT ----
#if ENABLE_DHT1
  static DHT dht1(DHT1_PIN, (DHT1_IS_DHT11 ? DHT11 : DHT22));
#endif
#if ENABLE_TEMP2_DHTxx
  static DHT dht2(TEMP2_PIN, (DHT2_IS_DHT11 ? DHT11 : DHT22));
  static int16_t temp2_x10_cache = 0;
#endif

// ---- Helpers: endian ----
static inline void put_u16(uint8_t *p, uint16_t v){ p[0]=(uint8_t)(v&0xFF); p[1]=(uint8_t)(v>>8); }
static inline uint16_t get_u16(const uint8_t *p){ return (uint16_t)p[0] | ((uint16_t)p[1]<<8); }
static inline void put_i16(uint8_t *p, int16_t v){ put_u16(p, (uint16_t)v); }
static inline int16_t get_i16(const uint8_t *p){ return (int16_t)get_u16(p); }
static inline void put_u32(uint8_t *p, uint32_t v){ p[0]=v&0xFF; p[1]=(v>>8)&0xFF; p[2]=(v>>16)&0xFF; p[3]=(v>>24)&0xFF; }
static inline uint32_t get_u32(const uint8_t *p){ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

// ---- CAN send ----
static void canSend8(uint16_t can_id, const uint8_t d[8]) {
  struct can_frame tx;
  tx.can_id  = can_id;
  tx.can_dlc = 8;
  memcpy(tx.data, d, 8);
  mcp2515.sendMessage(&tx);
}

// ---- Forward declarations (keeps Arduino prototype generator out of the way) ----
static void processCan();
static void sendBoot();
static void sendHeartbeat();
static void sendErrStatus();
static void setPtt(bool on, uint8_t seq, bool forceSendStatus);
static void sendPttStatus(bool result_ok, bool confirmed, bool inhibited) {
  // Protocol v1.4: RSCP_ID_PTT_STATUS:
  // data0=state (1/0), data1=seq, data2=status_flags, data3=err_latched(1/0)
  // NOTE: latency/overtime details are carried via ERR bits (e.g. RSCP_ERR_OVERTIME).
  uint8_t d[8] = {0};
  d[0] = ptt_active_hw ? RSCP_PTT_ON : RSCP_PTT_OFF;
  d[1] = last_ptt_seq;

  uint8_t flags = 0;
  if (confirmed) flags |= 0x01;
  if (inhibited) flags |= 0x02;
  if (last_overtime_drop) flags |= 0x04;
  if (!result_ok) flags |= 0x08; // local "FAIL" indicator (reserved)
  d[2] = flags;

  d[3] = err_latched ? 1 : 0;

  d[4] = d[5] = d[6] = d[7] = 0;
  DBG("PTT_STATUS TX: state="); DBG((int)d[0]);
  DBG(" seq="); DBG((int)d[1]);
  DBG(" flags=0x"); DBGLN(d[2], HEX);
  canSend8(RSCP_ID_PTT_STATUS, d);
}

static bool pttConfirmed() {
  // confirm is active LOW (per your spec)
  return (digitalRead(PIN_PTT_CONFIRM) == LOW);
}

static bool inhibitActive() {
  // INHIBIT input: with INPUT_PULLUP, active when pulled LOW (recommended wiring)
  return (digitalRead(PIN_PTT_INHIBIT) == LOW);
}


static void latchError(uint32_t bits) {
  err_bits |= bits;
  err_latched = (err_bits != 0);
  // On first latch of a new bit, publish immediately
  sendErrStatus();
  // Drop PTT if configured
  if ((stop_ptt_err_mask & bits) != 0) {
    setPtt(false, last_ptt_seq, true);
  }
}

static void clearErrors() {
  err_bits = 0;
  err_latched = false;
  sendErrStatus();
}

static void sendErrStatus() {
  // Protocol v1.4: RSCP_ID_ERR_STATUS: data0=module_id, data1=severity, data2..5=err_bits(uint32)
  uint8_t d[8] = {0};
  d[0] = RSCP_MOD_MAST;

  uint8_t sev = RSCP_SEV_INFO;
  if (err_bits != 0) sev = RSCP_SEV_ERROR;
  d[1] = sev;

  // err_bits is a bitmask (uint32)
  put_u32(&d[2], (uint32_t)err_bits);

  d[6] = 0;
  d[7] = 0;
  canSend8(RSCP_ID_ERR_STATUS, d);
}


static void setPtt(bool on, uint8_t seq, bool forceSendStatus) {
  // apply inhibit
  bool inhibited = inhibitActive();
  if (inhibited) {
    on = false;
    latchError(ERR_INHIBIT_ACTIVE);
  }

  // update seq/state
  last_ptt_seq = seq;
  if (on && !ptt_active_hw) {
    ptt_on_start_ms = now_ms;
    last_overtime_drop = false;
  }

  ptt_active_hw = on;
  digitalWrite(PIN_PTT_OUT, on ? HIGH : LOW);
  if (PIN_PTT_MIRROR_LED != 0xFF) digitalWrite(PIN_PTT_MIRROR_LED, on ? HIGH : LOW);

  // confirmation handling (local policy)
  bool confirmed = (on ? pttConfirmed() : false);
  bool ok = true;

  if (on) {
    if (ptt_require_confirm) {
      ok = confirmed;
      if (!confirmed) {
        // Only treat as an error when confirmation is required
        latchError(ERR_PTT_CONFIRM_FAIL);
      }
    } else {
      // Confirmation disabled: consider PTT OK as soon as output asserted
      confirmed = false;
      ok = true;
    }
  }

if (forceSendStatus) {
    sendPttStatus(ok, confirmed, inhibited);
  }
}

static void sendEnvTelem(int16_t temp_x10, uint8_t humidity_pct, uint16_t temp2_x10_or_na) {
  // Protocol v1.4: RSCP_ID_ENV_TELEM:
  // temp_c_x10(int16 LE), rh_pct(uint8), temp2_c_x10(int16 LE) or 0x7FFF if n/a, remaining reserved
  uint8_t d[8] = {0};
  put_i16(&d[0], temp_x10);
  d[2] = humidity_pct;
  put_i16(&d[3], (int16_t)temp2_x10_or_na);
  d[5] = d[6] = d[7] = 0;
  canSend8(RSCP_ID_ENV_TELEM, d);
}

static void sendRfTelem(uint8_t vswr_state, uint8_t drive_state) {
  // Protocol v1.4: RSCP_ID_RF_TELEM:
  // vbat_raw(uint16), fwd_raw(uint16), rev_raw(uint16), drive_state(uint8), vswr_state(uint8)
  uint8_t d[8] = {0};
  put_u16(&d[0], psu_raw_cache);
  put_u16(&d[2], fwd_raw_cache);
  put_u16(&d[4], rev_raw_cache);
  d[6] = drive_state;
  d[7] = vswr_state;
  canSend8(RSCP_ID_RF_TELEM, d);
}

static void sendBoot() {
  uint8_t d[8] = {0};
  d[0] = RSCP_NODE_MAST;
  d[1] = 1; // boot
  put_u16(&d[2], 0); // reset reason (todo)
  put_u32(&d[4], 0);
  canSend8(RSCP_ID_BOOT, d);
}

static void sendHeartbeat() {
  uint8_t d[8] = {0};
  d[0] = RSCP_NODE_MAST;
  d[1] = ptt_active_hw ? 1 : 0;
  put_u32(&d[2], (uint32_t)now_ms);
  d[6] = err_latched ? 1 : 0;
  static uint8_t seq = 0;
  d[7] = seq++;
  canSend8(RSCP_ID_HEARTBEAT, d);
}

static void processCan() {
  // Non-blocking: poll for messages
  while (mcp2515.readMessage(&rxMsg) == MCP2515::ERROR_OK) {
    uint16_t id = (uint16_t)rxMsg.can_id;
    const uint8_t *d = rxMsg.data;

    if (id == RSCP_ID_PTT_CMD) {
      // data0=on/off (RSCP_PTT_ON/OFF), data1=seq
      const bool on = (d[0] == RSCP_PTT_ON);
      last_ptt_seq = d[1];
      ptt_cmd_rx_ms = now_ms;
      ptt_requested = on;
      DBG("PTT_CMD RX: on="); DBG(on ? 1 : 0);
      DBG(" seq="); DBGLN((int)last_ptt_seq);
      // Apply immediately; force status back
      setPtt(on, last_ptt_seq, true);
    }
    else if (id == RSCP_ID_CFG_SET) {
  // Protocol v1.4: data0=module_id (0=broadcast), data1=key, data2..5=u32 value
  const uint8_t dest = d[0];
  if (dest != RSCP_MOD_MAST && dest != RSCP_NODE_BROADCAST) continue;

  const uint8_t key = d[1];
  const uint32_t val = get_u32(&d[2]);

  bool ok = true;
  switch (key) {
    case RSCP_CFG_PTT_MAX_MS:
      max_ptt_ms = val;
      break;
    case RSCP_CFG_STOP_PTT_ERR_MASK:
      // Stored locally as uint32; your internal err_bits are uint32 too
      stop_ptt_err_mask = val;
      break;
    case RSCP_CFG_PTT_CONFIRM_ENABLE:
      ptt_require_confirm = (val != 0);
      break;
    default:
      ok = false;
      break;
  }

  // ACK: RSCP_ID_CFG_ACK: data0=module_id, data1=key, data2=status(0=ok), rest reserved
  uint8_t a[8] = {0};
  a[0] = RSCP_MOD_MAST;
  a[1] = key;
  a[2] = ok ? 0 : 1;
  a[3] = 0;
  a[4] = a[5] = a[6] = a[7] = 0;
  canSend8(RSCP_ID_CFG_ACK, a);
}
else if (id == RSCP_ID_ERR_CLEAR) {
      // optional: allow station to clear errors
      clearErrors();
    }
  }
}

void setup() {
  // ===== TEMP DEBUG SERIAL (COMMENT OUT LATER) =====
  DBG_BEGIN();
  DBGLN("Module2 boot");
  DBG("ptt_require_confirm="); DBGLN(ptt_require_confirm ? 1 : 0);
  // ================================================

  pinMode(PIN_PTT_OUT, OUTPUT);
  digitalWrite(PIN_PTT_OUT, LOW);

  pinMode(PIN_PTT_CONFIRM, INPUT_PULLUP); // confirm active low
  pinMode(PIN_PTT_INHIBIT, INPUT_PULLUP); // inhibit active high (external pullup ok)
  pinMode(PIN_PTT_MIRROR_LED, OUTPUT);
  digitalWrite(PIN_PTT_MIRROR_LED, LOW);

  pinMode(PIN_FWD_ADC, INPUT);
  pinMode(PIN_REV_ADC, INPUT);
  pinMode(PIN_PSU_ADC, INPUT);

#if ENABLE_DHT1
  dht1.begin();
#endif
#if ENABLE_TEMP2_DS18B20
  ds2.begin();
#endif
#if ENABLE_TEMP2_DHTxx
  dht2.begin();
#endif

  SPI.begin();
  mcp2515.reset();
  MCP2515::ERROR e1 = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  MCP2515::ERROR e2 = mcp2515.setNormalMode();
  DBG("CAN setBitrate="); DBGLN((int)e1);
  DBG("CAN setNormalMode="); DBGLN((int)e2);
now_ms = millis();
  next_sensor_tick_ms = now_ms + SENSOR_TICK_MS;
  next_env_telem_ms = now_ms + 200;
  next_rf_telem_ms  = now_ms + 200;

  // default: drop PTT on critical errors
  stop_ptt_err_mask = ERR_VSWR | ERR_DRIVE | ERR_PTT_CONFIRM_FAIL | ERR_PSU;

  sendBoot();
  sendHeartbeat();
  sendErrStatus();
}

void loop() {
  now_ms = millis();
  static uint32_t dbg_last = 0;
  if (now_ms - dbg_last >= 1000) { dbg_last = now_ms; DBG("."); }
// 1) Always process CAN first
  processCan();

  // 2) PTT confirm + overtime every loop
  if (ptt_active_hw) {
    // confirm must remain valid (only if local policy requires it)
    if (ptt_require_confirm) {
      if (!pttConfirmed()) {
        latchError(ERR_PTT_CONFIRM_FAIL);
        setPtt(false, last_ptt_seq, true);
      }
    }

// overtime enforcement
    if (!last_overtime_drop && (now_ms - ptt_on_start_ms) > max_ptt_ms) {
      last_overtime_drop = true;
      setPtt(false, last_ptt_seq, true);
      // also publish error bit via ERR_STATUS if you want a dedicated bit
    }
  }

  // 3) Do at most ONE sensor-related action per tick
  if ((int32_t)(now_ms - next_sensor_tick_ms) >= 0) {
    next_sensor_tick_ms = now_ms + SENSOR_TICK_MS;

    switch (sensor_phase++ & 0x03) {
      case 0: { // temp/humidity (DHT1)
#if ENABLE_DHT1
        float t = dht1.readTemperature();
        if (!isnan(t) && t > -50.0f && t < 150.0f) {
          temp1_x10_cache = (int16_t)lroundf(t * 10.0f);
        } else {
          latchError(ERR_TEMP);
        }
        if (!ptt_active_hw) {
          float h = dht1.readHumidity();
          if (!isnan(h) && h >= 0.0f && h <= 100.0f) {
            hum_cache = (uint8_t)lroundf(h);
          } else {
            latchError(ERR_HUM);
          }
        }
#endif
      } break;

      case 1: { // PSU voltage raw
        psu_raw_cache = (uint16_t)analogRead(PIN_PSU_ADC);
        // optional: threshold check later once scaling is known
      } break;

      case 2: { // VSWR states (only when PTT active)
        if (ptt_active_hw) {
          fwd_raw_cache = (uint16_t)analogRead(PIN_FWD_ADC);
          rev_raw_cache = (uint16_t)analogRead(PIN_REV_ADC);
          uint16_t fwd = fwd_raw_cache;
          uint16_t rev = rev_raw_cache;
          // placeholder: state logic (real VSWR calc later)
          if (fwd < 5) {
            vswr_state_cache = 2; // ERR - no forward power
          } else if (rev > (fwd / 2)) {
            vswr_state_cache = 1; // HIGH
            latchError(ERR_VSWR);
          } else {
            vswr_state_cache = 0;
          }
        }
      } break;

      default: { // Drive state placeholder (only when PTT active)
        if (ptt_active_hw) {
          // placeholder: until colour sensor integrated
          drive_state_cache = 0;
        }
      } break;
    }
  }

  // 4) Telemetry (non-blocking; only one send per loop via gating)
  if ((int32_t)(now_ms - next_env_telem_ms) >= 0) {
    next_env_telem_ms = now_ms + ENV_TELEM_MS;
    sendEnvTelem(temp1_x10_cache, hum_cache, 0x7FFF); // temp2 n/a by default
  } else if ((int32_t)(now_ms - next_rf_telem_ms) >= 0) {
    next_rf_telem_ms = now_ms + RF_TELEM_MS;
    sendRfTelem(vswr_state_cache, drive_state_cache);
  }

  // 5) Heartbeat (slow)
  static uint32_t next_hb = 0;
  if ((int32_t)(now_ms - next_hb) >= 0) {
    next_hb = now_ms + 5000UL;
    sendHeartbeat();
  }
}