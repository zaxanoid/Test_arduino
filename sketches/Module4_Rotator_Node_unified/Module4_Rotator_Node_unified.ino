// -----------------------------------------------------------------------------
// Module 4: Rotator Controller Node (Arduino Nano/Pico + MCP2515)
// -----------------------------------------------------------------------------
// PURPOSE:
//   - Receive target azimuth from Station over CAN (RSCP_ID_ROT_SET_AZ)
//   - Drive CW/CCW relays (or SSRs) to move rotator
//   - Read position potentiometer via ADC and report azimuth periodically
//   - Accept report period config + query requests
//
// NOTES:
//   - This is a *compliance* implementation for the unified RSCP CAN protocol.
//   - You MUST calibrate the ADC->degrees mapping and verify relay logic on your
//     exact hardware before connecting to a real rotator.
//   - Deterministic loop: CAN RX every iteration, then at most one other task.
//
// CAN frames (from rscp_can_protocol.h):
//   STN->ROT: RSCP_ID_ROT_SET_AZ  (target_deg_x10 uint16)
//            RSCP_ID_ROT_CONFIG  (period_ms uint16)
//            RSCP_ID_ROT_QUERY   (data0=1)
//   ROT->STN: RSCP_ID_ROT_STATUS (az_x10 uint16, target_x10 uint16, state uint8, flags uint8)
//
// -----------------------------------------------------------------------------

#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>
#include <rscp_can_protocol.h>

// ------------------------- Pins (adjust to match your build)
static const uint8_t PIN_CAN_CS = 10;   // MCP2515 CS
static const uint8_t PIN_CW     = 7;    // relay: clockwise (active HIGH)
static const uint8_t PIN_CCW    = 8;    // relay: counter-clockwise (active HIGH)
static const uint8_t PIN_POT    = A0;   // position pot input

// ------------------------- Motion tuning (safe defaults)
static const uint16_t DEAD_BAND_X10      = 10;    // 1.0 degree
static const uint32_t STALL_TIMEOUT_MS   = 8000;  // if position doesn't change for this long, stop + flag
static const uint32_t MIN_MOVE_PULSE_MS  = 120;   // minimum on-time to avoid relay chatter
static const uint32_t STATUS_PERIOD_MS_DEFAULT = 500;

// ADC mapping (TODO: calibrate!)
// These are placeholder endpoints: pot_min->0deg, pot_max->360deg
static const uint16_t POT_MIN = 100;
static const uint16_t POT_MAX = 900;

struct Settings {
  uint32_t magic;
  uint16_t statusPeriodMs;
};
static const uint32_t SETTINGS_MAGIC = 0x52534C34UL; // 'RSL4'

Settings settings;

// ------------------------- CAN
static MCP2515 mcp(PIN_CAN_CS);
static struct can_frame tx;
static struct can_frame rx;

// ------------------------- State
enum MoveState : uint8_t { STOPPED=0, MOVING=1, STALLED=2 };
static volatile MoveState state = STOPPED;

static uint16_t target_x10 = 0xFFFF; // 0xFFFF = no target
static uint16_t az_x10 = 0;

static uint32_t lastStatusMs = 0;
static uint32_t moveStartMs  = 0;
static uint32_t lastAzChangeMs = 0;
static uint16_t lastAz_x10 = 0;

static uint32_t boot_ms = 0;

// ------------------------- Helpers
static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint16_t readAzimuth_x10(){
  uint16_t raw = (uint16_t)analogRead(PIN_POT);
  raw = clamp_u16(raw, POT_MIN, POT_MAX);
  // Linear map
  uint32_t span = (uint32_t)(POT_MAX - POT_MIN);
  uint32_t pos  = (uint32_t)(raw - POT_MIN);
  uint32_t deg_x10 = (pos * 3600UL) / (span ? span : 1UL);
  return (uint16_t)deg_x10; // 0..3600
}

static void relaysOff(){
  digitalWrite(PIN_CW, LOW);
  digitalWrite(PIN_CCW, LOW);
}

static int16_t shortestDelta_x10(uint16_t from_x10, uint16_t to_x10){
  // result in range [-1800..+1800] (wrap around 3600)
  int16_t a = (int16_t)to_x10 - (int16_t)from_x10;
  while (a > 1800) a -= 3600;
  while (a < -1800) a += 3600;
  return a;
}

static void setDirectionFromDelta(int16_t d){
  // d>0 => CW, d<0 => CCW
  if (d > 0){
    digitalWrite(PIN_CCW, LOW);
    digitalWrite(PIN_CW, HIGH);
  } else if (d < 0){
    digitalWrite(PIN_CW, LOW);
    digitalWrite(PIN_CCW, HIGH);
  } else {
    relaysOff();
  }
}

static void sendBoot(){
  tx.can_id  = RSCP_ID_BOOT;
  tx.can_dlc = 8;
  tx.data[0] = RSCP_MOD_ROTATOR;
  tx.data[1] = 0; // reset cause unknown (could be filled later)
  tx.data[2] = RSCP_PROTO_VER_MAJOR;
  tx.data[3] = RSCP_PROTO_VER_MINOR;
  rscp_put_u32(&tx.data[4], 0);
  mcp.sendMessage(&tx);
}
static void sendCfgAck(uint8_t key, uint8_t status){
  tx.can_id  = RSCP_ID_CFG_ACK;
  tx.can_dlc = 4;
  tx.data[0] = RSCP_MOD_ROTATOR;
  tx.data[1] = key;
  tx.data[2] = status; // 0=ok
  tx.data[3] = 0;
  mcp.sendMessage(&tx);
}


static void sendHeartbeat(){
  tx.can_id  = RSCP_ID_HEARTBEAT;
  tx.can_dlc = 8;
  tx.data[0] = RSCP_MOD_ROTATOR;
  tx.data[1] = 0;
  uint32_t up_s = (uint32_t)((millis() - boot_ms)/1000UL);
  rscp_put_u32(&tx.data[2], up_s);
  tx.data[6] = 0;
  tx.data[7] = 0;
  mcp.sendMessage(&tx);
}

static void sendStatus(uint8_t flags){
  tx.can_id  = RSCP_ID_ROT_STATUS;
  tx.can_dlc = 8;
  rscp_put_u16(&tx.data[0], az_x10);
  rscp_put_u16(&tx.data[2], target_x10);
  tx.data[4] = (uint8_t)state;
  tx.data[5] = flags;
  tx.data[6] = 0;
  tx.data[7] = 0;
  mcp.sendMessage(&tx);
}

static void loadSettings(){
  EEPROM.get(0, settings);
  if (settings.magic != SETTINGS_MAGIC){
    settings.magic = SETTINGS_MAGIC;
    settings.statusPeriodMs = STATUS_PERIOD_MS_DEFAULT;
    EEPROM.put(0, settings);
  }
}

static void saveSettings(){
  EEPROM.put(0, settings);
}

// ------------------------- CAN RX
static void processCan(){
  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK){
    switch ((uint16_t)rx.can_id){
      case RSCP_ID_ROT_SET_AZ:
        if (rx.can_dlc >= 2){
          target_x10 = rscp_get_u16(&rx.data[0]);
          // start moving immediately
          moveStartMs = millis();
          lastAzChangeMs = moveStartMs;
          lastAz_x10 = az_x10;
          state = MOVING;
        }
        break;

      case RSCP_ID_ROT_CONFIG:
        if (rx.can_dlc >= 2){
          uint16_t p = rscp_get_u16(&rx.data[0]);
          if (p < 100) p = 100;
          if (p > 5000) p = 5000;
          settings.statusPeriodMs = p;
          saveSettings();
          sendStatus(0x01); // flag: config updated
        }
        break;

      case RSCP_ID_ROT_QUERY:
        if (rx.can_dlc >= 1 && rx.data[0] == 1){
          sendStatus(0x02); // flag: query response
        }
        break;


      case RSCP_ID_CFG_SET:
        if (rx.can_dlc >= 8 && rx.data[0] == RSCP_MOD_ROTATOR) {
          uint8_t key = rx.data[1];
          uint32_t val = rscp_get_u32(&rx.data[4]);
          bool ok = true;
          if (key == RSCP_CFG_ROTATOR_PERIOD_MS) {
            uint16_t p = (uint16_t)val;
            if (p < 100) p = 100;
            if (p > 5000) p = 5000;
            settings.statusPeriodMs = p;
            saveSettings();
            sendCfgAck(key, 0);
            sendStatus(0x01); // config updated
          } else {
            ok = false;
            sendCfgAck(key, 1);
          }
        }
        break;
      case RSCP_ID_EMERG_PTT_OFF:
        // optional: could stop moving during emergency; we just report
        sendStatus(0x04);
        break;

      default:
        break;
    }
  }
}

// ------------------------- Main
void setup(){
  pinMode(PIN_CW, OUTPUT);
  pinMode(PIN_CCW, OUTPUT);
  relaysOff();

  pinMode(PIN_POT, INPUT);

  loadSettings();

  mcp.reset();
  // Match your bus speed. If you are using 500kbps elsewhere, keep this.
  mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp.setNormalMode();

  boot_ms = millis();
  az_x10 = readAzimuth_x10();
  sendBoot();
  sendStatus(0x00);
}

void loop(){
  // 1) Always: CAN RX
  processCan();

  // 2) Always: update azimuth (cheap)
  az_x10 = readAzimuth_x10();

  // 3) One other task per loop: motion control OR periodic status OR heartbeat
  static uint8_t task = 0;
  uint32_t now = millis();

  if (task == 0){
    // Motion control
    if (state == MOVING && target_x10 != 0xFFFF){
      int16_t d = shortestDelta_x10(az_x10, target_x10);

      if (abs(d) <= (int16_t)DEAD_BAND_X10){
        relaysOff();
        state = STOPPED;
        sendStatus(0x10); // reached
      } else {
        // prevent relay chatter: if we just started, keep direction for a minimum time
        if ((now - moveStartMs) < MIN_MOVE_PULSE_MS){
          // keep whatever direction is currently set
        } else {
          setDirectionFromDelta(d);
        }

        // stall detect: if az not changing
        if (az_x10 != lastAz_x10){
          lastAz_x10 = az_x10;
          lastAzChangeMs = now;
        } else if ((now - lastAzChangeMs) > STALL_TIMEOUT_MS){
          relaysOff();
          state = STALLED;
          sendStatus(0x20); // stalled
        }
      }
    } else {
      relaysOff();
    }
  } else if (task == 1){
    // Periodic status
    if ((now - lastStatusMs) >= settings.statusPeriodMs){
      lastStatusMs = now;
      sendStatus(0x00);
    }
  } else {
    // Heartbeat (every 5s)
    static uint32_t lastHb = 0;
    if ((now - lastHb) >= 5000){
      lastHb = now;
      sendHeartbeat();
    }
  }

  task = (uint8_t)((task + 1) % 3);
}