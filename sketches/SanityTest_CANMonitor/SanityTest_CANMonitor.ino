/*
  SanityTest_CANMonitor
  RSCP CAN sniffer + active query tester.

  Serial: 115200
  Board: Arduino Nano/Uno + MCP2515 module
*/

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <rscp_can_protocol.h>

static const uint8_t PIN_CAN_CS = 10;

// Query intervals
static const uint32_t HEADING_QUERY_EVERY_MS = 1000;
static const uint32_t ROT_QUERY_EVERY_MS     = 1000;
static const uint32_t ALIVE_REPORT_EVERY_MS  = 2000;
static const uint32_t ALIVE_TIMEOUT_MS       = 5000;

MCP2515 mcp(PIN_CAN_CS);
struct can_frame rx;
struct can_frame tx;

static uint32_t lastSeenMs[5] = {0,0,0,0,0};

static uint32_t lastHeadingQueryMs = 0;
static uint32_t lastRotQueryMs     = 0;
static uint32_t lastHeadingRttMs   = 0;
static uint32_t lastRotRttMs       = 0;

static uint16_t lastHeadingDegX10  = 0;
static uint16_t lastRotDegX10      = 0;

static void printHex2(uint8_t b){
  const char* h="0123456789ABCDEF";
  Serial.print(h[(b>>4)&0xF]);
  Serial.print(h[b&0xF]);
}

static void printFrame(const can_frame& f){
  Serial.print("ID=0x");
  Serial.print(f.can_id, HEX);
  Serial.print(" DLC=");
  Serial.print(f.can_dlc);
  Serial.print(" DATA=");
  for(uint8_t i=0;i<f.can_dlc;i++){
    printHex2(f.data[i]);
    if(i+1<f.can_dlc) Serial.print(' ');
  }
}

static void decodeRSCP(const can_frame& f){
  switch(f.can_id){
    case RSCP_ID_BOOT: {
      uint8_t node = f.data[0];
      uint8_t reset_cause = f.data[1];
      uint8_t pmaj = f.data[2];
      uint8_t pmin = f.data[3];
      Serial.print("  BOOT node=");
      Serial.print(node);
      Serial.print(" cause=");
      Serial.print(reset_cause);
      Serial.print(" proto=");
      Serial.print(pmaj);
      Serial.print(".");
      Serial.print(pmin);
      break;
    }

    case RSCP_ID_HEARTBEAT: {
      uint8_t node = f.data[0];
      uint8_t flags = f.data[1];
      uint32_t uptime_s = rscp_get_u32(&f.data[2]);
      Serial.print("  HB node=");
      Serial.print(node);
      Serial.print(" flags=0x");
      Serial.print(flags, HEX);
      Serial.print(" up_s=");
      Serial.print(uptime_s);
      break;
    }

    case RSCP_ID_PTT_STATUS: {
      uint8_t state = f.data[0];
      uint8_t seq   = f.data[1];
      uint8_t status_flags = f.data[2];
      uint8_t err_latched  = f.data[3];
      Serial.print("  PTT_STATUS state=");
      Serial.print(state);
      Serial.print(" seq=");
      Serial.print(seq);
      Serial.print(" flags=0x");
      Serial.print(status_flags, HEX);
      Serial.print(" err_latched=");
      Serial.print(err_latched);
      break;
    }

    case RSCP_ID_ERR_STATUS: {
      uint8_t node = f.data[0];
      uint8_t sev  = f.data[1];
      uint32_t flags = rscp_get_u32(&f.data[2]);
      Serial.print("  ERR node=");
      Serial.print(node);
      Serial.print(" sev=");
      Serial.print(sev);
      Serial.print(" bits=0x");
      Serial.print(flags, HEX);
      break;
    }

    case RSCP_ID_ENV_TELEM: {
      int16_t t_x10 = rscp_get_i16(&f.data[0]);
      uint8_t hum = f.data[2];
      int16_t ds18_x10 = rscp_get_i16(&f.data[3]);
      Serial.print("  ENV t=");
      Serial.print(t_x10/10.0, 1);
      Serial.print("C rh=");
      Serial.print(hum);
      Serial.print("% ds18=");
      if(ds18_x10 == 0x7FFF) Serial.print("n/a");
      else Serial.print(ds18_x10/10.0, 1);
      break;
    }

    case RSCP_ID_RF_TELEM: {
      uint16_t vraw = rscp_get_u16(&f.data[0]);
      uint16_t fwd  = rscp_get_u16(&f.data[2]);
      uint16_t rev  = rscp_get_u16(&f.data[4]);
      uint8_t drive_state = f.data[6];
      uint8_t vswr_state  = f.data[7];
      Serial.print("  RF vraw=");
      Serial.print(vraw);
      Serial.print(" fwd=");
      Serial.print(fwd);
      Serial.print(" rev=");
      Serial.print(rev);
      Serial.print(" drv=");
      Serial.print(drive_state);
      Serial.print(" vswr=");
      Serial.print(vswr_state);
      break;
    }

    case RSCP_ID_HEADING: {
      lastHeadingDegX10 = rscp_get_u16(&f.data[0]);
      uint8_t status = f.data[2];
      Serial.print("  HEADING ");
      Serial.print(lastHeadingDegX10/10.0, 1);
      Serial.print("deg status=0x");
      Serial.print(status, HEX);
      if(lastHeadingQueryMs){
        lastHeadingRttMs = millis() - lastHeadingQueryMs;
        Serial.print(" rtt_ms=");
        Serial.print(lastHeadingRttMs);
        lastHeadingQueryMs = 0;
      }
      break;
    }

    case RSCP_ID_ROT_STATUS: {
      lastRotDegX10 = rscp_get_u16(&f.data[0]);
      uint16_t tgt  = rscp_get_u16(&f.data[2]);
      uint8_t state = f.data[4];
      Serial.print("  ROT az=");
      Serial.print(lastRotDegX10/10.0, 1);
      Serial.print(" tgt=");
      Serial.print(tgt/10.0, 1);
      Serial.print(" state=");
      Serial.print(state);
      if(lastRotQueryMs){
        lastRotRttMs = millis() - lastRotQueryMs;
        Serial.print(" rtt_ms=");
        Serial.print(lastRotRttMs);
        lastRotQueryMs = 0;
      }
      break;
    }

    case RSCP_ID_CFG_ACK: {
      uint8_t node = f.data[0];
      uint8_t key = f.data[1];
      uint8_t status = f.data[2];
      Serial.print("  CFG_ACK node=");
      Serial.print(node);
      Serial.print(" key=");
      Serial.print(key);
      Serial.print(" status=");
      Serial.print(status);
      break;
    }

    default:
      break;
  }
}

static void sendHeadingQuery(){
  tx.can_id = RSCP_ID_HEADING_QUERY;
  tx.can_dlc = 1;
  tx.data[0] = 1;
  mcp.sendMessage(&tx);
  lastHeadingQueryMs = millis();
}

static void sendRotQuery(){
  tx.can_id = RSCP_ID_ROT_QUERY;
  tx.can_dlc = 1;
  tx.data[0] = 1;
  mcp.sendMessage(&tx);
  lastRotQueryMs = millis();
}

void setup(){
  Serial.begin(115200);
  while(!Serial){}

  SPI.begin();
  mcp.reset();
  mcp.setBitrate(CAN_500KBPS);
  mcp.setNormalMode();

  Serial.println("RSCP CAN Monitor ready (115200).");
  Serial.println("Actively querying HEADING and ROT once per second.");
}

void loop(){
  // Priority: drain CAN RX quickly
  while(mcp.readMessage(&rx) == MCP2515::ERROR_OK){
    // Update last-seen for frames that include node in data[0]
    if(rx.can_id == RSCP_ID_BOOT || rx.can_id == RSCP_ID_HEARTBEAT || rx.can_id == RSCP_ID_CFG_ACK){
      uint8_t node = rx.data[0];
      if(node <= 4) lastSeenMs[node] = millis();
    }
    if(rx.can_id == RSCP_ID_ERR_STATUS){
      uint8_t node = rx.data[0];
      if(node <= 4) lastSeenMs[node] = millis();
    }

    printFrame(rx);
    decodeRSCP(rx);
    Serial.println();
  }

  uint32_t now = millis();

  if(now - lastHeadingQueryMs > HEADING_QUERY_EVERY_MS && lastHeadingQueryMs == 0){
    sendHeadingQuery();
  }

  if(now - lastRotQueryMs > ROT_QUERY_EVERY_MS && lastRotQueryMs == 0){
    sendRotQuery();
  }

  static uint32_t lastReport = 0;
  if(now - lastReport > ALIVE_REPORT_EVERY_MS){
    lastReport = now;

    Serial.print("Alive: ");
    for(uint8_t n=1;n<=4;n++){
      bool ok = (now - lastSeenMs[n] < ALIVE_TIMEOUT_MS);
      Serial.print("M");
      Serial.print(n);
      Serial.print(ok ? "=Y " : "=N ");
    }
    Serial.print("| Last RTT: compass=");
    Serial.print(lastHeadingRttMs);
    Serial.print("ms rot=");
    Serial.print(lastRotRttMs);
    Serial.println("ms");
  }
}
