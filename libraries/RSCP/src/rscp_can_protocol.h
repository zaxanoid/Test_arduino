#pragma once
/*
  RSCP - Remote Station Controller CAN Protocol (11-bit IDs; MCP2515 friendly)

  Design goals:
  - Deterministic PTT path: low CAN IDs win arbitration
  - Simple fixed-size payloads (mostly 8 bytes)
  - Explicit Node IDs so Module1 can track multiple remotes
  - Versioned + extensible

  Node IDs (default):
    0x01 Station (Module1)
    0x02 Masthead (Module2)
    0x03 Compass (separate node)
    0x04 Rotator node

  Endianness:
    - Little-endian for all multi-byte values

  Units:
    - Temperatures: degC x10 (int16)
    - Bearing/Azimuth: degrees x10 (uint16)
    - Humidity: %RH x10 (uint16) where used
    - Milliseconds: uint16/uint32 as stated
*/

#include <Arduino.h>

// ------------------------- Protocol version
#define RSCP_PROTO_VER_MAJOR 1
#define RSCP_PROTO_VER_MINOR 5

// ------------------------- Node IDs
enum : uint8_t {
  RSCP_NODE_STATION = 0x01,
  RSCP_NODE_MAST    = 0x02,
  RSCP_NODE_COMPASS = 0x03,
  RSCP_NODE_ROTATOR = 0x04
};

// ------------------------- CAN IDs (11-bit)
// Lower ID = higher priority on bus.
enum : uint16_t {
  // ---- PTT critical path
  RSCP_ID_EMERG_PTT_OFF   = 0x001, // STN->ALL : data0=srcNode, data1=seq
  RSCP_ID_PTT_CMD         = 0x010, // STN->MAST: data0=cmd(0/1), data1=seq, data2=flags, data3=srcNode, data4..7=pttStartMs32 (optional)
  RSCP_ID_PTT_ACK         = 0x011, // MAST->STN: data0=state(0/1), data1=seq, data2=result(0=OK,1=FAIL), data3=overtimeDrop(0/1), data4..5=latMs16, data6=flags, data7=srcNode

  // ---- Errors / faults
  RSCP_ID_ERR_STATUS      = 0x012, // MODx->STN: data0=nodeId, data1=severity, data2..5=err_bits(uint32), data6=faultCode, data7=faultDetail
  RSCP_ID_ERR_CLEAR       = 0x013, // STN->MODx: data0=targetNode(0=MAST), data1=scope(0=all), data2..5=mask(uint32), data6=seq, data7=srcNode

  // ---- Boot / heartbeat / discovery
  RSCP_ID_BOOT            = 0x020, // MODx->STN: data0=nodeId, data1=resetCause, data2=protoMajor, data3=protoMinor, data4..7=fwBuild32 (optional)
  RSCP_ID_HEARTBEAT       = 0x021, // MODx->STN: data0=nodeId, data1=statusBits, data2..5=uptime_s(uint32), data6=fwMajor, data7=fwMinor

  // ---- Mast telemetry
  RSCP_ID_RF_TELEM        = 0x031, // MAST->STN: vbat_raw(u16), fwd_raw(u16), rev_raw(u16), drive_state(u8), vswr_state(u8)
  RSCP_ID_TEMP_REPORT     = 0x032, // MAST->STN: nodeId, count(1/2), sensorId1, sensorId2, temp1_x10(i16), temp2_x10(i16)
  RSCP_ID_HUM_REPORT      = 0x033, // MAST->STN: nodeId, sensorType(11/12/22), RH_x10(u16), T_x10(i16), status(u8), seq(u8)

  // ---- Compass
  RSCP_ID_COMPASS_BEARING = 0x060, // COMP->STN: nodeId, quality(u8), bearing_x10(u16), reserved(2), flags(u8), seq(u8)
  RSCP_ID_COMPASS_QUERY   = 0x061, // STN->COMP: data0=1(query_now), data1=seq, data2=srcNode
  RSCP_ID_COMPASS_CONFIG  = 0x062, // STN->COMP: period_ms(u16), delta_x10(u16), hb_s(u8), flags(u8), seq(u8), srcNode(u8)

  // ---- Rotator
  RSCP_ID_ROT_SET_AZ      = 0x070, // STN->ROT: target_deg_x10(u16), seq(u8), srcNode(u8)
  RSCP_ID_ROT_STATUS      = 0x071, // ROT->STN: az_deg_x10(u16), target_deg_x10(u16), state(u8), flags(u8), nodeId(u8), seq(u8)
  RSCP_ID_ROT_CONFIG      = 0x072, // STN->ROT: period_ms(u16), flags(u8), seq(u8), srcNode(u8)
  RSCP_ID_ROT_QUERY       = 0x073, // STN->ROT: data0=1(query_now), data1=seq, data2=srcNode

  // ---- Generic config (persist on module)
  RSCP_ID_CFG_SET         = 0x080, // STN->MODx: targetNode, key(u8), value32(LE), flags(u8), seq(u8)
  RSCP_ID_CFG_ACK         = 0x081  // MODx->STN: nodeId, key(u8), result(u8), seq(u8), appliedValue32(LE)
};

// ------------------------- PTT state
enum : uint8_t { RSCP_PTT_OFF = 0, RSCP_PTT_ON = 1 };

// ------------------------- Error severity
enum : uint8_t { RSCP_SEV_INFO = 0, RSCP_SEV_WARN = 1, RSCP_SEV_ERROR = 2, RSCP_SEV_FATAL = 3 };

// ------------------------- Mast VSWR state
enum : uint8_t { RSCP_VSWR_OK = 0, RSCP_VSWR_HIGH = 1, RSCP_VSWR_ERR = 2 };

// ------------------------- Drive state
enum : uint8_t { RSCP_DRIVE_OK = 0, RSCP_DRIVE_LOW = 1, RSCP_DRIVE_HIGH = 2 };

// ------------------------- Error bit definitions (err_bits in RSCP_ID_ERR_STATUS)
enum : uint32_t {
  RSCP_ERR_NONE        = 0,
  RSCP_ERR_PTT_FAIL    = (1UL << 0),
  RSCP_ERR_VSWR_HIGH   = (1UL << 1),
  RSCP_ERR_VBAT_LOW    = (1UL << 2),
  RSCP_ERR_TEMP_HIGH   = (1UL << 3),
  RSCP_ERR_HUM_HIGH    = (1UL << 4),
  RSCP_ERR_SENSOR_FAIL = (1UL << 5),
  RSCP_ERR_OVERTIME    = (1UL << 6),
  RSCP_ERR_CAN_LOSS    = (1UL << 7)
};

// ------------------------- Config keys (RSCP_ID_CFG_SET)
enum : uint8_t {
  RSCP_CFG_PTT_MAX_MS         = 1,  // uint32
  RSCP_CFG_STOP_PTT_ERR_MASK  = 2,  // uint32
  RSCP_CFG_COMPASS_PERIOD_MS  = 3,  // uint32
  RSCP_CFG_ROTATOR_PERIOD_MS  = 4,  // uint32

  // Station UI
  RSCP_CFG_LCD_AUTOROTATE     = 20, // uint32 (0/1)
  RSCP_CFG_LCD_ROTATE_MS      = 21, // uint32

  // Mast thresholds (raw / placeholders; calibrate later)
  RSCP_CFG_VBAT_LOW_RAW       = 10, // uint32
  RSCP_CFG_VSWR1_HIGH_RAW     = 11, // uint32
  RSCP_CFG_VSWR2_HIGH_RAW     = 12, // uint32
  RSCP_CFG_TEMP1_HIGH_X10     = 13, // uint32
  RSCP_CFG_TEMP2_HIGH_X10     = 14, // uint32
  RSCP_CFG_HUM_HIGH_PCT       = 15  // uint32
};

// ------------------------- Helpers (LE packing)
static inline void rscp_put_u16(uint8_t *d, uint16_t v) { d[0] = (uint8_t)(v & 0xFF); d[1] = (uint8_t)((v >> 8) & 0xFF); }
static inline void rscp_put_i16(uint8_t *d, int16_t v)  { rscp_put_u16(d, (uint16_t)v); }
static inline uint16_t rscp_get_u16(const uint8_t *d)   { return (uint16_t)d[0] | ((uint16_t)d[1] << 8); }
static inline int16_t  rscp_get_i16(const uint8_t *d)   { return (int16_t)rscp_get_u16(d); }

static inline void rscp_put_u32(uint8_t *d, uint32_t v) {
  d[0] = (uint8_t)(v & 0xFF);
  d[1] = (uint8_t)((v >> 8) & 0xFF);
  d[2] = (uint8_t)((v >> 16) & 0xFF);
  d[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline uint32_t rscp_get_u32(const uint8_t *d) {
  return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}
