#pragma once
/*
  Remote Station Controller CAN Protocol (11-bit IDs, MCP2515)

  - Deterministic PTT/error: low IDs win arbitration
  - Modular nodes (Station=1, Mast=2, Compass=3, Rotator=4)
  - Fixed 8-byte frames, little-endian multibyte integers
*/

#include <Arduino.h>

// ------------------------- Protocol version
#define RSCP_PROTO_VER_MAJOR 1
#define RSCP_PROTO_VER_MINOR 5

// ------------------------- Module IDs
enum : uint8_t {
  RSCP_MOD_STATION = 1,
  RSCP_MOD_MAST    = 2,
  RSCP_MOD_COMPASS = 3,
  RSCP_MOD_ROTATOR = 4
};


// Back-compat aliases (older code used RSCP_MOD_*, newer uses RSCP_NODE_*)
#define RSCP_NODE_STATION RSCP_MOD_STATION
#define RSCP_NODE_MAST    RSCP_MOD_MAST
#define RSCP_NODE_COMPASS RSCP_MOD_COMPASS
#define RSCP_NODE_ROTATOR RSCP_MOD_ROTATOR

// ------------------------- CAN IDs (11-bit) ---------------------------------
// Lower ID = higher priority on bus.
enum : uint16_t {
  // Emergency broadcast
  RSCP_ID_EMERG_PTT_OFF      = 0x001, // STN->ALL : data0=RSCP_MOD_STATION, data1=seq

  // PTT control handshake
  RSCP_ID_PTT_CMD            = 0x010, // STN->MAST: data0=1(on)/0(off), data1=seq, data2=flags
  RSCP_ID_PTT_STATUS         = 0x011, // MAST->STN: data0=state, data1=seq, data2=status_flags, data3=err_latched(1/0)
  RSCP_ID_ERR_STATUS         = 0x012, // MODx->STN: data0=module_id, data1=severity, data2..5=err_bits(uint32)

  RSCP_ID_ERR_CLEAR          = 0x013, // STN->MODx: data0=module_id(or 0=MAST), data1=scope(0=all), data2..5=mask(uint32)

  // Boot / heartbeat
  RSCP_ID_BOOT               = 0x020, // MODx->STN: data0=module_id, data1=reset_cause, data2=proto_major, data3=proto_minor
  RSCP_ID_HEARTBEAT          = 0x021, // MODx->STN: data0=module_id, data1=flags, data2..5=uptime_s (uint32)
  RSCP_ID_STATUS_SUMMARY     = 0x022, // MAST->STN: data0=module_id, data1=statusBits, data2=faultCode, data3=faultDetail, data4..5=vbat_raw, data6=reserved, data7=reserved, // MODx->STN: data0=module_id, data1=flags, data2..5=uptime_s (uint32)

  // Mast telemetry
  RSCP_ID_ENV_TELEM          = 0x030, // MAST->STN: temp_c_x10(int16), rh_pct(uint8), ds18_c_x10(int16) or 0x7FFF if n/a
  RSCP_ID_RF_TELEM           = 0x031, // MAST->STN: vbat_raw(uint16), fwd_raw(uint16), rev_raw(uint16), drive_state(uint8), vswr_state(uint8)

  // Compass
  RSCP_ID_HEADING            = 0x060, // COMP->STN: heading_deg_x10(uint16), status(uint8)
  RSCP_ID_HEADING_QUERY      = 0x061, // STN->COMP: data0=1(query_now)
  RSCP_ID_HEADING_CONFIG     = 0x062, // STN->COMP: period_ms(uint16), delta_x10(uint16)

  // Rotator
  RSCP_ID_ROT_SET_AZ         = 0x070, // STN->ROT: target_deg_x10(uint16)
  RSCP_ID_ROT_STATUS         = 0x071, // ROT->STN: az_deg_x10(uint16), target_deg_x10(uint16), state(uint8)
  RSCP_ID_ROT_CONFIG         = 0x072, // STN->ROT: period_ms(uint16)
  RSCP_ID_ROT_QUERY          = 0x073, // STN->ROT: data0=1(query_now)

  // Generic settings (persist on module)
  RSCP_ID_CFG_SET            = 0x080, // STN->MODx: module_id, key(uint8), value(uint32 LE)
  RSCP_ID_CFG_ACK            = 0x081  // MODx->STN: module_id, key, status(0=ok), reserved
};

// ------------------------- PTT state
enum : uint8_t {
  RSCP_PTT_OFF = 0,
  RSCP_PTT_ON  = 1
};

// ------------------------- Error severity (for display/prioritization)
enum : uint8_t {
  RSCP_SEV_INFO  = 0,
  RSCP_SEV_WARN  = 1,
  RSCP_SEV_ERROR = 2,
  RSCP_SEV_FATAL = 3
};

// ------------------------- Mast VSWR state (for 16x2 LCD display)
enum : uint8_t {
  RSCP_VSWR_OK   = 0,
  RSCP_VSWR_HIGH = 1,
  RSCP_VSWR_ERR  = 2
};

// ------------------------- Drive state (for 16x2 LCD display)
enum : uint8_t {
  RSCP_DRIVE_OK   = 0,
  RSCP_DRIVE_LOW  = 1,
  RSCP_DRIVE_HIGH = 2
};

// ------------------------- Error bit definitions (err_bits in RSCP_ID_ERR_STATUS)
enum : uint32_t {
  RSCP_ERR_NONE        = 0,
  RSCP_ERR_PTT_FAIL    = (1UL << 0),
  RSCP_ERR_VSWR_HIGH   = (1UL << 1),
  RSCP_ERR_VBAT_LOW    = (1UL << 2),
  RSCP_ERR_TEMP_HIGH   = (1UL << 3),
  RSCP_ERR_HUM_HIGH    = (1UL << 4),
  RSCP_ERR_SENSOR_FAIL = (1UL << 5),
  RSCP_ERR_OVERTIME    = (1UL << 6)
};

// ------------------------- Config keys (RSCP_ID_CFG_SET)
enum : uint8_t {
  RSCP_CFG_PTT_MAX_MS         = 1, // uint32
  RSCP_CFG_STOP_PTT_ERR_MASK  = 2, // uint32
  RSCP_CFG_COMPASS_PERIOD_MS  = 3, // uint32
  RSCP_CFG_ROTATOR_PERIOD_MS  = 4, // uint32

// Module 2 thresholds (raw / scaled x10 placeholders; calibrate later)
RSCP_CFG_VBAT_LOW_RAW       = 10, // uint32 (raw ADC threshold)
RSCP_CFG_VSWR1_HIGH_RAW     = 11, // uint32 (raw/derived)
RSCP_CFG_VSWR2_HIGH_RAW     = 12, // uint32 (raw/derived)
RSCP_CFG_TEMP1_HIGH_X10     = 13, // uint32 (degC x10)
RSCP_CFG_TEMP2_HIGH_X10     = 14, // uint32 (degC x10)
RSCP_CFG_HUM_HIGH_PCT       = 15  // uint32 (%)
};

// ------------------------- Helpers
static inline void rscp_u32_to_le(uint8_t *d, uint32_t v) {
  d[0] = (uint8_t)(v & 0xFF);
  d[1] = (uint8_t)((v >> 8) & 0xFF);
  d[2] = (uint8_t)((v >> 16) & 0xFF);
  d[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline uint32_t rscp_le_to_u32(const uint8_t *d) {
  return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}


static inline void rscp_put_u16(uint8_t *d, uint16_t v) { d[0]=(uint8_t)(v); d[1]=(uint8_t)(v>>8); }
static inline void rscp_put_i16(uint8_t *d, int16_t v)  { rscp_put_u16(d, (uint16_t)v); }
static inline uint16_t rscp_get_u16(const uint8_t *d)   { return (uint16_t)d[0] | ((uint16_t)d[1]<<8); }
static inline int16_t  rscp_get_i16(const uint8_t *d)   { return (int16_t)rscp_get_u16(d); }
static inline void rscp_put_u32(uint8_t *d, uint32_t v) { rscp_u32_to_le(d,v); }
static inline uint32_t rscp_get_u32(const uint8_t *d)   { return rscp_le_to_u32(d); }
