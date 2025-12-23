# RSCP CAN Messages

Protocol version: **1.5** (Docs aligned to current Module1/Module2 fixed sketches)

Node IDs (convention): **1=Station, 2=Masthead, 3=Compass, 4=Rotator**.

## Conventions

- CAN is **standard 11‑bit IDs**.
- All frames are **8 bytes**.
- Multi‑byte integers are **little‑endian**.
- Unless otherwise stated, `data[0]` is `node_id` (the sender) on telemetry/status frames.

> Note on naming: you’ll see **VBAT** in a few historical key names. In practice this is **PSU / supply rail voltage at the masthead** (measured after any long/thin feed wiring), so Module 1 can show “Volts” and you can alarm on undervoltage/overvoltage.

---

## Frame IDs

| CAN ID | Name | Purpose |
|---:|---|---|
| `0x001` | `RSCP_ID_EMERG_PTT_OFF` | Emergency PTT OFF (highest priority) |
| `0x010` | `RSCP_ID_PTT_CMD` | PTT command (0=off,1=on) |
| `0x011` | `RSCP_ID_PTT_STATUS` | Remote PTT status/confirm + latency |
| `0x012` | `RSCP_ID_ERR_STATUS` | Latched error flags/status |
| `0x013` | `RSCP_ID_ERR_CLEAR` | Manual clear errors command |
| `0x020` | `RSCP_ID_BOOT` | Boot/reboot announcement |
| `0x021` | `RSCP_ID_HEARTBEAT` | Heartbeat / uptime |
| `0x030` | `RSCP_ID_ENV_TELEM` | Environmental telemetry (Temp/Hum/PSU volts) |
| `0x031` | `RSCP_ID_RF_TELEM` | RF telemetry (VSWR/Drive state, PTT active) |
| `0x060` | `RSCP_ID_HEADING` | Compass heading (degrees) |
| `0x061` | `RSCP_ID_HEADING_QUERY` | Query heading now |
| `0x062` | `RSCP_ID_HEADING_CONFIG` | Compass config |
| `0x070` | `RSCP_ID_ROT_SET_AZ` | Set rotator target azimuth |
| `0x071` | `RSCP_ID_ROT_STATUS` | Rotator current azimuth/status |
| `0x072` | `RSCP_ID_ROT_CONFIG` | Rotator config |
| `0x073` | `RSCP_ID_ROT_QUERY` | Query rotator status now |
| `0x080` | `RSCP_ID_CFG_SET` | Generic config set (dest,key,value) |
| `0x081` | `RSCP_ID_CFG_ACK` | Config acknowledge (src,status,key) |

---

## PTT

### `RSCP_ID_PTT_CMD` (0x010) — Station → Masthead

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `ptt_cmd` | u8 | `0=OFF`, `1=ON` |
| 1 | `seq` | u8 | Sequence number (increment per command) |
| 2 | `flags` | u8 | Reserved for future; set 0 |
| 3..7 | reserved | — | Set 0 |

### `RSCP_ID_PTT_STATUS` (0x011) — Masthead → Station

Sent in response to `PTT_CMD` (and on any PTT failure/abort/overtime drop).

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `ptt_state` | u8 | `0=OFF`, `1=ON` (masthead local state) |
| 1 | `seq` | u8 | Echo of last `PTT_CMD.seq` |
| 2 | `result` | u8 | `0=OK`, `1=FAIL` |
| 3 | `overtime_drop` | u8 | `0/1` (PTT forced off due to max‑TX timer) |
| 4..5 | `latency_ms` | u16 | Time from cmd RX → confirm/fail (ms, capped 65535) |
| 6 | `flags` | u8 | Bitfield (below) |
| 7 | `node_id` | u8 | Source node (normally `2`) |

`flags` (byte 6):
- bit0: `CONFIRMED` (PTT confirm input asserted)
- bit1: `INHIBITED` (interlock/inhibit active)
- bit2: `OVERTIME` (max‑TX timer tripped)
- bits3..7: reserved

---

## Errors

### `RSCP_ID_ERR_STATUS` (0x012) — Masthead → Station

Sent immediately on a new error and periodically as status.

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `node_id` | u8 | Source node (`2`) |
| 1 | `err_latched` | u8 | `0/1` |
| 2..3 | `err_bits` | u16 | Latched error bitmask |
| 4..5 | `drop_mask` | u16 | Which errors are configured to drop PTT |
| 6 | `ptt_active` | u8 | `0/1` |
| 7 | `seq` | u8 | Rolling counter (optional use) |

Suggested `err_bits` assignments:
- bit0: `ERR_VSWR`
- bit1: `ERR_DRIVE`
- bit2: `ERR_TEMP`
- bit3: `ERR_HUM`
- bit4: `ERR_PSU_VOLT`  *(historical “VBAT”)*  
- bit5: `ERR_PTT_CONFIRM_FAIL`
- bit6: `ERR_INHIBIT_ACTIVE`
- bits7..15: reserved

### `RSCP_ID_ERR_CLEAR` (0x013) — Station → Masthead

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `dest_node` | u8 | Typically `2` |
| 1 | `flags` | u8 | Reserved (set 0) |
| 2..7 | reserved | — | Set 0 |

---

## Telemetry (final set)

### `RSCP_ID_ENV_TELEM` (0x030) — Masthead → Station

Environmental + PSU/supply monitoring.  
Rules (masthead):
- **Temp** always eligible
- **Humidity** only sampled when **PTT is OFF**
- **PSU voltage** always eligible

| Byte | Name | Type | Units / Notes |
|---:|---|---|---|
| 0 | `node_id` | u8 | Source node (`2`) |
| 1..2 | `temp_c_x10` | i16 | Temperature ×10 (e.g. `221` = 22.1°C) |
| 3 | `humidity_pct` | u8 | 0–100 (0 if not sampled / disabled) |
| 4..5 | `psu_mv` | u16 | PSU/supply millivolts measured at masthead |
| 6 | `sensor_flags` | u8 | Bitfield (below) |
| 7 | `seq` | u8 | Rolling counter |

`sensor_flags` (byte 6):
- bit0: `TEMP_VALID`
- bit1: `HUM_VALID`
- bit2: `PSU_VALID`
- bit3: `TEMP2_VALID` (reserved; future)
- bits4..7: reserved

### `RSCP_ID_RF_TELEM` (0x031) — Masthead → Station

RF‑related states.  
Rules (masthead):
- **VSWR** only read when **PTT is ON**
- **Drive** (colour sensor) only read when **PTT is ON**

States are intentionally coarse for the “front screen” (OK / HIGH / ERR etc).

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `node_id` | u8 | Source node (`2`) |
| 1 | `vswr_state` | u8 | `0=OK`, `1=HIGH`, `2=ERR` |
| 2 | `drive_state` | u8 | `0=OK`, `1=LOW`, `2=HIGH`, `3=ERR` |
| 3 | `ptt_active` | u8 | `0/1` |
| 4 | `rf_flags` | u8 | Bitfield (below) |
| 5 | reserved | u8 | Set 0 |
| 6 | reserved | u8 | Set 0 |
| 7 | `seq` | u8 | Rolling counter |

`rf_flags` (byte 4):
- bit0: `VSWR_VALID`
- bit1: `DRIVE_VALID`
- bit2: `COLOR_SENSOR_PRESENT` (optional)
- bits3..7: reserved

---

## Boot / Heartbeat

### `RSCP_ID_BOOT` (0x020) — ModuleX → Station

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `node_id` | u8 | Source node |
| 1 | `boot_reason` | u8 | 0=power‑up, 1=watchdog, 2=software reset (best‑effort) |
| 2..5 | `uptime_s` | u32 | Seconds (optional; 0 if unknown) |
| 6..7 | reserved | — | Set 0 |

### `RSCP_ID_HEARTBEAT` (0x021) — ModuleX → Station

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `node_id` | u8 | Source node |
| 1 | `status` | u8 | 0=OK, non‑zero reserved |
| 2..5 | `uptime_s` | u32 | Seconds since boot |
| 6..7 | reserved | — | Set 0 |

---

## Config (RSCP_ID_CFG_SET)

### `RSCP_ID_CFG_SET` (0x080) — Station → ModuleX

Single key/value write.

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `dest_node` | u8 | Target node (2/3/4) |
| 1 | `key` | u8 | Config key (see table below) |
| 2..3 | reserved | — | Set 0 |
| 4..7 | `value_u32` | u32 | Value (interpret per key) |

### `RSCP_ID_CFG_ACK` (0x081) — ModuleX → Station

| Byte | Name | Type | Notes |
|---:|---|---|---|
| 0 | `node_id` | u8 | Source node |
| 1 | `status` | u8 | `0=OK`, `1=FAIL` |
| 2 | `key` | u8 | Echo of `CFG_SET.key` |
| 3..7 | reserved | — | Set 0 |

### Config keys

| Key | Name | Meaning |
|---:|---|---|
| 1 | `RSCP_CFG_PTT_MAX_MS` | Max transmit time before forced drop (ms) |
| 2 | `RSCP_CFG_STOP_PTT_ERR_MASK` | Bitmask: which errors drop PTT |
| 3 | `RSCP_CFG_COMPASS_PERIOD_MS` | Compass send period (ms) |
| 4 | `RSCP_CFG_ROTATOR_PERIOD_MS` | Rotator status period (ms) |
| 10 | `RSCP_CFG_VBAT_LOW_RAW` | **PSU undervoltage threshold (raw ADC)** *(historical name)* |
| 11 | `RSCP_CFG_VSWR1_HIGH_RAW` | VSWR high threshold (raw ADC FWD/REV derived; placeholder) |
| 12 | `RSCP_CFG_VSWR2_HIGH_RAW` | Optional 2nd VSWR threshold (future) |
| 13 | `RSCP_CFG_TEMP1_HIGH_X10` | Temp1 high threshold (°C×10) |
| 14 | `RSCP_CFG_TEMP2_HIGH_X10` | Temp2 high threshold (°C×10) |
| 15 | `RSCP_CFG_HUM_HIGH_PCT` | Humidity high threshold (%) |
