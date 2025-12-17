# RSCP CAN Messages

Protocol version: **1.4**

Node IDs (convention): **1=Station, 2=Masthead, 3=Compass, 4=Rotator**.

## Frame IDs

| CAN ID | Name | Purpose |
|---:|---|---|
| `0x001` | `RSCP_ID_EMERG_PTT_OFF` | Emergency PTT OFF (highest priority) |
| `0x010` | `RSCP_ID_PTT_CMD` | PTT command (0=off,1=on) |
| `0x011` | `RSCP_ID_PTT_STATUS` | Remote PTT status/confirm |
| `0x012` | `RSCP_ID_ERR_STATUS` | Latched error flags |
| `0x013` | `RSCP_ID_ERR_CLEAR` | Manual clear errors command |
| `0x020` | `RSCP_ID_BOOT` | Boot/reboot announcement |
| `0x021` | `RSCP_ID_HEARTBEAT` | Heartbeat / uptime |
| `0x030` | `RSCP_ID_ENV_TELEM` | Environment telemetry |
| `0x031` | `RSCP_ID_RF_TELEM` | RF/PSU telemetry |
| `0x060` | `RSCP_ID_HEADING` | Compass heading (degrees) |
| `0x061` | `RSCP_ID_HEADING_QUERY` | Query heading now |
| `0x062` | `RSCP_ID_HEADING_CONFIG` | Compass config |
| `0x070` | `RSCP_ID_ROT_SET_AZ` | Set rotator target azimuth |
| `0x071` | `RSCP_ID_ROT_STATUS` | Rotator current azimuth/status |
| `0x072` | `RSCP_ID_ROT_CONFIG` | Rotator config |
| `0x073` | `RSCP_ID_ROT_QUERY` |  |
| `0x080` | `RSCP_ID_CFG_SET` | Generic config set (key,value) |
| `0x081` | `RSCP_ID_CFG_ACK` | Config acknowledge (node,status,key) |

## Config keys (RSCP_ID_CFG_SET)

| Key | Name |
|---:|---|
| 1 | `RSCP_CFG_PTT_MAX_MS` |
| 2 | `RSCP_CFG_STOP_PTT_ERR_MASK` |
| 3 | `RSCP_CFG_COMPASS_PERIOD_MS` |
| 4 | `RSCP_CFG_ROTATOR_PERIOD_MS` |
| 10 | `RSCP_CFG_VBAT_LOW_RAW` |
| 11 | `RSCP_CFG_VSWR1_HIGH_RAW` |
| 12 | `RSCP_CFG_VSWR2_HIGH_RAW` |
| 13 | `RSCP_CFG_TEMP1_HIGH_X10` |
| 14 | `RSCP_CFG_TEMP2_HIGH_X10` |
| 15 | `RSCP_CFG_HUM_HIGH_PCT` |

## Payload notes

- Multi-byte integers are **little-endian**.
- BOOT/HEARTBEAT/CFG_ACK include `node_id` in `data[0]`.
