# RemoteStationCAN Arduino Project

This bundle is structured so the **CAN protocol is shared** across all modules via an Arduino library, while each module remains a separate Arduino sketch.

## Folder layout
- `libraries/RSCP/` : shared protocol library (install once)
- `sketches/` : one sketch per module + a CAN monitor sanity test
- `docs/` : pinouts + CAN message reference

## 1) Install the RSCP library (required)

1. Arduino IDE → File → Preferences → note **Sketchbook location**
2. Copy the folder `libraries/RSCP` into:

   `<Sketchbook>/libraries/RSCP`

3. Restart Arduino IDE.

You should now be able to include:
```cpp
#include <rscp_can_protocol.h>
```

## 2) Open and upload each module

Open the relevant sketch under `sketches/`, select the right board+port, and upload.

- `Module1_Station_Controller`
- `Module2_Masthead_Protection_PTT`
- `Module3_Compass_BNO085_CAN`
- `Module4_Rotator_Node_unified`

## 3) Sanity test (recommended)

Before connecting RF/PTT hardware, verify the CAN bus using:

`sketches/SanityTest_CANMonitor/SanityTest_CANMonitor.ino`

- Arduino Nano/Uno + MCP2515
- Serial: **115200**
- Prints every CAN frame and decodes key RSCP frames
- Prints `Alive: M1..M4` based on BOOT/HEARTBEAT/CFG_ACK last seen within 5 seconds
- Also sends **HEADING_QUERY** and **ROT_QUERY** once per second and prints approximate RTT when replies arrive
- If you don’t see RTT values, check Modules 3/4 implement the query frames and are connected to the bus

## Docs

- `docs/Pinouts.md`
- `docs/CAN_Messages.md`

## Quick troubleshooting

- If nothing prints in the monitor: check CAN bitrate, MCP2515 wiring (SPI), and 120Ω termination.
- If only some nodes show alive: check each node sends BOOT/HEARTBEAT and shares the same protocol header.