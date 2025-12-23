# Pinouts

Single source of truth for **hardware pin mappings** across the RemoteStationCAN modules.

**Rules / conventions**
- Pin mappings are defined in each module’s sketch as `PIN_*` constants (or `#define`s).
- This document mirrors the *intended wiring* and should be updated whenever pins change in code or hardware.
- **SPI pins** for MCP2515 follow the board’s hardware SPI pins (Nano: D11/D12/D13). Only **CS** (and optionally **INT**) are assigned here.
- **I2C pins** follow the board’s I2C pins (Nano: A4/A5).
- “Optional/Reserved” pins are chosen so we can add features later without re-wiring core functions.

---

## Module 1 — Station Controller (Arduino Nano / AVR)

### Core IO
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D3 | PTT_IN | In | External PTT input (debounced in code) |
| D4 | PTT_OUT_CONFIRM | Out | Output back to sequencer/radio confirming remote PTT success |
| D5 | PTT_LED | Out | Local PTT indicator LED |
| D2 | UI_BUTTON | In | User button (INPUT_PULLUP) |

### Display (I2C LCD)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| A4 | SDA | I/O | LCD I2C data |
| A5 | SCL | I/O | LCD I2C clock |

### CAN (MCP2515)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D10 | CAN_CS | Out | MCP2515 chip-select |
| *(D11/D12/D13)* | MOSI/MISO/SCK | — | Hardware SPI (board default) |
| *(optional)* | CAN_INT | In | **Not used in current Module1 mapping** (D2 is used for UI button). Module1 currently polls CAN. |

### Optional / reserved (recommended)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D6 | UI_ROT_A | In | Rotary encoder A (future) |
| D7 | UI_ROT_B | In | Rotary encoder B (future) |
| D8 | UI_ROT_SW | In | Rotary encoder push (future) |
| A0–A3 | ANALOG_SPARE | In | Spare analogs for future local sensing |

---

## Module 2 — Masthead Protection + PTT (Arduino Nano / AVR)

**Definitive masthead mapping (matches your wiring plan).**

### CAN (MCP2515)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D10 | CAN_CS | Out | MCP2515 chip-select |
| D2 | CAN_INT | In | MCP2515 interrupt (recommended; active-low typical) |
| *(D11/D12/D13)* | MOSI/MISO/SCK | — | Hardware SPI (board default) |

### PTT / control
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D7 | PTT_OUT | Out | Keys transverter/linear via opto/relay |
| D6 | PTT_CONFIRM_IN | In | Confirms remote switching (polarity defined in code) |
| D4 | PTT_MIRROR_LED | Out | LED mirrors PTT state (local) |

### Sensors
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D8 | TEMP1_1WIRE | I/O | DS18B20 (Temp1) |
| A0 | FWD_RAW | In | Forward power (ADC raw) |
| A1 | REV_RAW | In | Reverse power (ADC raw) |
| A2 | VBAT_RAW | In | Supply voltage (ADC raw) |
| A4 | SDA | I/O | Colour sensor I2C (optional) |
| A5 | SCL | I/O | Colour sensor I2C (optional) |

### Optional / reserved (recommended)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D9 | TEMP2_1WIRE | I/O | DS18B20 (Temp2) optional |
| D5 | INHIBIT_IN | In | Optional inhibit/interlock input (INPUT_PULLUP) |
| D3 | SPARE_DIGITAL | I/O | Spare digital IO |
| A3 | SPARE_ANALOG | In | Spare analog input |
| A6 | FWD2_RAW | In | Optional second forward channel (analog-only) |
| A7 | REV2_RAW | In | Optional second reverse channel (analog-only) |

---

## Module 3 — Compass (Pi Pico / RP2040 + BNO085 + MCP2515)

### BNO085 (SPI)
| Pin (GP) | Signal | Dir | Notes |
|---|---|---:|---|
| GP5 | BNO_CS | Out | BNO085 chip-select |
| GP6 | BNO_INT | In | BNO085 interrupt |
| GP7 | BNO_RST | Out | BNO085 reset (optional but recommended) |
| *(board SPI pins)* | MOSI/MISO/SCK | — | Uses board SPI pins configured in code/core |

### CAN (MCP2515)
| Pin (GP) | Signal | Dir | Notes |
|---|---|---:|---|
| GP9 | CAN_CS | Out | MCP2515 chip-select |
| *(optional)* | CAN_INT | In | Use a free GP pin if you wire MCP2515 INT |
| *(board SPI pins)* | MOSI/MISO/SCK | — | Uses board SPI pins configured in code/core |

### RF mitigation / power gating
| Pin (GP) | Signal | Dir | Notes |
|---|---|---:|---|
| GP15 | RF_ENABLE | Out | Drives relay/FET to disable sensor during PTT/high RF (optional feature) |

---

## Module 4 — Rotator Node (Arduino Nano / AVR + MCP2515)

### CAN (MCP2515)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D10 | CAN_CS | Out | MCP2515 chip-select |
| *(D11/D12/D13)* | MOSI/MISO/SCK | — | Hardware SPI (board default) |
| *(optional)* | CAN_INT | In | Recommended if you add MCP2515 INT wiring |

### Rotator control outputs
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D7 | ROT_CW | Out | Relay CW (active HIGH) |
| D8 | ROT_CCW | Out | Relay CCW (active HIGH) |

### Position feedback
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| A0 | ROT_POT | In | Position pot input (ADC) |

### Optional / reserved (recommended)
| Pin | Signal | Dir | Notes |
|---|---|---:|---|
| D4 | ROT_BRAKE | Out | Optional brake relay/SSR |
| D5 | LIMIT_CW_IN | In | Optional limit switch |
| D6 | LIMIT_CCW_IN | In | Optional limit switch |
| A1–A3 | ANALOG_SPARE | In | Spare analogs (current sense, etc.) |

---
