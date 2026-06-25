📡 CAN Protocol – Wall Switch → LED Controller

This document describes the CAN message format used by wall switch nodes to control the ESP32-IDF-LED-CONTROLLER hub.

🧱 Frame Type

Standard CAN frame (11-bit identifier)

No extended ID

No RTR

Classic CAN (not CAN-FD)

🆔 CAN Identifier (11-bit Layout)

The identifier encodes both the message type and the switch ID.

  10 9 8 7 | 6 5 4 3 2 1 0
+-----------+----------------+
| Msg Type  |   Node ID      |
+-----------+----------------+
   4 bits        7 bits
Formula
CAN_ID = (MessageType << 7) | (NodeID & 0x7F);
For Wall Switch Events
MessageType = 0x1   // LIGHTING_COMMAND

The NodeID field represents the Switch ID (source device).

📦 Payload Format

Minimum DLC = 2 bytes

Byte	Field	Description
0	Button	Button number (1..N)
1	Action	Event type
2	Brightness	For EVT_DIM only (DLC ≥ 3): brightness 0..100

Additional bytes (3–7) reserved (e.g. fade_ms).

🎯 Action Codes
Value	Meaning
1	CLICK
2	HOLD
3	DOUBLE_CLICK

Additional (optional) action codes supported by the hub:
4	TRIPLE_CLICK
5	LONG_HOLD
6	HOLD_REPEAT
7	DIM	Payload byte 2 = brightness 0..100. Hub matches the same binding as CLICK for (node_id, button) and applies that brightness to the target.

---

## Hub to Node messages

### NODE_CONFIG (MessageType = 0x3)

CAN_ID = (0x3 << 7) | target_node_id. Payload: byte 0 = command, bytes 1-7 = command-specific data.

- 0x01 CMD_SET_NODE_ID: new_id (1..126)
- 0x02 CMD_SET_INPUT_CFG: bytes after command: `input_index` (0..5), `input_id`, `mode` (0 momentary, 1 toggle). Legacy DLC 4 ends here. DLC 5 adds `gpio` (0..48 or 0xFF unset). DLC 6 adds `active_high` (0 = active low / switch to GND with pull-up, 1 = active high with pull-down). If DLC < 6, node uses active low.
- 0x03 CMD_SET_INPUT_COUNT: count (1..6)
- 0x04 CMD_SET_TIMING: timing sent as two 6-byte frames (cmd + 5 data). Frame 1: data[1]=0 (part), data[2..3]=click_max_ms LE, data[4..5]=double_click_gap_ms LE. Frame 2: data[1]=1 (part), data[2..3]=hold_min_ms LE, data[4..5]=long_hold_min_ms LE. All values uint16, 0–65535 ms.
- 0x05 CMD_FIND_ME: duration_min (1–30). On **mechanical WS2812 nodes**, blinks the 12-LED strip on D5 (GPIO 7); `CMD_SET_FIND_ME_OUTPUT` is ignored. On other nodes, drives the configured GPIO.
- 0x06 CMD_SET_FIND_ME_OUTPUT: output_index (GPIO). Not used on WS2812 mechanical nodes.
- 0x07 CMD_SET_INPUT_LABEL: input_index, total_len_or_0xFF, up to 6 chars per frame. Multi-frame for labels longer than 6 chars. Byte 2 = total length (1..24) on first frame, 0xFF on continuation; use 0 to clear label.
- 0x08 CMD_SET_DATETIME: Unix timestamp (uint32_t LE, bytes 1–4). Hub sends UTC; LCD applies CMD_SET_TIMEZONE and uses localtime() for display.
- 0x09 CMD_REBOOT: no payload; node restarts.
- 0x0A CMD_SET_CAN_LINK_INDICATOR: gpio (0–48 = CAN link LED: solid=good link, flash=bad/no link; 0xFF=disable). Mechanical node only.
- 0x0B CMD_SET_TIMEZONE: multi-frame like CMD_SET_INPUT_LABEL: byte 1 = total_len (first frame) or 0xFF (continuation), bytes 2–7 = TZ string (e.g. `Australia/Sydney`), 6 chars per frame. Hub sends before CMD_SET_DATETIME so LCD can setenv("TZ", ...) and show local time.
- 0x0C CMD_SET_NIGHT_LIGHT: `enabled` (0=off, 1=on), `brightness` (0–100). **Mechanical node only** — WS2812 night light (warm white); stored in node NVS. Does not affect LCD nodes.
- 0x0D CMD_SET_WS2812_CLICK_EFFECT: `effect` (0=strobe, 1=chase). **Mechanical node only** — stores click/double-click LED feedback style in NVS; does not drive the strip until the user presses a switch. Click and double-click use the same effect.
- 0x0E CMD_SET_WS2812_EFFECT_PARAMS: `effect_id` (0=strobe, 1=chase, 2=find_me), `r`, `g`, `b` (0–255), `timing_ms` uint16 LE (10–2000). **Mechanical node only** — stores RGB and speed in NVS; does not animate on receipt. Strobe: half-period ms; chase: step interval ms; find-me: blink toggle ms.

See **[WS2812_MECHANICAL_NODE.md](WS2812_MECHANICAL_NODE.md)** for hub UI (Strip lighting modal), HTTP API (`set_night_light`, `set_ws2812_click_effect`, `set_ws2812_effect_params`), GPIO 7 rules, and troubleshooting.

### NODE_STATE_FEEDBACK (MessageType = 0x4)

Hub to Node: per-button brightness for LCD nodes. CAN_ID = (0x4 << 7) | target_node_id, DLC = 4. Payload bytes 0-3 = brightness for button 1-4 (0-100, or 0xFF if no binding). Hub sends when output state changes; each byte is the bound output brightness for that button.

### CAN_OTA_REMOTE (MessageType = 0xE) / CAN_OTA_NODE (MessageType = 0xF)

Node-paced OTA for mechanical nodes (mcp-can-boot-style). Hub uses **0xE** (lower bus priority); node responses on **0xF** win arbitration. See **[CAN_OTA.md](CAN_OTA.md)**.

**8-byte payload** (both directions):

| Byte | Field |
|------|--------|
| 0–1 | Target `node_id` u16 BE |
| 2 | Opcode |
| 3 | `FLASH_DATA`: bits 7–5 = length (1–4), bits 4–0 = offset[4:0]; else `0` |
| 4–7 | Data bytes or u32 offset/CRC BE |

| Opcode | Value | Direction | Purpose |
|--------|-------|-----------|---------|
| `OTA_FLASH_READY` | `0x04` | Node → hub | Ready; bytes 4–7 = next write offset u32 BE |
| `OTA_FLASH_INIT` | `0x06` | Hub → node | Start; bytes 4–7 = image size u32 BE |
| `OTA_FLASH_DATA` | `0x08` | Hub → node | 1–4 payload bytes at current offset |
| `OTA_FLASH_DATA_ERR` | `0x0D` | Node → hub | Offset mismatch; bytes 4–7 = expected offset |
| `OTA_FLASH_DONE` | `0x10` | Hub → node | End; bytes 4–7 = CRC32 BE (`0` = streaming finalize) |
| `OTA_FLASH_COMPLETE` | `0x14` | Node → hub | Success |
| `OTA_FLASH_ERROR` | `0x15` | Node → hub | Failure; byte 4 = reason |
| `OTA_FLASH_ABORT` | `0x18` | Either | Cancel |

Hub waits for `FLASH_READY` before each `FLASH_DATA` — no per-frame ACK or sequence numbers.

### HEARTBEAT (MessageType = 0x8) — extended payload

Minimum DLC 2 (backward compatible). Extended fields (DLC ≥ 5):

| Byte | Field |
|------|--------|
| 2–3 | `fw_version u16 LE` (e.g. `0x0100` = v1.0) |
| 4 | `ota_capable` (`1` = supports CAN OTA) |

Old nodes send zeros; hub treats as USB-only / unknown version. See **[NODE_HEARTBEAT_SPEC.md](NODE_HEARTBEAT_SPEC.md)**.

---

🧪 Example Messages
Example 1 — Switch 10, Button 2, CLICK
ID:  0x08A
DLC: 2
DATA: 02 01

Explanation:

CAN_ID = (0x1 << 7) | 10
       = 0x80 | 0x0A
       = 0x08A
Example 2 — Switch 5, Button 1, HOLD
ID:  0x085
DLC: 2
DATA: 01 02
🧠 Protocol Behavior

Wall switches send event messages only.

The LED Controller Hub listens for MessageType = LIGHTING_COMMAND.

The hub extracts:

switchId (from CAN ID)

button

action

The hub matches (switchId, button, action) against configured bindings.

The hub performs the configured lighting action (toggle, dim, scene, etc).

Wall switches do not directly send brightness or output commands.

⚙️ System Requirements

Bitrate must match across all nodes (e.g., 250k or 500k)

Exactly two 120Ω termination resistors on the bus

All nodes share common GND

Classic CAN only (no CAN-FD)
