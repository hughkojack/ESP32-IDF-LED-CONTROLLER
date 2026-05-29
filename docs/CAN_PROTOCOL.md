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
- 0x05 CMD_FIND_ME: duration_min
- 0x06 CMD_SET_FIND_ME_OUTPUT: output_index
- 0x07 CMD_SET_INPUT_LABEL: input_index, total_len_or_0xFF, up to 6 chars per frame. Multi-frame for labels longer than 6 chars. Byte 2 = total length (1..24) on first frame, 0xFF on continuation; use 0 to clear label.
- 0x08 CMD_SET_DATETIME: Unix timestamp (uint32_t LE, bytes 1–4). Hub sends UTC; LCD applies CMD_SET_TIMEZONE and uses localtime() for display.
- 0x09 CMD_REBOOT: no payload; node restarts.
- 0x0A CMD_SET_CAN_LINK_INDICATOR: gpio (0–48 = CAN link LED: solid=good link, flash=bad/no link; 0xFF=disable). Mechanical node only.
- 0x0B CMD_SET_TIMEZONE: multi-frame like CMD_SET_INPUT_LABEL: byte 1 = total_len (first frame) or 0xFF (continuation), bytes 2–7 = TZ string (e.g. `Australia/Sydney`), 6 chars per frame. Hub sends before CMD_SET_DATETIME so LCD can setenv("TZ", ...) and show local time.
- 0x0C CMD_SET_NIGHT_LIGHT: enabled (0=off, 1=on), brightness (0–100). Mechanical node WS2812 strip; stored in node NVS.

### NODE_STATE_FEEDBACK (MessageType = 0x4)

Hub to Node: per-button brightness for LCD nodes. CAN_ID = (0x4 << 7) | target_node_id, DLC = 4. Payload bytes 0-3 = brightness for button 1-4 (0-100, or 0xFF if no binding). Hub sends when output state changes; each byte is the bound output brightness for that button.

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
