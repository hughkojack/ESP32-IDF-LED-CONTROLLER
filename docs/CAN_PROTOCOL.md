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

Additional bytes (2–7) are ignored by the hub.

🎯 Action Codes
Value	Meaning
1	CLICK
2	HOLD
3	DOUBLE_CLICK
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
