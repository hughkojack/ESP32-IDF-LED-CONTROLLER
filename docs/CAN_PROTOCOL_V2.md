📡 CAN Protocol V2 – Node Input Event Specification

Protocol Version: 2.0
Applies To: ESP32-IDF-LED-CONTROLLER (Hub) and all Node devices
Frame Type: Standard 11-bit CAN (Classic CAN, not CAN-FD)

> DEPRECATED (repo configuration): the current hub firmware in this repo is configured for
> **event-based input** (CLICK/HOLD/etc) as specified in `docs/CAN_PROTOCOL.md`.
> This V2/raw-state document is retained for reference only.

🧱 Design Philosophy

Protocol V2 is designed to be:

Hardware-agnostic

Scalable

Consistent across all node types

Independent of UI interpretation (click, hold, etc.)

Nodes report input state changes only.
The Hub interprets behavior (click, hold, double click, scenes).

This ensures:

One firmware supports all switch types

Hub behavior can evolve without reflashing nodes

All devices behave consistently

🆔 CAN Identifier (11-bit)

The identifier encodes:

Bits	Field	Description
10–7	MessageType	4-bit category
6–0	NodeID	7-bit Node ID
Identifier Formula
CAN_ID = (MessageType << 7) | (NodeID & 0x7F);
V2 Input Event Type
MessageType = 0x1   // INPUT_EVENT

Example (Node ID = 10):

CAN_ID = (0x1 << 7) | 10
       = 0x80 | 0x0A
       = 0x08A
📦 Payload Format

Minimum DLC = 2 bytes.

Byte	Field	Description
0	input_id	Input number (1..255)
1	event	Event type
2	value	Used only for LEVEL event
3..	reserved	Future use
🎯 Event Types
Code	Name	Description
0x01	PRESS	Input became active
0x02	RELEASE	Input became inactive
0x03	LEVEL	Maintained state change
🧪 Examples
1️⃣ Momentary Button (Mechanical or Touch)

Node sends transitions only.

PRESS (Node 10, Input 2)
ID:  0x08A
DLC: 2
DATA: 02 01
RELEASE
ID:  0x08A
DLC: 2
DATA: 02 02

Works for:

Mechanical push button

Capacitive touch button

LVGL normal button

2️⃣ Toggle / Maintained Switch

Node sends state level.

Switch ON (value=1)
ID:  0x08A
DLC: 3
DATA: 01 03 01
Switch OFF (value=0)
ID:  0x08A
DLC: 3
DATA: 01 03 00

Works for:

Rocker switch

Shelly maintained input

LVGL toggle widget

🧠 Hub Behavior Model

The Hub performs interpretation logic.

PRESS / RELEASE Handling

The hub may derive:

CLICK

HOLD

DOUBLE_CLICK

Using centralized timing rules:

Parameter	Typical Value
HOLD threshold	800 ms
DOUBLE click window	350 ms

All nodes behave consistently because gesture detection occurs in one place.

> Implementation note (this repo): the current hub firmware primarily uses the raw events
> (`PRESS`, `RELEASE`, `LEVEL` -> `LEVEL_ON`/`LEVEL_OFF`) directly for bindings, as exposed
> in the web UI. Full CLICK/HOLD/DOUBLE_CLICK gesture synthesis may be added later.

LEVEL Handling

LEVEL events represent stable state.

The hub may:

Map LEVEL=1 → Light ON

Map LEVEL=0 → Light OFF

Map LEVEL → Scene activation

No gesture detection required.

⚙ Node Firmware Requirements

Each node input should define:

mode = momentary | toggle

active_level = high | low

debounce_ms = typically 20–50ms

Nodes must not encode UI-specific semantics into the protocol.

🔒 System Requirements

Classic CAN only

11-bit standard frames

Consistent bitrate across all nodes

Exactly two 120Ω termination resistors on the bus

Shared ground between nodes

🚀 Future Extension

Reserved for future event types:

Code	Purpose
0x04	VALUE (analog 0–100)
0x05	ROTARY_DELTA
0x06	SENSOR_EVENT

These can be added without breaking V2 compatibility.

🏗 Architecture Summary
Node Device
   ↓
INPUT_EVENT (CAN)
   ↓
LED Controller Hub
   ↓
Lighting Output / Scene / Automation

Nodes report truth.
Hub defines behavior.
