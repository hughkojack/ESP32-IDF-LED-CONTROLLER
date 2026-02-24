📘 NODE_FIRMWARE_GUIDE.md
🛠 Node Firmware Design Guide (Protocol V2)
Objective

All nodes must:

Support multiple input types

Use a single universal firmware

Report standardized INPUT_EVENT messages

1️⃣ Supported Input Modes

Each input channel must define:

Parameter	Description
mode	momentary or toggle
active_level	HIGH or LOW
debounce_ms	20–50 ms
2️⃣ Momentary Mode

Used for:

Push buttons

Touch buttons

Capacitive switches

Behavior

On stable edge:

Physical State	Event Sent
Becomes active	PRESS
Becomes inactive	RELEASE
3️⃣ Toggle Mode

Used for:

Rocker switches

Shelly maintained input

LVGL toggle widget

Behavior

On stable change:

State	Event Sent
ON	LEVEL value=1
OFF	LEVEL value=0
4️⃣ GPIO Input Processing
Debounce Algorithm

Sample input every 5–10ms

Confirm stable state for debounce_ms

Trigger event only after stable confirmation

5️⃣ LVGL Integration
For normal button widget

LV_EVENT_PRESSED → send PRESS

LV_EVENT_RELEASED → send RELEASE

For toggle widget

LV_EVENT_VALUE_CHANGED

Checked → send LEVEL=1

Unchecked → send LEVEL=0

6️⃣ CAN Transmission Format

See: CAN_PROTOCOL_V2.md

Example:

PRESS
ID:  (0x1<<7) | nodeId
DLC: 2
DATA: input_id, 0x01
LEVEL
ID:  (0x1<<7) | nodeId
DLC: 3
DATA: input_id, 0x03, value
7️⃣ Design Rules

Nodes must:

Not interpret gestures (click/hold/double)

Not embed lighting logic

Not assume application behavior

Only report input state changes

🏗 Architecture Philosophy
Input Hardware
     ↓
Node Firmware (Normalize Input)
     ↓
CAN INPUT_EVENT
     ↓
Hub (Interpret + Execute)
     ↓
Lighting Outputs
🚀 Scalability

This model supports future expansion:

Rotary encoders

Dimmers

Analog sensors

Scene controllers

Environmental sensors

Without protocol redesign.
