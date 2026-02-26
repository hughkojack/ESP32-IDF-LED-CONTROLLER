📘 HUB_INPUT_LOGIC.md
🧠 Hub Input Processing – Gesture Interpretation (V2)
Overview

In Protocol V2, nodes transmit only raw input events:

PRESS

RELEASE

LEVEL

The Hub is responsible for interpreting these into higher-level actions such as:

CLICK

HOLD

DOUBLE_CLICK

Direct ON/OFF control

This ensures:

Consistent behavior across all nodes

Centralized timing rules

No need to reflash node firmware for UX changes

1️⃣ Input State Model

For each (nodeId, input_id) pair, the Hub maintains:

is_down

press_timestamp

last_click_timestamp

click_pending

hold_fired

2️⃣ Timing Parameters
Parameter	Recommended Default
HOLD_MS	800 ms
DOUBLE_MS	350 ms

These values apply system-wide.

3️⃣ Event Handling Logic
On PRESS

Mark is_down = true

Record press_timestamp

Reset hold_fired = false

On RELEASE

If is_down == false → ignore.

Compute:

press_duration = now - press_timestamp
If press_duration ≥ HOLD_MS

Emit HOLD action

Clear pending click

Else (short press)

If within DOUBLE_MS of previous press:

Emit DOUBLE_CLICK

Clear click tracking

Else:

Mark as click_pending

Record last_click_timestamp

Periodic Timer (10–20ms loop)

If click_pending == true and
now - last_click_timestamp > DOUBLE_MS:

Emit CLICK

Clear click_pending

4️⃣ LEVEL Event Handling

LEVEL events represent maintained states.

Example logic:

Event	Hub Action
LEVEL=1	Turn ON
LEVEL=0	Turn OFF

LEVEL events bypass gesture detection.

5️⃣ Why Centralized Gesture Detection?

Advantages:

Uniform user experience

Single place to tune timing

Works across mechanical, touch, and smart devices

No node-specific gesture drift

🏗 Hub Responsibility Summary

Nodes report truth.
Hub defines meaning.
