# WS2812 strip — mechanical node (SwitchUI / hub)

Mechanical nodes (`NODE_TYPE_MECHANICAL` = 2) can drive a **12× WS2812B** strip for:

- **Night light** — hub-controlled on/off and brightness (warm white, persisted in node NVS)
- **Click feedback** — short **strobe** or **chase** on physical **click** and **double-click** (same effect for both; persisted in NVS)
- **Configurable RGB and speed** — per-effect colors and timing via CAN `0x0E` (strobe, chase, find-me)

Firmware: **SwitchUI** env `node_min_c3` (Seeed XIAO ESP32-C3 + MCP2515 CAN board).  
Hub: **LED-Controller** web UI + `POST /api/node/config`.

---

## Hardware (XIAO ESP32-C3)

| Item | Value |
|------|--------|
| LED data | **GPIO 7** (board **D5**) |
| LED count | 12 (build flag `WS2812_COUNT`) |
| CAN (MCP2515) CS | **GPIO 20** (`CAN_CS_GPIO`, board D7) |
| Strip power | **5 V** to WS2812; **common GND** with the MCU |
| Data level | **3.3 V → 5 V level shifter** on GPIO 7 → strip DIN (required for reliable 24/7 operation) |
| Decoupling | Capacitor at strip power recommended (100–1000 µF) |

**GPIO 7 is reserved for the strip.** Do not assign it as a switch input (`CMD_SET_INPUT_CFG` gpio is rejected).

**CAN link fault (automatic):** Hub sends `SENSOR_DATA (0x2)` keepalive every ~7 s. If the node sees no hub traffic for 30 s, **WS2812 pixel 0** flashes red (500 ms) regardless of night light; LEDs 1–11 are unchanged.

**Find Me** uses the WS2812 strip via `CMD_FIND_ME` (all LEDs blink configured find-me color). `CMD_SET_FIND_ME_OUTPUT` is ignored on mechanical WS2812 nodes.

---

## CAN commands (hub → node, `NODE_CONFIG` 0x3)

Target ID in CAN ID low 7 bits must match the node’s configured ID (`CFG node=N` on serial).

| Cmd | Name | Payload (after cmd byte) | Effect on strip |
|-----|------|--------------------------|-----------------|
| `0x05` | `CMD_FIND_ME` | `duration_min` (1–30) | Blink all 12 LEDs using find-me RGB + timing for N minutes; then restore night light |
| `0x0C` | `CMD_SET_NIGHT_LIGHT` | `enabled` (0/1), `brightness` (0–100) | Turns night light on/off; applies warm white at brightness |
| `0x0D` | `CMD_SET_WS2812_CLICK_EFFECT` | `effect` (0=strobe, 1=chase) | **Stores** effect only; does **not** animate the strip |
| `0x0E` | `CMD_SET_WS2812_EFFECT_PARAMS` | `effect_id`, `r`, `g`, `b`, `timing_ms` u16 LE | **Stores** RGB + speed in NVS; does **not** animate until next click or find-me |

**`timing_ms` semantics (0x0E):**

| effect_id | Effect | timing_ms meaning |
|-----------|--------|-------------------|
| 0 | Strobe | Half-period ms (default 45) |
| 1 | Chase | Step interval ms (default 50) |
| 2 | Find Me | Blink toggle ms (default 150) |

**Defaults** (when NVS empty):

| Effect | RGB | timing_ms |
|--------|-----|-----------|
| Strobe | 220, 180, 80 | 45 |
| Chase | 80, 60, 24 | 50 |
| Find Me | 180, 120, 20 | 150 |

Click/double-click events from the input engine call the stored effect locally (not via CAN).

---

## Hub web UI

On the **Nodes** tab, each **mechanical** node has:

| Control | Action |
|---------|--------|
| **Configure** | Node ID, inputs, switch types (no find-me) |
| **Strip lighting** | Modal: effect colors/speed, night light, click effect type, find-me |
| **Reboot** / **Remove** | As before |

**Strip lighting modal**

- **Click effect** — dropdown (strobe / chase) → **Save click effect** → sends `set_ws2812_click_effect` (`0x0D`).
- **Strobe / Chase / Find Me** — color picker + speed input + **Save** per effect → sends `set_ws2812_effect_params` (`0x0E`).
- **Night light** — checkbox + brightness slider → **Apply night light** → sends `set_night_light` (`0x0C`).
- **Find Me** — duration + **Find Me** → sends `find_me` (`0x05`).

**Configure modal (mechanical):** Find Me section is hidden; use **Strip lighting** for find-me on the D6 strip.

Hub persists `night_light_on`, `night_light_brightness`, `ws2812_click_effect`, and `ws2812_effects` (strobe/chase/find_me RGB + timing) in node JSON.

---

## Hub HTTP API

`POST /api/node/config` with JSON body:

**Night light**

```json
{
  "target_node_id": 4,
  "command": "set_night_light",
  "enabled": true,
  "brightness": 40
}
```

**Click effect**

```json
{
  "target_node_id": 4,
  "command": "set_ws2812_click_effect",
  "effect": "strobe"
}
```

`effect` may also be `0` / `1` or `"chase"`.

**Effect RGB + speed**

```json
{
  "target_node_id": 4,
  "command": "set_ws2812_effect_params",
  "effect": "strobe",
  "rgb": [220, 180, 80],
  "timing_ms": 45
}
```

`effect` accepts `"strobe"` / `"chase"` / `"find_me"` or `0` / `1` / `2`. Hub clamps RGB to 0–255 and timing to 10–2000 ms.

**Find Me**

```json
{
  "target_node_id": 4,
  "command": "find_me",
  "duration_min": 5
}
```

Mechanical nodes: do **not** send `set_find_me_output` — the strip is used automatically.

---

## Node firmware behaviour

- **Driver**: ESP-IDF RMT, GRB order, 12 LEDs on GPIO 7 (SwitchUI `node_min_c3`).
- **Task**: Dedicated FreeRTOS `ws2812` task + command queue; only the task calls RMT refresh.
- **API**: Hub/CAN/input use `ws2812_post_*` (non-blocking queue).
- **Boot**: Red sanity flash ~1.5 s inside ws2812 task, then NVS night-light state.
- **Night light on**: Baseline re-sent every **10 minutes**.
- **Recovery**: On refresh failure → deinit/reinit once and retry; fault state retries init every 60 s.
- **Find Me**: Blink configured find-me RGB; restores night light when done.
- **Chase**: Forward 0→11 then backward 10→1; 2 s watchdog.

### Serial messages (USB)

| Line | Meaning |
|------|---------|
| `WS2812: driver init 12 LEDs GPIO 7 RMT` | Driver OK (ws2812 task) |
| `CONFIG: find-me on for N min (WS2812 strip)` | CAN `0x05` received |
| `WS2812: find-me N min` | Strip find-me started |
| `CONFIG: night light on brightness=N` | CAN `0x0C` received |
| `WS2812: applied night light on brightness=N` | Strip update succeeded |
| `WS2812: stats ok=... fail=... state=...` | Health counters (every 60 s) |
| `CONFIG: WS2812 click effect strobe (stored)` | CAN `0x0D` received (no strip animation) |
| `CONFIG: WS2812 effect params id=... (stored)` | CAN `0x0E` received (no strip animation) |

If you see **CONFIG** but not **WS2812: applied** for night light, reflash the node firmware.

### Build / flash (node)

```bash
pio run -e node_min_c3 -t upload --upload-port COMx
```

Hub (embedded HTML):

```bash
idf.py build flash
```

---

## Troubleshooting

| Symptom | Check |
|---------|--------|
| Hub “Saved” but no LEDs | Node serial: `CFG node=` must match hub **target_node_id**; look for `WS2812: applied` |
| Only CONFIG, no applied | Old node firmware; flash latest `node_min_c3` |
| Boot flash works, hub does not | Same as above; confirm `ws2812_start_task()` in node firmware |
| Strip stops after hours, CAN OK | Add 5 V level shifter on data; check serial for fault/retry lines |
| Chase stuck mid-pattern | Flash latest firmware (watchdog + step-based chase) |
| Very dim | Raise brightness (e.g. 40–60); night light caps RGB internally |
| Strip always on after “off” | Flash latest firmware (off path fills black) |

---

## Related docs

- [CAN_PROTOCOL.md](CAN_PROTOCOL.md) — full command list
- [NODE_HEARTBEAT_SPEC.md](NODE_HEARTBEAT_SPEC.md) — announce / online detection
