# Config settings and brightness defaults

## `config.settings`

Persisted under NVS via `GET` / `POST /api/config` as part of the nested `config` object.

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `defaultBrightness` | number 1–100 | `50` | Brightness used when a command omits `brightness` |
| `groupStaggerMs` | number ≥ 0 | `0` | Inter-channel delay (ms) for staggered group fades |

Example:

```json
{
  "settings": {
    "defaultBrightness": 40,
    "groupStaggerMs": 150
  }
}
```

The Advanced tab Global Settings UI loads and saves these fields with **Save All Configurations**. After save, the hub reloads settings immediately (no reboot required for brightness behavior).

## Command brightness rules

Lighting commands arrive from HTTP `/api/command`, MQTT, CAN bindings, and the local panel. Unspecified brightness is represented as **`-1`** internally (HTTP/MQTT parsers no longer coerce missing brightness to `0` or `50`).

| `state` | Result |
|---------|--------|
| `OFF` | Always 0 |
| `TOGGLE` → off | 0 |
| `TOGGLE` → on | Explicit `brightness` if ≥ 0; else `defaultBrightness` |
| `ON` | Explicit if ≥ 0; else `defaultBrightness` |
| empty (slider dim) | Explicit level; if omitted, `defaultBrightness` |

Explicit `0` is valid when brightness is present in the JSON (for example a dim command to off).

### Call sites

- **Web Command tab**: sends `brightness` only when “Use brightness” is checked; otherwise the hub uses the global default.
- **Control page sliders**: always send an explicit brightness.
- **CAN bindings**: omit `brightness` in the binding action → `-1` → global default; optional per-binding brightness remains authoritative.
- **CAN `EVT_DIM`**: payload brightness is always explicit.
- **LCD panel toggle**: omits brightness → global default.

## Related UI

- Advanced → Default Brightness slider (same glowing track style as Command / Binding sliders)
- Real-time control page (`/control`) per-output sliders
