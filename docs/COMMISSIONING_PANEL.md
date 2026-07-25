# Olimex commissioning panel

Local ST7789 (320×240) LCD + EC11 rotary encoder UI for **Olimex ESP32-POE** hub builds. Implemented in `components/hsg_panel` and started from `main.cpp` when `BOARD_OLIMEX_POE` is defined.

## Features

- **Outputs**: schematic-style connectors for mapped PCA channels; press encoder to toggle the selected channel
- Toggle-on brightness comes from `config.settings.defaultBrightness` (see [CONFIG_AND_BRIGHTNESS.md](CONFIG_AND_BRIGHTNESS.md))
- Live pad refresh while the Outputs page is visible
- Status / network / nodes views for commissioning

## Hardware (defaults in `src/hardware_config.h`)

| Signal | GPIO | Notes |
|--------|------|-------|
| SPI host | `SPI2_HOST` | Separate from CAN on SPI3 |
| SCLK | 14 | UEXT |
| MOSI | 2 | Silk “SDA” on many 7-pin modules |
| CS | none (`-1`) | Common 7-pin modules |
| DC | 0 | |
| RST | 15 | |
| BL | none | Tie backlight to 3.3 V if needed |
| Encoder A/B/SW | 34 / 36 / 39 | |

## Build

Included automatically for the `olimex-poe` PlatformIO environment via `hsg_panel` in `src/CMakeLists.txt`. Rack32 builds keep the component linked but panel start is a no-op without Olimex panel pins.
