# WLED LD2450 Presence (ESP32-C3)

WLED usermod for the HLK-LD2450 mmWave radar: presence switches two outputs
(e.g. picture lights), with an off-delay and optional WLED presets on
presence/absence. Presence is published over MQTT with Home Assistant discovery.

Built by GitHub Actions against WLED **v16.0.1** for **ESP32-C3**
(`ARDUINO_USB_CDC_ON_BOOT=1`, radar on UART1).

## Build
Push to `main` (or *Actions → Build WLED + LD2450 → Run workflow*).
The workflow runs host-side logic tests, then builds WLED with the usermod.
Download the artifact `wled-ld2450-esp32c3`:

| File | Use |
|---|---|
| `WLED-LD2450-C3-factory.bin` | First flash, at **0x0** (esp.huhn.me) |
| `WLED-LD2450-C3-ota.bin` | Later updates via WLED *Config → Security & Updates → Manual OTA* |

Release name is `ESP32-C3-LD2450`, so a stock WLED OTA bin won't silently
replace this build.

## Default wiring
| Function | GPIO |
|---|---|
| Radar TX → ESP RX | 20 |
| ESP TX → Radar RX (Bluetooth on/off only) | 21 |
| Output 1 | 4 |
| Output 2 | 5 |
| LED data (set in WLED LED prefs) | e.g. 6 or 7 |

Radar runs at 256000 baud, 5 V supply. Avoid GPIO 2/8/9 (strapping) and 18/19 (USB).

See `usermod/LD2450_Presence/readme.md` for settings and the JSON/MQTT API.
