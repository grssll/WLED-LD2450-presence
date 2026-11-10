# WLED LD2450 Presence (ESP32-C3 slat controller)

WLED usermod for the slat controller: an HLK-LD2450 mmWave radar switches two
XL6013 picture-light drivers, with per-light dimming, a board thermistor with
over-temperature cut-off, optional WLED presets, and Home Assistant discovery
(presence, two dimmable lights, board temperature).

Built by GitHub Actions against WLED **v16.0.1** for **ESP32-C3**.

## Build
Push to `main` (or *Actions → Build WLED + LD2450 → Run workflow*). The workflow
runs host-side logic tests, then builds WLED with the usermod. Artifact
`wled-ld2450-esp32c3`:

| File | Use |
|---|---|
| `WLED-LD2450-C3-factory.bin` | First flash, at **0x0** (esp.huhn.me) |
| `WLED-LD2450-C3-ota.bin` | Updates via WLED *Config → Security & Updates → Manual OTA* |

## Pin map (ESP32-C3-MINI-1)
| GPIO | Function |
|---|---|
| 0 | NTC divider (board temperature) |
| 1 | DIM2: PWM → RC → diode → XL6013 FB, light 2 |
| 3 | DIM1: PWM → RC → diode → XL6013 FB, light 1 |
| 4 | XL6013 EN, light 1 (4.7 k pull-down) |
| 5 | XL6013 EN, light 2 (4.7 k pull-down) |
| 6 | LED data (set in WLED LED Preferences), via level shifter |
| 7 | Radar TX → ESP RX |
| 10 | ESP TX → radar RX |
| 2, 8 | Strapping: pull-ups only |
| 9 | BOOT button |
| 18 / 19 | USB |
| 20 / 21 | Left free (test pads) so Improv/serial over USB keeps working |

See `usermod/LD2450_Presence/readme.md` for settings, JSON/MQTT API and hardware notes.
