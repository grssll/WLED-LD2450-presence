# LD2450_Presence

Reads an HLK-LD2450 over UART1. Zones are configured on the radar itself
(HLKRadarTool app); the radar then only reports targets inside them, so the
usermod only decides "any target present?".

## Behaviour
- On: `min-frames` consecutive frames with a target (radar sends ~10/s).
- Off: `off-delay-s` after the last frame with a target. A new target restarts the delay.
- Radar silent for 5 s: Info shows "radar offline" and the off-delay runs out normally.
- `preset-if-off-only` (default on): on-preset only fires if WLED is off; the
  off-preset only fires if the usermod applied the on-preset.

## Settings (Config → Usermods → LD2450)
| Key | Default | Notes |
|---|---|---|
| enabled | true | |
| rx-pin / tx-pin | 20 / 21 | tx optional (Bluetooth command only) |
| out1-pin / out2-pin | 4 / 5 | |
| active-low | false | inverts both outputs |
| off-delay-s | 60 | |
| min-frames | 2 | |
| max-range-mm | 0 | extra range limit, 0 = radar zones only |
| on-preset / off-preset | 0 | 0 = none |
| preset-if-off-only | true | |
| ha-discovery | true | |

## JSON API (`POST /json/state`)
- `{"LD2450":{"enabled":false}}` disable automation, outputs off (not persisted)
- `{"LD2450":{"out1":true}}` manual override of out1/out2 until the next presence change
- `{"LD2450":{"bt":false}}` radar Bluetooth off/on (sends enable-config, 0x00A4, restart)

State (`GET /json/state`) includes `LD2450.presence`, `targets`, `nearest` (mm), `out1`, `out2`.

## MQTT
- `<device topic>/presence` → `on` / `off` (retained)
- HA discovery: `binary_sensor` with device class `occupancy`, availability from WLED's LWT.
