# LD2450_Presence

Reads an HLK-LD2450 over UART1 (256000 baud). Zones are configured on the radar
(HLKRadarTool); it then only reports targets inside them.

## Behaviour
- **Lights on:** `min-frames` consecutive frames with a target (radar sends ~10/s).
- **Lights off:** `off-delay-s` after the last target; a new target restarts the delay.
- **Radar silent 5 s:** Info shows "radar offline"; the off-delay then runs out normally.
- **Dimming** is PWM into each XL6013 FB node, driven by this usermod (not by WLED LED
  outputs), so slat on/off and presets never change picture-light brightness.
  `dim-inverted` (default on): more PWM = less current, so 100 % = 0 % PWM = rated current.
- **Over-temperature:** at `temp-limit-c` both lights are held off; they come back
  at 10 °C below the limit if still wanted. A sensor fault (open/short) is reported
  and does not cut the lights.
- **Presets:** with `preset-if-off-only` (default), the on-preset fires only if WLED is
  off, and the off-preset only undoes what the usermod turned on.

## Settings (Config → Usermods → LD2450)
| Key | Default | Notes |
|---|---|---|
| rx-pin / tx-pin | 7 / 10 | tx only needed for Bluetooth on/off |
| out1-pin / out2-pin | 4 / 5 | XL6013 EN |
| active-low | false | inverts both EN outputs |
| dim1-pin / dim2-pin | 3 / 1 | -1 = no dimming |
| dim-inverted | true | |
| dim-freq | 20000 | Hz, 100–40000, 10-bit |
| bri1-pct / bri2-pct | 100 | brightness at boot; changing it applies immediately |
| off-delay-s | 60 | |
| min-frames | 2 | |
| max-range-mm | 0 | extra range limit, 0 = radar zones only |
| on-preset / off-preset | 0 | 0 = none |
| preset-if-off-only | true | |
| therm-pin | 0 | -1 = no thermistor |
| ntc-r25 / ntc-beta | 10000 / 3950 | |
| ntc-series-r | 10000 | fixed divider resistor |
| ntc-to-gnd | true | NTC on the low side, fixed R to 3V3 |
| temp-limit-c | 75 | 0 = no cut-off |
| ha-discovery | true | |

Pin changes need a reboot for WLED's own serial check; the usermod itself re-inits on save.

## JSON API (`POST /json/state`)
- `{"LD2450":{"enabled":false}}`: automation off, lights off (not persisted)
- `{"LD2450":{"out1":true}}`: light on/off (out1/out2), holds until the next presence change
- `{"LD2450":{"bri1":40}}`: brightness 0–100 % (bri1/bri2), runtime only
- `{"LD2450":{"bt":false}}`: radar Bluetooth off/on

`GET /json/state` → `LD2450`: `presence`, `targets`, `nearest` (mm), `out1/2`, `bri1/2`, `temp` (°C or null), `overtemp`.

## MQTT (device topic prefix)
| Topic | Direction | Payload |
|---|---|---|
| `/presence` | out, retained | `on` / `off` |
| `/light1/state`, `/light2/state` | out, retained | `ON` / `OFF` (actual output, incl. over-temp) |
| `/light1/bri`, `/light2/bri` | out, retained | 0–100 |
| `/light1/set`, `/light2/set` | in | `ON` / `OFF` |
| `/light1/bri/set`, `/light2/bri/set` | in | 0–100 |
| `/temperature` | out, retained, 30 s | °C or `unknown` |
| `/overtemp` | out, retained | `on` / `off` |

HA discovery creates: a Presence occupancy sensor, Light 1 and Light 2 (dimmable), and Board Temperature.

## Hardware notes
- Each FB injection path needs RC filtering **and a series diode**. Without the diode,
  0 % PWM pulls current out of FB and the LEDs run above rated current.
- EN pull-downs: 4.7 k keeps EN under 0.8 V during boot even with the XL6013's 10 µA
  leakage and any C3 reset pull-up.
- The C3 ADC is reliable to about 2.5 V, so with a 10 k/10 k divider readings saturate
  below roughly 0 °C. That's fine for board temperature.
- **Bench testing without an NTC:** set `therm-pin` to -1 (or `temp-limit-c` to 0). A
  floating ADC pin reads random values and can trip the over-temperature cut.
