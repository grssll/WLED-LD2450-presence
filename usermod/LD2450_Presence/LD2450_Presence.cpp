/*
 * LD2450_Presence - WLED usermod (WLED 16.x, ESP32-C3 slat controller)
 *
 *  - HLK-LD2450 radar on UART1 -> presence
 *  - two picture lights: XL6013 EN outputs, on with presence, off after an off-delay
 *  - per-light brightness: PWM into each XL6013 FB node (DIM1/DIM2), independent of
 *    WLED's own LEDs, so slat on/off/presets never change picture-light brightness
 *  - board NTC thermistor: temperature reporting + over-temperature cut of the lights
 *  - optional WLED presets on presence / absence
 *  - MQTT + Home Assistant discovery: presence, two dimmable lights, board temperature
 *
 * Default pins keep GPIO20/21 free: WLED disables Improv/serial over USB at boot if
 * either is claimed. Nothing is on a strapping pin (2/8/9).
 *
 * JSON API (POST /json/state):
 *   {"LD2450":{"enabled":false}}  automation off, lights off (not persisted)
 *   {"LD2450":{"out1":true}}      light 1/2 on/off, holds until the next presence change
 *   {"LD2450":{"bri1":40}}        light 1/2 brightness 0-100 % (runtime; boot value in settings)
 *   {"LD2450":{"bt":false}}       radar Bluetooth on/off (needs ESP TX -> radar RX)
 */

#include "wled.h"

#ifndef LD2450_RX_PIN
  #define LD2450_RX_PIN    7    // ESP RX  <- radar TX
#endif
#ifndef LD2450_TX_PIN
  #define LD2450_TX_PIN    10   // ESP TX  -> radar RX
#endif
#ifndef LD2450_OUT1_PIN
  #define LD2450_OUT1_PIN  4    // XL6013 EN, light 1
#endif
#ifndef LD2450_OUT2_PIN
  #define LD2450_OUT2_PIN  5    // XL6013 EN, light 2
#endif
#ifndef LD2450_DIM1_PIN
  #define LD2450_DIM1_PIN  3    // PWM -> RC -> diode -> FB, light 1
#endif
#ifndef LD2450_DIM2_PIN
  #define LD2450_DIM2_PIN  1    // PWM -> RC -> diode -> FB, light 2
#endif
#ifndef LD2450_THERM_PIN
  #define LD2450_THERM_PIN 0    // NTC divider (ADC1_CH0)
#endif

#define LD2450_BAUD        256000
#define LD2450_FRAME_LEN   30       // AA FF 03 00 | 3 x 8-byte targets | 55 CC
#define LD2450_TIMEOUT_MS  5000     // no valid frame for this long = radar offline
#define LD2450_CMD_GAP_MS  150      // spacing between radar config commands
#define LD2450_RX_BUDGET   128      // max bytes parsed per loop() pass
#define LD2450_DIM_BITS    10
#define LD2450_DIM_MAX     ((1u << LD2450_DIM_BITS) - 1)
#define LD2450_TEMP_MS     1000     // thermistor read interval
#define LD2450_TEMP_HYST   10       // degrees below the limit before the lights come back
#define LD2450_TEMP_PUB_MS 30000    // MQTT temperature publish interval
#define LD2450_VCC_MV      3300.0f  // divider supply

class LD2450Presence : public Usermod {
  private:
    struct HwCfg {
      int8_t   rx, tx, out[2], dim[2], therm;
      bool     activeLow, dimInverted;
      uint16_t dimFreq;
      bool same(const HwCfg& o) const {
        return rx == o.rx && tx == o.tx && out[0] == o.out[0] && out[1] == o.out[1] &&
               dim[0] == o.dim[0] && dim[1] == o.dim[1] && therm == o.therm &&
               activeLow == o.activeLow && dimInverted == o.dimInverted && dimFreq == o.dimFreq;
      }
    };

    // ---- configuration (Config -> Usermods -> LD2450) ----
    bool     enabled         = true;
    HwCfg    hw              = { LD2450_RX_PIN, LD2450_TX_PIN,
                                 { LD2450_OUT1_PIN, LD2450_OUT2_PIN },
                                 { LD2450_DIM1_PIN, LD2450_DIM2_PIN },
                                 LD2450_THERM_PIN, false, true, 20000 };
    uint16_t offDelaySec     = 60;      // seconds without a target before the lights go off
    uint8_t  minFrames       = 2;       // consecutive target frames to switch on (~10 frames/s)
    uint16_t maxRangeMm      = 0;       // extra range limit on top of the radar's zones, 0 = off
    uint8_t  onPreset        = 0;       // 0 = none
    uint8_t  offPreset       = 0;       // 0 = none
    bool     presetIfOffOnly = true;    // on-preset only if WLED is off; off-preset only undoes our own
    bool     haDiscovery     = true;
    uint8_t  briDefault[2]   = { 100, 100 };  // % at boot
    uint32_t ntcR25          = 10000;   // NTC resistance at 25 C
    uint16_t ntcBeta         = 3950;
    uint32_t ntcSeriesR      = 10000;   // fixed divider resistor
    bool     ntcToGnd        = true;    // NTC from ADC pin to GND, fixed R to 3V3
    uint8_t  tempLimitC      = 75;      // lights cut at/above this, back at limit-10; 0 = off

    // ---- runtime ----
    bool     initDone    = false;
    bool     uartActive  = false;
    bool     thermActive = false;
    uint8_t  ledcStart   = 255;
    uint8_t  frame[LD2450_FRAME_LEN];
    uint8_t  frameIdx    = 0;
    uint32_t lastFrameMs = 0;
    uint32_t lastSeenMs  = 0;
    uint8_t  hitStreak   = 0;           // 0 = latest frame had no target
    uint8_t  targetCount = 0;
    uint16_t nearestMm   = 0;
    bool     present     = false;
    bool     outState[2] = { false, false };   // requested light state (presence or manual)
    uint8_t  lightBri[2] = { 100, 100 };       // current brightness %
    bool     presetOwned = false;
    float    tempC       = NAN;
    bool     overTemp    = false;
    uint32_t lastTempMs  = 0;
    uint32_t lastTempPub = 0;
    bool     lightsDirty = true;        // light state/brightness changed, publish over MQTT
    bool     tempDirty   = true;
    volatile bool mqttFresh = false;    // (re)connected: subscribe + discovery from loop()

    // requests from the web/MQTT tasks, acted on in loop()
    volatile int8_t reqEnable = -1;
    volatile int8_t reqOut[2] = { -1, -1 };
    volatile int8_t reqBri[2] = { -1, -1 };
    volatile int8_t reqBt     = -1;
    volatile bool   reqHw     = false;
    HwCfg    pendHw;

    // non-blocking Bluetooth command sequence
    uint8_t  btStep   = 0;
    bool     btTarget = false;
    uint32_t btNextMs = 0;

    static const char _name[];

    // ---------------------------------------------------------------------

    static int16_t decodeSigned(uint16_t raw) {
      // LD2450: bit 15 set = positive, clear = negative; bits 0-14 = magnitude
      int16_t v = raw & 0x7FFF;
      return (raw & 0x8000) ? v : -v;
    }

    bool radarOnline(uint32_t now) const {
      return uartActive && lastFrameMs != 0 && (now - lastFrameMs) < LD2450_TIMEOUT_MS;
    }

    bool lightOn(uint8_t i) const { return outState[i] && !overTemp; }

    void applyOutput(uint8_t i) {
      if (hw.out[i] >= 0) digitalWrite(hw.out[i], (lightOn(i) != hw.activeLow) ? HIGH : LOW);
    }

    void setOutput(uint8_t i, bool on) {
      if (outState[i] != on) lightsDirty = true;
      outState[i] = on;
      applyOutput(i);
    }

    void applyDim(uint8_t i) {
      if (hw.dim[i] < 0 || ledcStart == 255) return;
      uint32_t duty = ((uint32_t)lightBri[i] * LD2450_DIM_MAX + 50) / 100;
      if (hw.dimInverted) duty = LD2450_DIM_MAX - duty;   // more PWM into FB = less LED current
      ledcWrite(ledcStart + i, duty);
    }

    void setBrightness(uint8_t i, uint8_t pct) {
      if (pct > 100) pct = 100;
      if (lightBri[i] != pct) lightsDirty = true;
      lightBri[i] = pct;
      applyDim(i);
    }

    void setOverTemp(bool ot) {
      overTemp = ot;
      applyOutput(0);
      applyOutput(1);
      lightsDirty = true;
      tempDirty   = true;
    }

    void initHardware() {
      // light enables first, so they are driven low as early as possible
      for (uint8_t i = 0; i < 2; i++) {
        if (hw.out[i] < 0) continue;
        if (PinManager::allocatePin(hw.out[i], true, PinOwner::UM_Unspecified)) {
          pinMode(hw.out[i], OUTPUT);
          applyOutput(i);
        } else {
          DEBUG_PRINTF_P(PSTR("LD2450: output pin %d unavailable\n"), hw.out[i]);
          hw.out[i] = -1;
        }
      }

      // dimming PWM: one LEDC channel pair (shared timer)
      if (hw.dim[0] >= 0 || hw.dim[1] >= 0) {
        ledcStart = PinManager::allocateLedc(2);
        if (ledcStart == 255) {
          DEBUG_PRINTLN(F("LD2450: no free LEDC channels"));
          hw.dim[0] = hw.dim[1] = -1;
        }
        for (uint8_t i = 0; i < 2 && ledcStart != 255; i++) {
          if (hw.dim[i] < 0) continue;
          if (PinManager::allocatePin(hw.dim[i], true, PinOwner::UM_Unspecified)) {
            ledcSetup(ledcStart + i, hw.dimFreq, LD2450_DIM_BITS);
            ledcAttachPin(hw.dim[i], ledcStart + i);
            applyDim(i);
          } else {
            DEBUG_PRINTF_P(PSTR("LD2450: dim pin %d unavailable\n"), hw.dim[i]);
            hw.dim[i] = -1;
          }
        }
        if (ledcStart != 255 && hw.dim[0] < 0 && hw.dim[1] < 0) {
          PinManager::deallocateLedc(ledcStart, 2);
          ledcStart = 255;
        }
      }

      // thermistor
      thermActive = hw.therm >= 0 && PinManager::allocatePin(hw.therm, false, PinOwner::UM_Unspecified);
      if (!thermActive) hw.therm = -1;
      tempC = NAN;
      lastTempMs = 0;
      if (overTemp) setOverTemp(false);

      // radar UART
      if (hw.rx >= 0 && PinManager::allocatePin(hw.rx, false, PinOwner::UM_Unspecified)) {
        if (hw.tx >= 0 && !PinManager::allocatePin(hw.tx, true, PinOwner::UM_Unspecified)) {
          DEBUG_PRINTF_P(PSTR("LD2450: TX pin %d unavailable\n"), hw.tx);
          hw.tx = -1;
        }
        Serial1.setRxBufferSize(256);
        Serial1.begin(LD2450_BAUD, SERIAL_8N1, hw.rx, hw.tx);
        uartActive = true;
      } else {
        if (hw.rx >= 0) { DEBUG_PRINTF_P(PSTR("LD2450: RX pin %d unavailable\n"), hw.rx); }
        hw.rx = -1;
      }
      frameIdx = 0; lastFrameMs = 0; hitStreak = 0;
      lightsDirty = true;
    }

    void releaseHardware() {
      if (uartActive) { Serial1.end(); uartActive = false; }
      if (hw.rx >= 0) PinManager::deallocatePin(hw.rx, PinOwner::UM_Unspecified);
      if (hw.tx >= 0) PinManager::deallocatePin(hw.tx, PinOwner::UM_Unspecified);
      for (uint8_t i = 0; i < 2; i++) {
        if (hw.out[i] >= 0) {
          digitalWrite(hw.out[i], hw.activeLow ? HIGH : LOW);   // leave the light off
          PinManager::deallocatePin(hw.out[i], PinOwner::UM_Unspecified);
        }
        if (hw.dim[i] >= 0 && ledcStart != 255) {
          ledcDetachPin(hw.dim[i]);
          PinManager::deallocatePin(hw.dim[i], PinOwner::UM_Unspecified);
        }
      }
      if (ledcStart != 255) { PinManager::deallocateLedc(ledcStart, 2); ledcStart = 255; }
      if (thermActive) { PinManager::deallocatePin(hw.therm, PinOwner::UM_Unspecified); thermActive = false; }
    }

    void setPresence(bool p) {
      if (p == present) return;
      present = p;
      setOutput(0, p);
      setOutput(1, p);
      publishPresence();
      if (p) {
        if (onPreset && (!presetIfOffOnly || ::bri == 0)) {
          applyPreset(onPreset, CALL_MODE_BUTTON_PRESET);
          presetOwned = true;
        }
      } else {
        if (offPreset && (!presetIfOffOnly || presetOwned)) applyPreset(offPreset, CALL_MODE_BUTTON_PRESET);
        presetOwned = false;
      }
    }

    void setEnabled(bool en) {
      if (en == enabled) return;
      enabled   = en;
      hitStreak = 0;
      if (!en) {   // stop automation, lights off, WLED's LEDs left alone
        present = false; presetOwned = false;
        setOutput(0, false);
        setOutput(1, false);
        publishPresence();
      }
    }

    // ---- thermistor ------------------------------------------------------

    void readTemperature(uint32_t now) {
      if (!thermActive || (lastTempMs && now - lastTempMs < LD2450_TEMP_MS)) return;
      lastTempMs = now ? now : 1;

      uint32_t mv = 0;
      for (uint8_t i = 0; i < 8; i++) mv += analogReadMilliVolts(hw.therm);
      mv /= 8;

      float t = NAN;
      if (mv > 20 && mv < (uint32_t)LD2450_VCC_MV - 50) {     // outside = open/short
        float v = (float)mv;
        float r = ntcToGnd ? (float)ntcSeriesR * v / (LD2450_VCC_MV - v)
                           : (float)ntcSeriesR * (LD2450_VCC_MV - v) / v;
        t = 1.0f / (1.0f / 298.15f + logf(r / (float)ntcR25) / (float)ntcBeta) - 273.15f;
      }
      if (isnan(t) != isnan(tempC)) tempDirty = true;
      tempC = t;

      if (!tempLimitC) { if (overTemp) setOverTemp(false); return; }
      if (isnan(t)) return;                                   // sensor fault: keep last state
      if (!overTemp && t >= tempLimitC) setOverTemp(true);
      else if (overTemp && t <= (float)tempLimitC - LD2450_TEMP_HYST) setOverTemp(false);
    }

    // ---- radar UART ------------------------------------------------------

    void readRadar(uint32_t now) {
      if (!uartActive) return;
      static const uint8_t header[4] = { 0xAA, 0xFF, 0x03, 0x00 };
      uint16_t budget = LD2450_RX_BUDGET;
      while (budget-- && Serial1.available()) {
        uint8_t b = Serial1.read();
        if (frameIdx < 4) {                       // hunting for the header
          if (b == header[frameIdx])  frame[frameIdx++] = b;
          else if (b == header[0])    { frame[0] = b; frameIdx = 1; }
          else                        frameIdx = 0;
          continue;
        }
        frame[frameIdx++] = b;
        if (frameIdx == LD2450_FRAME_LEN) {
          if (frame[28] == 0x55 && frame[29] == 0xCC) processFrame(now);
          frameIdx = 0;
        }
      }
    }

    void processFrame(uint32_t now) {
      lastFrameMs = now;
      uint8_t  count   = 0;
      uint16_t nearest = UINT16_MAX;
      for (uint8_t t = 0; t < 3; t++) {
        const uint8_t* p = frame + 4 + t * 8;   // x(2) y(2) speed(2) res(2), little-endian
        bool empty = true;
        for (uint8_t i = 0; i < 8; i++) if (p[i]) { empty = false; break; }
        if (empty) continue;                    // unused slot is all zeros
        int16_t x = decodeSigned(p[0] | (p[1] << 8));
        int16_t y = decodeSigned(p[2] | (p[3] << 8));
        uint16_t dist = (uint16_t)sqrtf((float)x * x + (float)y * y);
        if (maxRangeMm && dist > maxRangeMm) continue;
        count++;
        if (dist < nearest) nearest = dist;
      }
      targetCount = count;
      nearestMm   = count ? nearest : 0;
      if (!enabled) return;
      if (count) {
        lastSeenMs = now;
        if (hitStreak < UINT8_MAX) hitStreak++;
        if (!present && hitStreak >= minFrames) setPresence(true);
      } else {
        hitStreak = 0;
      }
    }

    // command frame: FD FC FB FA | len(2) | cmd(2) | value | 04 03 02 01
    void sendCommand(uint16_t cmd, const uint8_t* value, uint8_t len) {
      if (!uartActive || hw.tx < 0) return;
      static const uint8_t head[4] = { 0xFD, 0xFC, 0xFB, 0xFA };
      static const uint8_t tail[4] = { 0x04, 0x03, 0x02, 0x01 };
      uint16_t dataLen = 2 + len;
      Serial1.write(head, 4);
      Serial1.write((uint8_t)(dataLen & 0xFF)); Serial1.write((uint8_t)(dataLen >> 8));
      Serial1.write((uint8_t)(cmd & 0xFF));     Serial1.write((uint8_t)(cmd >> 8));
      if (len) Serial1.write(value, len);
      Serial1.write(tail, 4);
    }

    // enable config mode -> set Bluetooth -> restart (applies after restart)
    void runBtSequence(uint32_t now) {
      if (!btStep || (int32_t)(now - btNextMs) < 0) return;
      static const uint8_t cfgOn[2] = { 0x01, 0x00 };
      switch (btStep) {
        case 1: sendCommand(0x00FF, cfgOn, 2); break;
        case 2: { uint8_t v[2] = { (uint8_t)(btTarget ? 0x01 : 0x00), 0x00 }; sendCommand(0x00A4, v, 2); } break;
        case 3: sendCommand(0x00A3, nullptr, 0); break;
      }
      btStep   = (btStep >= 3) ? 0 : btStep + 1;
      btNextMs = now + LD2450_CMD_GAP_MS;
    }

    void handleRequests(uint32_t now) {
      if (reqHw) {
        reqHw = false; btStep = 0;
        releaseHardware();               // still uses the old pins and polarity
        hw = pendHw;
        initHardware();
      }
      if (reqEnable >= 0) { setEnabled(reqEnable); reqEnable = -1; }
      for (uint8_t i = 0; i < 2; i++) {
        if (reqOut[i] >= 0) { setOutput(i, reqOut[i]); reqOut[i] = -1; }
        if (reqBri[i] >= 0) { setBrightness(i, reqBri[i]); reqBri[i] = -1; }
      }
      if (reqBt >= 0) {
        if (uartActive && hw.tx >= 0 && !btStep) { btTarget = reqBt; btStep = 1; btNextMs = now; }
        reqBt = -1;
      }
    }

    // ---- MQTT ------------------------------------------------------------

    bool mqttReady() const {
#ifndef WLED_DISABLE_MQTT
      return WLED_MQTT_CONNECTED && mqttDeviceTopic[0];
#else
      return false;
#endif
    }

    void publishPresence() {
#ifndef WLED_DISABLE_MQTT
      if (!mqttReady()) return;
      char topic[MQTT_MAX_TOPIC_LEN + 16];
      snprintf_P(topic, sizeof(topic), PSTR("%s/presence"), mqttDeviceTopic);
      mqtt->publish(topic, 0, true, present ? "on" : "off");
#endif
    }

    void publishLights() {
#ifndef WLED_DISABLE_MQTT
      char topic[MQTT_MAX_TOPIC_LEN + 24], val[8];
      for (uint8_t i = 0; i < 2; i++) {
        if (hw.out[i] < 0) continue;
        snprintf_P(topic, sizeof(topic), PSTR("%s/light%u/state"), mqttDeviceTopic, i + 1);
        mqtt->publish(topic, 0, true, lightOn(i) ? "ON" : "OFF");
        snprintf_P(topic, sizeof(topic), PSTR("%s/light%u/bri"), mqttDeviceTopic, i + 1);
        snprintf_P(val, sizeof(val), PSTR("%u"), lightBri[i]);
        mqtt->publish(topic, 0, true, val);
      }
#endif
    }

    void publishTemp() {
#ifndef WLED_DISABLE_MQTT
      char topic[MQTT_MAX_TOPIC_LEN + 16], val[12];
      snprintf_P(topic, sizeof(topic), PSTR("%s/temperature"), mqttDeviceTopic);
      if (isnan(tempC)) strcpy(val, "unknown");
      else snprintf_P(val, sizeof(val), PSTR("%.1f"), tempC);
      mqtt->publish(topic, 0, true, val);
      snprintf_P(topic, sizeof(topic), PSTR("%s/overtemp"), mqttDeviceTopic);
      mqtt->publish(topic, 0, true, overTemp ? "on" : "off");
#endif
    }

    void subscribeTopics() {
#ifndef WLED_DISABLE_MQTT
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      for (uint8_t i = 0; i < 2; i++) {
        if (hw.out[i] < 0) continue;
        snprintf_P(topic, sizeof(topic), PSTR("%s/light%u/set"), mqttDeviceTopic, i + 1);
        mqtt->subscribe(topic, 0);
        snprintf_P(topic, sizeof(topic), PSTR("%s/light%u/bri/set"), mqttDeviceTopic, i + 1);
        mqtt->subscribe(topic, 0);
      }
#endif
    }

    // one HA discovery config; `suffix` makes the unique id and the config topic
    void publishConfig(const char* component, const char* suffix, DynamicJsonDocument& doc) {
#ifndef WLED_DISABLE_MQTT
      char uid[48], topic[112], buf[64];
      snprintf_P(uid, sizeof(uid), PSTR("%s_ld2450%s"), escapedMac.c_str(), suffix);
      doc[F("uniq_id")] = uid;
      doc[F("avty_t")]  = mqttStatusTopic;   // WLED LWT: online/offline
      JsonObject device = doc.createNestedObject(F("device"));
      device[F("name")] = serverDescription;
      snprintf_P(buf, sizeof(buf), PSTR("wled-sensor-%s"), mqttClientID);
      device[F("ids")]  = buf;
      device[F("mf")]   = F(WLED_BRAND);
      device[F("mdl")]  = F(WLED_PRODUCT_NAME);
      device[F("sw")]   = versionString;
      String json;
      serializeJson(doc, json);
      snprintf_P(topic, sizeof(topic), PSTR("homeassistant/%s/%s/config"), component, uid);
      mqtt->publish(topic, 0, true, json.c_str(), json.length());
#endif
    }

    void publishDiscovery() {
#ifndef WLED_DISABLE_MQTT
      char name[64], t1[MQTT_MAX_TOPIC_LEN + 24], t2[MQTT_MAX_TOPIC_LEN + 24],
           t3[MQTT_MAX_TOPIC_LEN + 24], t4[MQTT_MAX_TOPIC_LEN + 24], sfx[8];

      { // presence (same unique id as earlier builds)
        DynamicJsonDocument doc(1024);
        snprintf_P(name, sizeof(name), PSTR("%s Presence"), serverDescription);
        snprintf_P(t1, sizeof(t1), PSTR("%s/presence"), mqttDeviceTopic);
        doc[F("name")] = name; doc[F("dev_cla")] = F("occupancy");
        doc[F("pl_on")] = "on"; doc[F("pl_off")] = "off"; doc[F("stat_t")] = t1;
        publishConfig("binary_sensor", "", doc);
      }
      for (uint8_t i = 0; i < 2; i++) {   // dimmable picture lights
        if (hw.out[i] < 0) continue;
        DynamicJsonDocument doc(1024);
        snprintf_P(name, sizeof(name), PSTR("%s Light %u"), serverDescription, i + 1);
        snprintf_P(t1, sizeof(t1), PSTR("%s/light%u/set"),     mqttDeviceTopic, i + 1);
        snprintf_P(t2, sizeof(t2), PSTR("%s/light%u/state"),   mqttDeviceTopic, i + 1);
        snprintf_P(t3, sizeof(t3), PSTR("%s/light%u/bri/set"), mqttDeviceTopic, i + 1);
        snprintf_P(t4, sizeof(t4), PSTR("%s/light%u/bri"),     mqttDeviceTopic, i + 1);
        doc[F("name")] = name;
        doc[F("cmd_t")] = t1; doc[F("stat_t")] = t2;
        if (hw.dim[i] >= 0) { doc[F("bri_cmd_t")] = t3; doc[F("bri_stat_t")] = t4; doc[F("bri_scl")] = 100; }
        snprintf_P(sfx, sizeof(sfx), PSTR("_l%u"), i + 1);
        publishConfig("light", sfx, doc);
      }
      if (thermActive) {                  // board temperature
        DynamicJsonDocument doc(1024);
        snprintf_P(name, sizeof(name), PSTR("%s Board Temperature"), serverDescription);
        snprintf_P(t1, sizeof(t1), PSTR("%s/temperature"), mqttDeviceTopic);
        doc[F("name")] = name; doc[F("dev_cla")] = F("temperature");
        doc[F("unit_of_meas")] = "\xC2\xB0" "C"; doc[F("stat_t")] = t1;
        doc[F("stat_cla")] = F("measurement");
        publishConfig("sensor", "_temp", doc);
      }
#endif
    }

    void mqttLoop(uint32_t now) {
      if (!mqttReady()) return;
      if (mqttFresh) {
        mqttFresh = false;
        subscribeTopics();
        if (haDiscovery) publishDiscovery();
        publishPresence();
        lightsDirty = true;
        tempDirty   = true;
      }
      if (lightsDirty) { publishLights(); lightsDirty = false; }
      if (thermActive && (tempDirty || now - lastTempPub >= LD2450_TEMP_PUB_MS)) {
        publishTemp(); tempDirty = false; lastTempPub = now;
      }
    }

  public:
    void setup() override {
      for (uint8_t i = 0; i < 2; i++) lightBri[i] = briDefault[i];
      initHardware();
      initDone = true;
    }

    void loop() override {
      if (!initDone) return;
      uint32_t now = millis();
      handleRequests(now);
      readRadar(now);
      runBtSequence(now);
      readTemperature(now);
      if (!radarOnline(now)) { hitStreak = 0; targetCount = 0; nearestMm = 0; }
      if (enabled && present && hitStreak == 0 &&
          (now - lastSeenMs) >= (uint32_t)offDelaySec * 1000UL) {
        setPresence(false);
      }
      mqttLoop(now);
    }

    void onMqttConnect(bool sessionPresent) override { mqttFresh = true; }

    // topic arrives with the device topic already stripped, e.g. "/light1/set"
    bool onMqttMessage(char* topic, char* payload) override {
      char t[24];
      for (uint8_t i = 0; i < 2; i++) {
        snprintf_P(t, sizeof(t), PSTR("/light%u/set"), i + 1);
        if (!strcmp(topic, t)) {
          reqOut[i] = (!strcasecmp(payload, "on") || !strcmp(payload, "1") || !strcasecmp(payload, "true")) ? 1 : 0;
          return true;
        }
        snprintf_P(t, sizeof(t), PSTR("/light%u/bri/set"), i + 1);
        if (!strcmp(topic, t)) {
          int v = atoi(payload);
          reqBri[i] = v < 0 ? 0 : (v > 100 ? 100 : v);
          return true;
        }
      }
      return false;
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      uint32_t now = millis();

      JsonArray info = user.createNestedArray(FPSTR(_name));
      String s;
      if (!enabled)               s = F("disabled");
      else if (!uartActive)       s = F("no RX pin");
      else if (!radarOnline(now)) s = F("radar offline");
      else if (present && hitStreak == 0) {
        uint32_t delayMs = (uint32_t)offDelaySec * 1000UL, since = now - lastSeenMs;
        uint32_t left    = since < delayMs ? delayMs - since : 0;
        s = F("off in "); s += (left + 999) / 1000; s += F("s");
      }
      else if (present) { s = F("present, "); s += targetCount; s += F(" tgt @ "); s += nearestMm; s += F(" mm"); }
      else              s = F("clear");
      info.add(s);
      String btn = F(" <button class=\"btn btn-xs\" onclick=\"requestJson({");
      btn += FPSTR(_name);
      btn += enabled ? F(":{enabled:false}});\"><i class=\"icons on\">&#xe325;</i>")
                     : F(":{enabled:true}});\"><i class=\"icons off\">&#xe08f;</i>");
      btn += F("</button>");
      info.add(btn);

      char buf[64];
      JsonArray lights = user.createNestedArray(F("Picture lights"));
      if (overTemp) lights.add(F("off (over temperature)"));
      else {
        snprintf_P(buf, sizeof(buf), PSTR("1: %s %u%% / 2: %s %u%%"),
                   outState[0] ? "on" : "off", lightBri[0], outState[1] ? "on" : "off", lightBri[1]);
        lights.add(buf);
      }

      if (thermActive) {
        JsonArray temp = user.createNestedArray(F("Board temp"));
        if (isnan(tempC)) temp.add(F("sensor fault"));
        else { snprintf_P(buf, sizeof(buf), PSTR("%.1f &deg;C"), tempC); temp.add(buf); }
      }

      if (hw.tx >= 0 && uartActive) {
        JsonArray bt = user.createNestedArray(F("LD2450 Bluetooth"));
        String b = F("<button class=\"btn btn-xs\" onclick=\"requestJson({");
        b += FPSTR(_name); b += F(":{bt:true}});\">on</button> <button class=\"btn btn-xs\" onclick=\"requestJson({");
        b += FPSTR(_name); b += F(":{bt:false}});\">off</button>");
        bt.add(b);
      }
    }

    void addToJsonState(JsonObject& root) override {
      if (!initDone) return;
      JsonObject um = root[FPSTR(_name)];
      if (um.isNull()) um = root.createNestedObject(FPSTR(_name));
      um[F("enabled")]  = enabled;
      um[F("presence")] = present;
      um[F("targets")]  = targetCount;
      um[F("nearest")]  = nearestMm;
      um[F("out1")]     = outState[0];
      um[F("out2")]     = outState[1];
      um[F("bri1")]     = lightBri[0];
      um[F("bri2")]     = lightBri[1];
      um[F("overtemp")] = overTemp;
      if (isnan(tempC)) um[F("temp")] = nullptr;
      else              um[F("temp")] = roundf(tempC * 10.0f) / 10.0f;
    }

    void readFromJsonState(JsonObject& root) override {
      if (!initDone) return;
      JsonObject um = root[FPSTR(_name)];
      if (um.isNull()) return;
      if (um[F("enabled")].is<bool>()) reqEnable = um[F("enabled")].as<bool>();
      if (um[F("out1")].is<bool>())    reqOut[0] = um[F("out1")].as<bool>();
      if (um[F("out2")].is<bool>())    reqOut[1] = um[F("out2")].as<bool>();
      if (um[F("bri1")].is<int>())     { int v = um[F("bri1")]; reqBri[0] = v < 0 ? 0 : (v > 100 ? 100 : v); }
      if (um[F("bri2")].is<int>())     { int v = um[F("bri2")]; reqBri[1] = v < 0 ? 0 : (v > 100 ? 100 : v); }
      if (um[F("bt")].is<bool>())      reqBt     = um[F("bt")].as<bool>();
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[F("enabled")]            = enabled;
      top[F("rx-pin")]             = hw.rx;
      top[F("tx-pin")]             = hw.tx;
      top[F("out1-pin")]           = hw.out[0];
      top[F("out2-pin")]           = hw.out[1];
      top[F("active-low")]         = hw.activeLow;
      top[F("dim1-pin")]           = hw.dim[0];
      top[F("dim2-pin")]           = hw.dim[1];
      top[F("dim-inverted")]       = hw.dimInverted;
      top[F("dim-freq")]           = hw.dimFreq;
      top[F("bri1-pct")]           = briDefault[0];
      top[F("bri2-pct")]           = briDefault[1];
      top[F("off-delay-s")]        = offDelaySec;
      top[F("min-frames")]         = minFrames;
      top[F("max-range-mm")]       = maxRangeMm;
      top[F("on-preset")]          = onPreset;
      top[F("off-preset")]         = offPreset;
      top[F("preset-if-off-only")] = presetIfOffOnly;
      top[F("therm-pin")]          = hw.therm;
      top[F("ntc-r25")]            = ntcR25;
      top[F("ntc-beta")]           = ntcBeta;
      top[F("ntc-series-r")]       = ntcSeriesR;
      top[F("ntc-to-gnd")]         = ntcToGnd;
      top[F("temp-limit-c")]       = tempLimitC;
      top[F("ha-discovery")]       = haDiscovery;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) return false;

      HwCfg   nh = hw;
      bool    newEnabled = enabled;
      uint8_t newBri[2]  = { briDefault[0], briDefault[1] };

      bool ok = true;
      ok &= getJsonValue(top[F("enabled")],            newEnabled,      true);
      ok &= getJsonValue(top[F("rx-pin")],             nh.rx,           (int8_t)LD2450_RX_PIN);
      ok &= getJsonValue(top[F("tx-pin")],             nh.tx,           (int8_t)LD2450_TX_PIN);
      ok &= getJsonValue(top[F("out1-pin")],           nh.out[0],       (int8_t)LD2450_OUT1_PIN);
      ok &= getJsonValue(top[F("out2-pin")],           nh.out[1],       (int8_t)LD2450_OUT2_PIN);
      ok &= getJsonValue(top[F("active-low")],         nh.activeLow,    false);
      ok &= getJsonValue(top[F("dim1-pin")],           nh.dim[0],       (int8_t)LD2450_DIM1_PIN);
      ok &= getJsonValue(top[F("dim2-pin")],           nh.dim[1],       (int8_t)LD2450_DIM2_PIN);
      ok &= getJsonValue(top[F("dim-inverted")],       nh.dimInverted,  true);
      ok &= getJsonValue(top[F("dim-freq")],           nh.dimFreq,      (uint16_t)20000);
      ok &= getJsonValue(top[F("bri1-pct")],           newBri[0],       (uint8_t)100);
      ok &= getJsonValue(top[F("bri2-pct")],           newBri[1],       (uint8_t)100);
      ok &= getJsonValue(top[F("off-delay-s")],        offDelaySec,     (uint16_t)60);
      ok &= getJsonValue(top[F("min-frames")],         minFrames,       (uint8_t)2);
      ok &= getJsonValue(top[F("max-range-mm")],       maxRangeMm,      (uint16_t)0);
      ok &= getJsonValue(top[F("on-preset")],          onPreset,        (uint8_t)0);
      ok &= getJsonValue(top[F("off-preset")],         offPreset,       (uint8_t)0);
      ok &= getJsonValue(top[F("preset-if-off-only")], presetIfOffOnly, true);
      ok &= getJsonValue(top[F("therm-pin")],          nh.therm,        (int8_t)LD2450_THERM_PIN);
      ok &= getJsonValue(top[F("ntc-r25")],            ntcR25,          (uint32_t)10000);
      ok &= getJsonValue(top[F("ntc-beta")],           ntcBeta,         (uint16_t)3950);
      ok &= getJsonValue(top[F("ntc-series-r")],       ntcSeriesR,      (uint32_t)10000);
      ok &= getJsonValue(top[F("ntc-to-gnd")],         ntcToGnd,        true);
      ok &= getJsonValue(top[F("temp-limit-c")],       tempLimitC,      (uint8_t)75);
      ok &= getJsonValue(top[F("ha-discovery")],       haDiscovery,     true);

      if (minFrames < 1)      minFrames = 1;
      if (nh.dimFreq < 100)   nh.dimFreq = 100;
      if (nh.dimFreq > 40000) nh.dimFreq = 40000;     // keeps 10-bit resolution valid
      if (!ntcR25)            ntcR25 = 10000;
      if (!ntcSeriesR)        ntcSeriesR = 10000;
      if (!ntcBeta)           ntcBeta = 3950;
      for (uint8_t i = 0; i < 2; i++) if (newBri[i] > 100) newBri[i] = 100;

      if (!initDone) {                 // boot: setup() claims the pins
        enabled = newEnabled;
        hw = nh;
        briDefault[0] = newBri[0]; briDefault[1] = newBri[1];
        return ok;
      }
      // settings saved at runtime (web server task): hand changes to loop()
      if (!nh.same(hw)) { pendHw = nh; reqHw = true; }
      for (uint8_t i = 0; i < 2; i++) {
        if (newBri[i] != briDefault[i]) { briDefault[i] = newBri[i]; reqBri[i] = newBri[i]; }
      }
      reqEnable = newEnabled;
      return ok;
    }

    void appendConfigData(Print& settingsScript) override {
      settingsScript.print(F("addInfo('LD2450:tx-pin',1,'optional, for Bluetooth on/off');"));
      settingsScript.print(F("addInfo('LD2450:out1-pin',1,'XL6013 EN, light 1');"));
      settingsScript.print(F("addInfo('LD2450:out2-pin',1,'XL6013 EN, light 2');"));
      settingsScript.print(F("addInfo('LD2450:dim1-pin',1,'PWM into FB, light 1 (-1 = none)');"));
      settingsScript.print(F("addInfo('LD2450:dim2-pin',1,'PWM into FB, light 2 (-1 = none)');"));
      settingsScript.print(F("addInfo('LD2450:dim-inverted',1,'more PWM = dimmer');"));
      settingsScript.print(F("addInfo('LD2450:dim-freq',1,'Hz, 100-40000');"));
      settingsScript.print(F("addInfo('LD2450:bri1-pct',1,'% at boot');"));
      settingsScript.print(F("addInfo('LD2450:bri2-pct',1,'% at boot');"));
      settingsScript.print(F("addInfo('LD2450:off-delay-s',1,'seconds after last target');"));
      settingsScript.print(F("addInfo('LD2450:min-frames',1,'radar sends ~10/s');"));
      settingsScript.print(F("addInfo('LD2450:max-range-mm',1,'0 = use radar zones only');"));
      settingsScript.print(F("addInfo('LD2450:on-preset',1,'0 = none');"));
      settingsScript.print(F("addInfo('LD2450:off-preset',1,'0 = none');"));
      settingsScript.print(F("addInfo('LD2450:therm-pin',1,'NTC divider, -1 = none');"));
      settingsScript.print(F("addInfo('LD2450:ntc-r25',1,'ohm');"));
      settingsScript.print(F("addInfo('LD2450:ntc-series-r',1,'ohm');"));
      settingsScript.print(F("addInfo('LD2450:ntc-to-gnd',1,'NTC low side, fixed R to 3V3');"));
      settingsScript.print(F("addInfo('LD2450:temp-limit-c',1,'lights cut, back 10 below; 0 = off');"));
    }
};

const char LD2450Presence::_name[] PROGMEM = "LD2450";

static LD2450Presence ld2450_presence;
REGISTER_USERMOD(ld2450_presence);
