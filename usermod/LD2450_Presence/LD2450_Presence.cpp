/*
 * LD2450_Presence - WLED usermod
 *
 * Reads an HLK-LD2450 mmWave radar over UART, derives presence, and:
 *   - switches two GPIO outputs (e.g. picture-light MOSFETs) on presence
 *   - switches them off after a configurable off-delay
 *   - optionally applies a WLED preset on presence / on absence
 *   - publishes presence over MQTT (+ optional Home Assistant discovery)
 *
 * Zones / detection areas are configured on the radar itself (HLKRadarTool
 * app over Bluetooth). The radar then only reports targets inside its zones,
 * so this usermod just has to decide "any target present?".
 *
 * Target: WLED 16.x / 17-dev (custom_usermods build system), ESP32-C3.
 * The radar runs on UART1 (Serial1), so it never collides with WLED's own
 * Serial (USB-CDC on the C3).
 *
 * JSON API (POST /json/state):
 *   {"LD2450":{"enabled":false}}   disable/enable automation (not persisted)
 *   {"LD2450":{"out1":true}}       manual override of output 1 (or out2)
 *                                  - holds until the next presence change
 *   {"LD2450":{"bt":true}}         re-enable the radar's Bluetooth (needs TX wired)
 */

#include "wled.h"

#ifndef LD2450_RX_PIN
  #define LD2450_RX_PIN   20   // ESP RX  <- radar TX
#endif
#ifndef LD2450_TX_PIN
  #define LD2450_TX_PIN   21   // ESP TX  -> radar RX (only needed for the Bluetooth command)
#endif
#ifndef LD2450_OUT1_PIN
  #define LD2450_OUT1_PIN 4
#endif
#ifndef LD2450_OUT2_PIN
  #define LD2450_OUT2_PIN 5
#endif

#define LD2450_BAUD        256000
#define LD2450_FRAME_LEN   30      // AA FF 03 00 | 3 x 8-byte targets | 55 CC
#define LD2450_TIMEOUT_MS  5000    // no valid frame for this long = radar offline
#define LD2450_CMD_GAP_MS  150     // spacing between radar config commands
#define LD2450_RX_BUDGET   128     // max bytes parsed per loop() pass

class LD2450Presence : public Usermod {
  private:
    // ---- configuration (cfg.json / Usermod settings page) ----
    bool     enabled         = true;
    int8_t   rxPin           = LD2450_RX_PIN;
    int8_t   txPin           = LD2450_TX_PIN;
    int8_t   outPin[2]       = { LD2450_OUT1_PIN, LD2450_OUT2_PIN };
    bool     activeLow       = false;  // invert both outputs
    uint16_t offDelaySec     = 60;     // seconds without a target before switching off
    uint8_t  minFrames       = 2;      // consecutive target frames before switching on (~10 frames/s)
    uint16_t maxRangeMm      = 0;      // extra range limit on top of the radar's zones, 0 = off
    uint8_t  onPreset        = 0;      // 0 = none
    uint8_t  offPreset       = 0;      // 0 = none
    bool     presetIfOffOnly = true;   // only apply on-preset if WLED is off; then off-preset only undoes our own
    bool     haDiscovery     = true;

    // ---- runtime ----
    bool     initDone    = false;
    bool     uartActive  = false;
    uint8_t  frame[LD2450_FRAME_LEN];
    uint8_t  frameIdx    = 0;
    uint32_t lastFrameMs = 0;
    uint32_t lastSeenMs  = 0;
    uint8_t  hitStreak   = 0;          // consecutive frames with a target, 0 = latest frame empty
    uint8_t  targetCount = 0;
    uint16_t nearestMm   = 0;
    bool     present     = false;
    bool     outState[2] = { false, false };
    bool     presetOwned = false;      // we applied the on-preset, so the off-preset is ours to apply

    // requests from the JSON API: the web server runs on another task,
    // so it only sets these and loop() acts on them
    volatile int8_t reqEnable = -1;
    volatile int8_t reqOut[2] = { -1, -1 };
    volatile int8_t reqBt     = -1;
    volatile bool   reqHw     = false;   // pin / polarity change from the settings page
    int8_t pendRx = -1, pendTx = -1, pendOut[2] = { -1, -1 };
    bool   pendActiveLow = false;

    // non-blocking radar command sequence (Bluetooth on/off)
    uint8_t  btStep   = 0;
    bool     btTarget = false;
    uint32_t btNextMs = 0;

    static const char _name[];

    // ---------------------------------------------------------------------

    static int16_t decodeSigned(uint16_t raw) {
      // LD2450 encoding: bit 15 set = positive, clear = negative; bits 0-14 = magnitude
      int16_t v = raw & 0x7FFF;
      return (raw & 0x8000) ? v : -v;
    }

    bool radarOnline(uint32_t now) const {
      return uartActive && lastFrameMs != 0 && (now - lastFrameMs) < LD2450_TIMEOUT_MS;
    }

    void writeOutput(uint8_t i, bool on) {
      outState[i] = on;
      if (outPin[i] >= 0) digitalWrite(outPin[i], (on != activeLow) ? HIGH : LOW);
    }

    void initHardware() {
      for (uint8_t i = 0; i < 2; i++) {
        if (outPin[i] < 0) continue;
        if (PinManager::allocatePin(outPin[i], true, PinOwner::UM_Unspecified)) {
          pinMode(outPin[i], OUTPUT);
          writeOutput(i, outState[i]);
        } else {
          DEBUG_PRINTF_P(PSTR("LD2450: output pin %d unavailable\n"), outPin[i]);
          outPin[i] = -1;
        }
      }

      if (rxPin >= 0 && PinManager::allocatePin(rxPin, false, PinOwner::UM_Unspecified)) {
        if (txPin >= 0 && !PinManager::allocatePin(txPin, true, PinOwner::UM_Unspecified)) {
          DEBUG_PRINTF_P(PSTR("LD2450: TX pin %d unavailable\n"), txPin);
          txPin = -1;
        }
        Serial1.setRxBufferSize(256);
        Serial1.begin(LD2450_BAUD, SERIAL_8N1, rxPin, txPin);
        uartActive = true;
      } else {
        if (rxPin >= 0) { DEBUG_PRINTF_P(PSTR("LD2450: RX pin %d unavailable\n"), rxPin); }
        rxPin = -1;
      }
      frameIdx    = 0;
      lastFrameMs = 0;
      hitStreak   = 0;
    }

    void releaseHardware() {
      if (uartActive) {
        Serial1.end();
        uartActive = false;
      }
      if (rxPin >= 0) PinManager::deallocatePin(rxPin, PinOwner::UM_Unspecified);
      if (txPin >= 0) PinManager::deallocatePin(txPin, PinOwner::UM_Unspecified);
      for (uint8_t i = 0; i < 2; i++) {
        if (outPin[i] < 0) continue;
        digitalWrite(outPin[i], activeLow ? HIGH : LOW);  // leave the load off
        PinManager::deallocatePin(outPin[i], PinOwner::UM_Unspecified);
      }
    }

    void setPresence(bool p) {
      if (p == present) return;
      present = p;
      writeOutput(0, p);
      writeOutput(1, p);
      publishPresence();

      if (p) {
        if (onPreset && (!presetIfOffOnly || bri == 0)) {
          applyPreset(onPreset, CALL_MODE_BUTTON_PRESET);
          presetOwned = true;
        }
      } else {
        if (offPreset && (!presetIfOffOnly || presetOwned)) {
          applyPreset(offPreset, CALL_MODE_BUTTON_PRESET);
        }
        presetOwned = false;
      }
    }

    void setEnabled(bool en) {
      if (en == enabled) return;
      enabled   = en;
      hitStreak = 0;
      if (!en) {
        // stop automation and leave the outputs off; WLED's LEDs are left alone
        present     = false;
        presetOwned = false;
        writeOutput(0, false);
        writeOutput(1, false);
        publishPresence();
      }
    }

    // ---- radar UART ------------------------------------------------------

    void readRadar(uint32_t now) {
      if (!uartActive) return;
      static const uint8_t header[4] = { 0xAA, 0xFF, 0x03, 0x00 };
      uint16_t budget = LD2450_RX_BUDGET;

      while (budget-- && Serial1.available()) {
        uint8_t b = Serial1.read();

        if (frameIdx < 4) {                       // hunting for the header
          if (b == header[frameIdx]) {
            frame[frameIdx++] = b;
          } else if (b == header[0]) {
            frame[0] = b;
            frameIdx = 1;
          } else {
            frameIdx = 0;
          }
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
        const uint8_t* p = frame + 4 + t * 8;   // x(2) y(2) speed(2) resolution(2), little-endian

        bool empty = true;
        for (uint8_t i = 0; i < 8; i++) if (p[i]) { empty = false; break; }
        if (empty) continue;                     // unused target slot is all zeros

        int16_t x = decodeSigned(p[0] | (p[1] << 8));   // mm
        int16_t y = decodeSigned(p[2] | (p[3] << 8));   // mm
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

    // LD2450 command frame: FD FC FB FA | len(2) | cmd(2) | value | 04 03 02 01
    void sendCommand(uint16_t cmd, const uint8_t* value, uint8_t len) {
      if (!uartActive || txPin < 0) return;
      static const uint8_t head[4] = { 0xFD, 0xFC, 0xFB, 0xFA };
      static const uint8_t tail[4] = { 0x04, 0x03, 0x02, 0x01 };
      uint16_t dataLen = 2 + len;
      Serial1.write(head, 4);
      Serial1.write((uint8_t)(dataLen & 0xFF));
      Serial1.write((uint8_t)(dataLen >> 8));
      Serial1.write((uint8_t)(cmd & 0xFF));
      Serial1.write((uint8_t)(cmd >> 8));
      if (len) Serial1.write(value, len);
      Serial1.write(tail, 4);
    }

    // enable config mode -> set Bluetooth -> restart (the setting applies after restart)
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
        reqHw  = false;
        btStep = 0;
        releaseHardware();               // still uses the old pins and polarity
        rxPin = pendRx; txPin = pendTx; outPin[0] = pendOut[0]; outPin[1] = pendOut[1];
        activeLow = pendActiveLow;
        initHardware();
      }
      if (reqEnable >= 0) { setEnabled(reqEnable); reqEnable = -1; }
      for (uint8_t i = 0; i < 2; i++) {
        if (reqOut[i] >= 0) { writeOutput(i, reqOut[i]); reqOut[i] = -1; }
      }
      if (reqBt >= 0) {
        if (uartActive && txPin >= 0 && !btStep) {
          btTarget = reqBt;
          btStep   = 1;
          btNextMs = now;
        }
        reqBt = -1;
      }
    }

    // ---- MQTT ------------------------------------------------------------

    void publishPresence() {
#ifndef WLED_DISABLE_MQTT
      if (!WLED_MQTT_CONNECTED) return;
      char topic[MQTT_MAX_TOPIC_LEN + 16];
      snprintf_P(topic, sizeof(topic), PSTR("%s/presence"), mqttDeviceTopic);
      mqtt->publish(topic, 0, true, present ? "on" : "off");
#endif
    }

    void publishDiscovery() {
#ifndef WLED_DISABLE_MQTT
      if (!WLED_MQTT_CONNECTED) return;
      StaticJsonDocument<640> doc;
      char uid[40], buf[96], json[640];

      snprintf_P(uid, sizeof(uid), PSTR("%s_ld2450"), escapedMac.c_str());
      snprintf_P(buf, sizeof(buf), PSTR("%s Presence"), serverDescription);
      doc[F("name")]    = buf;
      doc[F("uniq_id")] = uid;
      doc[F("dev_cla")] = F("occupancy");
      doc[F("pl_on")]   = "on";
      doc[F("pl_off")]  = "off";
      snprintf_P(buf, sizeof(buf), PSTR("%s/presence"), mqttDeviceTopic);
      doc[F("stat_t")]  = buf;
      doc[F("avty_t")]  = mqttStatusTopic;   // WLED's LWT: "online"/"offline"

      JsonObject device = doc.createNestedObject(F("device"));
      device[F("name")] = serverDescription;
      device[F("ids")]  = String(F("wled-sensor-")) + mqttClientID;
      device[F("mf")]   = F(WLED_BRAND);
      device[F("mdl")]  = F(WLED_PRODUCT_NAME);
      device[F("sw")]   = versionString;

      size_t n = serializeJson(doc, json, sizeof(json));
      snprintf_P(buf, sizeof(buf), PSTR("homeassistant/binary_sensor/%s/config"), uid);
      mqtt->publish(buf, 0, true, json, n);
#endif
    }

  public:
    void setup() override {
      initHardware();
      initDone = true;
    }

    void loop() override {
      if (!initDone) return;
      uint32_t now = millis();

      handleRequests(now);
      readRadar(now);
      runBtSequence(now);

      if (!radarOnline(now)) {   // no data: treat as "no target" so the off-delay can run out
        hitStreak   = 0;
        targetCount = 0;
        nearestMm   = 0;
      }
      if (enabled && present && hitStreak == 0 &&
          (now - lastSeenMs) >= (uint32_t)offDelaySec * 1000UL) {
        setPresence(false);
      }
    }

    void onMqttConnect(bool sessionPresent) override {
      publishPresence();
      if (haDiscovery) publishDiscovery();
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      JsonArray info = user.createNestedArray(FPSTR(_name));

      uint32_t now = millis();
      String s;
      if (!enabled)                s = F("disabled");
      else if (!uartActive)        s = F("no RX pin");
      else if (!radarOnline(now))  s = F("radar offline");
      else if (present && hitStreak == 0) {
        uint32_t delayMs = (uint32_t)offDelaySec * 1000UL, since = now - lastSeenMs;
        uint32_t left    = since < delayMs ? delayMs - since : 0;
        s = F("off in "); s += (left + 999) / 1000; s += F("s");
      }
      else if (present)            { s = F("present, "); s += (int)targetCount; s += F(" tgt @ "); s += (int)nearestMm; s += F(" mm"); }
      else                         s = F("clear");
      info.add(s);

      String btn = F(" <button class=\"btn btn-xs\" onclick=\"requestJson({");
      btn += FPSTR(_name);
      btn += enabled ? F(":{enabled:false}});\"><i class=\"icons on\">&#xe325;</i>")
                     : F(":{enabled:true}});\"><i class=\"icons off\">&#xe08f;</i>");
      btn += F("</button>");
      info.add(btn);

      // one row per output: state, GPIO, and a toggle (manual override until the next presence change)
      for (uint8_t i = 0; i < 2; i++) {
        char key[16];
        snprintf_P(key, sizeof(key), PSTR("LD2450 Output %d"), i + 1);   // char[] key is copied by ArduinoJson
        JsonArray row = user.createNestedArray(key);
        if (outPin[i] < 0) { row.add(F("no pin set")); continue; }
        String o = outState[i] ? F("on") : F("off");
        o += F(" (GPIO "); o += (int)outPin[i]; o += F(")");
        o += F(" <button class=\"btn btn-xs\" onclick=\"requestJson({");
        o += FPSTR(_name);
        o += F(":{out"); o += (int)(i + 1);
        o += outState[i] ? F(":false}});\"><i class=\"icons on\">&#xe08f;</i>")
                         : F(":true}});\"><i class=\"icons off\">&#xe08f;</i>");
        o += F("</button>");
        row.add(o);
      }

      if (txPin >= 0 && uartActive) {
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
    }

    void readFromJsonState(JsonObject& root) override {
      if (!initDone) return;
      JsonObject um = root[FPSTR(_name)];
      if (um.isNull()) return;
      if (um[F("enabled")].is<bool>()) reqEnable = um[F("enabled")].as<bool>();
      if (um[F("out1")].is<bool>())    reqOut[0] = um[F("out1")].as<bool>();
      if (um[F("out2")].is<bool>())    reqOut[1] = um[F("out2")].as<bool>();
      if (um[F("bt")].is<bool>())      reqBt     = um[F("bt")].as<bool>();
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[F("enabled")]            = enabled;
      top[F("rx-pin")]             = rxPin;
      top[F("tx-pin")]             = txPin;
      top[F("out1-pin")]           = outPin[0];
      top[F("out2-pin")]           = outPin[1];
      top[F("active-low")]         = activeLow;
      top[F("off-delay-s")]        = offDelaySec;
      top[F("min-frames")]         = minFrames;
      top[F("max-range-mm")]       = maxRangeMm;
      top[F("on-preset")]          = onPreset;
      top[F("off-preset")]         = offPreset;
      top[F("preset-if-off-only")] = presetIfOffOnly;
      top[F("ha-discovery")]       = haDiscovery;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) return false;   // first boot: defaults are used and written out

      int8_t newRx = rxPin, newTx = txPin, newOut[2] = { outPin[0], outPin[1] };
      bool   newActiveLow = activeLow, newEnabled = enabled;

      bool ok = true;
      ok &= getJsonValue(top[F("enabled")],            newEnabled,      true);
      ok &= getJsonValue(top[F("rx-pin")],             newRx,           (int8_t)LD2450_RX_PIN);
      ok &= getJsonValue(top[F("tx-pin")],             newTx,           (int8_t)LD2450_TX_PIN);
      ok &= getJsonValue(top[F("out1-pin")],           newOut[0],       (int8_t)LD2450_OUT1_PIN);
      ok &= getJsonValue(top[F("out2-pin")],           newOut[1],       (int8_t)LD2450_OUT2_PIN);
      ok &= getJsonValue(top[F("active-low")],         newActiveLow,    false);
      ok &= getJsonValue(top[F("off-delay-s")],        offDelaySec,     (uint16_t)60);
      ok &= getJsonValue(top[F("min-frames")],         minFrames,       (uint8_t)2);
      ok &= getJsonValue(top[F("max-range-mm")],       maxRangeMm,      (uint16_t)0);
      ok &= getJsonValue(top[F("on-preset")],          onPreset,        (uint8_t)0);
      ok &= getJsonValue(top[F("off-preset")],         offPreset,       (uint8_t)0);
      ok &= getJsonValue(top[F("preset-if-off-only")], presetIfOffOnly, true);
      ok &= getJsonValue(top[F("ha-discovery")],       haDiscovery,     true);

      if (minFrames < 1) minFrames = 1;

      if (!initDone) {                   // boot: setup() will claim the pins
        enabled = newEnabled;
        rxPin = newRx; txPin = newTx; outPin[0] = newOut[0]; outPin[1] = newOut[1];
        activeLow = newActiveLow;
        return ok;
      }

      // settings saved at runtime: this runs on the web server task, so hand
      // pin/UART changes to loop() instead of touching Serial1 from here
      if (newRx != rxPin || newTx != txPin || newOut[0] != outPin[0] ||
          newOut[1] != outPin[1] || newActiveLow != activeLow) {
        pendRx = newRx; pendTx = newTx; pendOut[0] = newOut[0]; pendOut[1] = newOut[1];
        pendActiveLow = newActiveLow;
        reqHw = true;
      }
      reqEnable = newEnabled;
      return ok;
    }

    void appendConfigData(Print& settingsScript) override {
      settingsScript.print(F("addInfo('LD2450:off-delay-s',1,'seconds after last target');"));
      settingsScript.print(F("addInfo('LD2450:min-frames',1,'radar sends ~10/s');"));
      settingsScript.print(F("addInfo('LD2450:max-range-mm',1,'0 = use radar zones only');"));
      settingsScript.print(F("addInfo('LD2450:on-preset',1,'0 = none');"));
      settingsScript.print(F("addInfo('LD2450:off-preset',1,'0 = none');"));
      settingsScript.print(F("addInfo('LD2450:tx-pin',1,'optional, for Bluetooth on/off');"));
    }
};

const char LD2450Presence::_name[] PROGMEM = "LD2450";

static LD2450Presence ld2450_presence;
REGISTER_USERMOD(ld2450_presence);
