// Host-side logic tests: feeds synthetic LD2450 frames, ADC readings and MQTT/JSON
// commands through the usermod and checks its behaviour. Run by CI before the build.
#include "../usermod/LD2450_Presence/LD2450_Presence.cpp"
#include <cassert>
#include <iostream>
#include <map>

int g_pin[64]; uint32_t g_now = 1000; byte bri = 0; HardwareSerial Serial1;
uint32_t g_duty[8]; uint32_t g_mv = 1650; std::map<uint8_t,uint8_t> g_ledcPin;
std::vector<int> presets;
bool applyPreset(byte i, byte) { presets.push_back(i); return true; }
bool PinManager::allocatePin(byte, bool, PinOwner) { return true; }
bool PinManager::deallocatePin(byte, PinOwner) { return true; }
byte PinManager::allocateLedc(byte) { return 0; }
void PinManager::deallocateLedc(byte, byte) {}
uint32_t ledcSetup(uint8_t, uint32_t f, uint8_t) { return f; }
void ledcAttachPin(uint8_t pin, uint8_t ch) { g_ledcPin[ch] = pin; }
void ledcDetachPin(uint8_t) {}
void ledcWrite(uint8_t ch, uint32_t d) { g_duty[ch] = d; }
uint32_t analogReadMilliVolts(uint8_t) { return g_mv; }

std::vector<std::pair<std::string,std::string>> pubs; std::vector<std::string> subs;
bool AsyncMqttClient::connected() { return true; }
uint16_t AsyncMqttClient::publish(const char* t, uint8_t, bool, const char* p, size_t n) { pubs.push_back({t, n ? std::string(p, n) : std::string(p)}); return 1; }
uint16_t AsyncMqttClient::subscribe(const char* t, uint8_t) { subs.push_back(t); return 1; }
AsyncMqttClient mqttObj; AsyncMqttClient* mqtt = &mqttObj;
char mqttDeviceTopic[33] = "wled/slat", mqttStatusTopic[40] = "wled/slat/status", mqttClientID[41] = "x", serverDescription[33] = "Slat", versionString[] = "16";
String escapedMac("aabbcc");

static void enc(std::vector<uint8_t>& f, int v) { uint16_t r = v >= 0 ? (0x8000 | v) : (-v); f.push_back(r & 0xFF); f.push_back(r >> 8); }
static void sendFrame(int nTargets, int x = 300, int y = 1200) {
  std::vector<uint8_t> f = {0xAA, 0xFF, 0x03, 0x00};
  for (int t = 0; t < 3; t++) {
    if (t < nTargets) { enc(f, x); enc(f, y); enc(f, 0); f.push_back(0x68); f.push_back(0x01); }
    else for (int i = 0; i < 8; i++) f.push_back(0);
  }
  f.push_back(0x55); f.push_back(0xCC);
  for (auto b : f) Serial1.rx.push_back(b);
}
static void tick(uint32_t ms, int targets, bool frames = true) {   // ~10 frames/s
  for (uint32_t t = 0; t < ms; t += 100) { g_now += 100; if (frames) sendFrame(targets); ld2450_presence.loop(); }
}
static DynamicJsonDocument state() {
  DynamicJsonDocument d(1024); JsonObject r = d.to<JsonObject>(); ld2450_presence.addToJsonState(r); return d;
}
static bool presence() { return state()["LD2450"]["presence"]; }
static void configure(const char* json) {
  DynamicJsonDocument d(2048); deserializeJson(d, json); JsonObject r = d.as<JsonObject>(); ld2450_presence.readFromConfig(r);
}
static void jsonState(const char* json) {
  DynamicJsonDocument d(256); deserializeJson(d, json); JsonObject r = d.as<JsonObject>(); ld2450_presence.readFromJsonState(r);
}
static void mqttMsg(const char* topic, const char* payload) {
  std::string t(topic), p(payload); assert(ld2450_presence.onMqttMessage(&t[0], &p[0]));
}
static bool published(const std::string& topic, const std::string& val) {
  for (auto& p : pubs) { if (p.first == topic && p.second == val) return true; }
  return false;
}
static uint32_t mvFor(float c) {   // 10k NTC (B3950) to GND, 10k to 3.3 V
  float r = 10000.0f * expf(3950.0f * (1.0f / (c + 273.15f) - 1.0f / 298.15f));
  return (uint32_t)lroundf(3300.0f * r / (10000.0f + r));
}

int main() {
  // only the non-pin settings are given, so every pin is the compiled-in default
  configure(R"({"LD2450":{"off-delay-s":10,"on-preset":3,"off-preset":4}})");
  { DynamicJsonDocument d(2048); JsonObject r = d.to<JsonObject>(); ld2450_presence.addToConfig(r);
    JsonObject c = r["LD2450"];
    assert(c["rx-pin"] == 7 && c["tx-pin"] == 10 && c["out1-pin"] == 4 && c["out2-pin"] == 5);
    assert(c["dim1-pin"] == 3 && c["dim2-pin"] == 1 && c["therm-pin"] == 0);
    assert(c["dim-inverted"] == true && c["temp-limit-c"] == 75); }
  std::cout << "default pins: radar 7/10, EN 4/5, DIM 3/1, NTC 0 (GPIO20/21 untouched)\n";

  ld2450_presence.setup();
  assert(Serial1.open && g_ledcPin[0] == 3 && g_ledcPin[1] == 1);
  assert(g_duty[0] == 0 && g_duty[1] == 0);        // 100 % inverted = 0 % PWM = rated current
  std::cout << "dim PWM on LEDC 0/1, 100 % = 0 % duty\n";

  // ---- presence (unchanged behaviour) ----
  sendFrame(1); g_now += 100; ld2450_presence.loop();
  assert(!presence() && g_pin[4] == 0);
  tick(100, 0); tick(100, 1); assert(!presence());
  tick(100, 1); assert(presence() && g_pin[4] == 1 && g_pin[5] == 1);
  assert(presets.size() == 1 && presets[0] == 3);
  tick(9900, 0); assert(presence());
  tick(200, 0);  assert(!presence() && g_pin[4] == 0 && presets.back() == 4);
  tick(300, 1); assert(presence());
  tick(5000, 0); tick(200, 1); tick(9000, 0); assert(presence());
  tick(1200, 0); assert(!presence());
  presets.clear(); bri = 128;
  tick(300, 1); tick(10200, 0); assert(!presence() && presets.empty());
  bri = 0;
  tick(300, 1); assert(presence()); tick(12000, 0, false); assert(!presence());
  std::cout << "presence, off-delay, presets, radar-offline: ok\n";

  // ---- dimming ----
  jsonState(R"({"LD2450":{"bri1":40}})"); tick(100, 0);
  assert(g_duty[0] == 1023 - 409 && (int)state()["LD2450"]["bri1"] == 40);
  jsonState(R"({"LD2450":{"bri1":0,"bri2":250}})"); tick(100, 0);
  assert(g_duty[0] == 1023 && g_duty[1] == 0);      // 0 % = full PWM, >100 clamps to 100
  std::cout << "dimming: 40 % -> duty 614, 0 % -> 1023, clamp ok\n";

  // ---- MQTT / Home Assistant ----
  pubs.clear(); subs.clear();
  ld2450_presence.onMqttConnect(false); tick(100, 0);
  assert(subs.size() == 4 && subs[0] == "wled/slat/light1/set" && subs[1] == "wled/slat/light1/bri/set");
  assert(published("wled/slat/light1/state", "OFF") && published("wled/slat/light1/bri", "0"));
  int cfgs = 0; std::string lightCfg;
  for (auto& p : pubs) if (p.first.rfind("homeassistant/", 0) == 0) {
    cfgs++; if (p.first == "homeassistant/light/aabbcc_ld2450_l1/config") lightCfg = p.second; }
  assert(cfgs == 4 && published("homeassistant/binary_sensor/aabbcc_ld2450/config", pubs[0].second));
  assert(lightCfg.find("\"bri_scl\":100") != std::string::npos && lightCfg.find("\"cmd_t\":\"wled/slat/light1/set\"") != std::string::npos);
  std::cout << "HA discovery: presence + 2 dimmable lights + temperature\n";

  tick(300, 1); assert(presence() && g_pin[4] == 1);
  pubs.clear();
  mqttMsg("/light1/set", "OFF"); mqttMsg("/light2/bri/set", "25"); tick(100, 1);
  assert(g_pin[4] == 0 && g_pin[5] == 1 && g_duty[1] == 1023 - 256);
  assert(published("wled/slat/light1/state", "OFF") && published("wled/slat/light2/bri", "25"));
  mqttMsg("/light1/set", "ON"); tick(100, 1); assert(g_pin[4] == 1);
  std::cout << "MQTT light on/off + brightness commands ok\n";

  // ---- thermistor / over-temperature ----
  g_mv = 1650; tick(1100, 1);
  float t = state()["LD2450"]["temp"]; assert(t > 24.9f && t < 25.1f);
  g_mv = mvFor(80); tick(1100, 1);
  assert(state()["LD2450"]["overtemp"] == true && g_pin[4] == 0 && g_pin[5] == 0);
  assert((bool)state()["LD2450"]["out1"]);          // still wanted, just held off
  g_mv = mvFor(70); tick(1100, 1); assert(g_pin[4] == 0);   // inside hysteresis
  g_mv = mvFor(64); tick(1100, 1);
  assert(state()["LD2450"]["overtemp"] == false && g_pin[4] == 1 && g_pin[5] == 1);
  g_mv = 0; tick(1100, 1); assert(state()["LD2450"]["temp"].isNull() && g_pin[4] == 1);
  g_mv = 1650; tick(1100, 1);
  std::cout << "NTC: 25.0 C, cut at 80, held at 70, back at 64, fault = null\n";

  // ---- runtime pin/polarity change from the settings page ----
  configure(R"({"LD2450":{"off-delay-s":10,"dim-inverted":false,"bri1-pct":60,"bri2-pct":100}})");
  tick(100, 1);
  // light 1: boot value changed to 60 -> applied now; light 2: boot value unchanged,
  // so its runtime 25 % (from the MQTT command above) is kept, now non-inverted
  assert(g_duty[0] == (60u * 1023 + 50) / 100 && g_duty[1] == (25u * 1023 + 50) / 100);
  std::cout << "settings change applied in loop(): non-inverted dim, changed boot brightness\n";

  // ---- disable + Bluetooth bytes ----
  jsonState(R"({"LD2450":{"enabled":false}})"); tick(100, 1);
  assert(!presence() && g_pin[4] == 0);
  jsonState(R"({"LD2450":{"enabled":true,"bt":false}})");
  Serial1.tx.clear(); tick(600, 0);
  std::vector<uint8_t> expect = {
    0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01,
    0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xA4,0x00, 0x00,0x00, 0x04,0x03,0x02,0x01,
    0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xA3,0x00,            0x04,0x03,0x02,0x01 };
  assert(Serial1.tx == expect);
  std::cout << "disable + Bluetooth command bytes ok\nALL TESTS PASSED\n";
}
