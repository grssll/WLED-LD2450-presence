// Host-side logic tests: feeds synthetic LD2450 frames through the usermod and checks behaviour.
#include "../usermod/LD2450_Presence/LD2450_Presence.cpp"
#include <cassert>
#include <iostream>
int g_pin[64]; uint32_t g_now = 1000; byte bri = 0; HardwareSerial Serial1;
std::vector<int> presets;
bool applyPreset(byte i, byte) { presets.push_back(i); return true; }
bool PinManager::allocatePin(byte, bool, PinOwner) { return true; }
bool PinManager::deallocatePin(byte, PinOwner) { return true; }
std::vector<std::pair<std::string,std::string>> pubs;
bool AsyncMqttClient::connected() { return true; }
uint16_t AsyncMqttClient::publish(const char* t, uint8_t, bool, const char* p, size_t n) { pubs.push_back({t, n ? std::string(p, n) : std::string(p)}); return 1; }
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
static void tick(uint32_t ms, int targets, bool frames = true) {   // advance time at ~10 frames/s
  for (uint32_t t = 0; t < ms; t += 100) { g_now += 100; if (frames) sendFrame(targets); ld2450_presence.loop(); }
}
static bool presence() {
  StaticJsonDocument<512> d; JsonObject r = d.to<JsonObject>(); ld2450_presence.addToJsonState(r);
  return r["LD2450"]["presence"];
}
static void configure(const char* json) {
  StaticJsonDocument<1024> d; deserializeJson(d, json); JsonObject r = d.as<JsonObject>(); ld2450_presence.readFromConfig(r);
}
int main() {
  configure(R"({"LD2450":{"enabled":true,"rx-pin":20,"tx-pin":21,"out1-pin":4,"out2-pin":5,"active-low":false,
    "off-delay-s":10,"min-frames":2,"max-range-mm":0,"on-preset":3,"off-preset":4,"preset-if-off-only":true,"ha-discovery":true}})");
  ld2450_presence.setup();
  assert(Serial1.open);

  // garbage + a split frame must still parse; a single target frame must NOT switch on (min-frames 2)
  for (uint8_t b : {0x12, 0xAA, 0x00, 0xAA, 0xFF}) Serial1.rx.push_back(b);
  Serial1.rx.clear(); sendFrame(1); g_now += 100; ld2450_presence.loop();
  assert(!presence() && g_pin[4] == 0);
  tick(100, 0);                                       // empty frame resets the streak
  tick(100, 1); assert(!presence());
  tick(100, 1); assert(presence() && g_pin[4] == 1 && g_pin[5] == 1);
  assert(presets.size() == 1 && presets[0] == 3);     // on-preset applied (WLED was off)
  std::cout << "on after 2 frames, on-preset applied\n";

  // off-delay 10 s: still on at 9.9 s, off at 10 s, off-preset applied
  tick(9900, 0); assert(presence());
  tick(200, 0);  assert(!presence() && g_pin[4] == 0 && g_pin[5] == 0);
  assert(presets.size() == 2 && presets[1] == 4);
  std::cout << "off after off-delay, off-preset applied\n";

  // presence returning during the off-delay cancels it
  tick(300, 1); assert(presence());
  tick(5000, 0); tick(200, 1); tick(9000, 0); assert(presence());
  tick(1200, 0); assert(!presence());
  std::cout << "re-detection restarts the off-delay\n";

  // WLED already on -> no on-preset, and no off-preset either
  presets.clear(); bri = 128;
  tick(300, 1); assert(presence()); tick(10200, 0); assert(!presence()); assert(presets.empty());
  bri = 0;
  std::cout << "presets skipped when WLED was already on\n";

  // radar goes silent while present -> times out and switches off
  tick(300, 1); assert(presence());
  tick(12000, 0, false); assert(!presence());
  std::cout << "radar offline -> off\n";

  // max-range filter: target at 3 m ignored with a 2 m limit
  configure(R"({"LD2450":{"enabled":true,"rx-pin":20,"tx-pin":21,"out1-pin":4,"out2-pin":5,"active-low":false,
    "off-delay-s":10,"min-frames":2,"max-range-mm":2000,"on-preset":0,"off-preset":0,"preset-if-off-only":true,"ha-discovery":true}})");
  for (int i = 0; i < 5; i++) { g_now += 100; sendFrame(1, 0, 3000); ld2450_presence.loop(); }
  assert(!presence());
  for (int i = 0; i < 3; i++) { g_now += 100; sendFrame(1, -400, 1500); ld2450_presence.loop(); }
  assert(presence());
  std::cout << "max-range filter + negative X decode\n";

  // active-low change via settings is applied in loop(); output polarity follows
  configure(R"({"LD2450":{"enabled":true,"rx-pin":20,"tx-pin":21,"out1-pin":4,"out2-pin":5,"active-low":true,
    "off-delay-s":10,"min-frames":2,"max-range-mm":2000,"on-preset":0,"off-preset":0,"preset-if-off-only":true,"ha-discovery":true}})");
  tick(100, 1); assert(presence() && g_pin[4] == 0);
  std::cout << "active-low applied at runtime\n";

  // disable via JSON state -> outputs off; enable again
  { StaticJsonDocument<128> d; deserializeJson(d, R"({"LD2450":{"enabled":false}})"); JsonObject r = d.as<JsonObject>(); ld2450_presence.readFromJsonState(r); }
  tick(100, 1); assert(!presence() && g_pin[4] == 1);
  std::cout << "disable via JSON -> outputs off\n";

  // Bluetooth sequence bytes
  { StaticJsonDocument<128> d; deserializeJson(d, R"({"LD2450":{"enabled":true,"bt":false}})"); JsonObject r = d.as<JsonObject>(); ld2450_presence.readFromJsonState(r); }
  Serial1.tx.clear(); tick(600, 0);
  std::vector<uint8_t> expect = {
    0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01,
    0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xA4,0x00, 0x00,0x00, 0x04,0x03,0x02,0x01,
    0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xA3,0x00,            0x04,0x03,0x02,0x01 };
  assert(Serial1.tx == expect);
  std::cout << "bluetooth-off command sequence bytes correct\n";
  pubs.clear(); ld2450_presence.onMqttConnect(false);
  assert(pubs.size() == 2 && pubs[0].first == "wled/slat/presence");
  assert(pubs[1].first == "homeassistant/binary_sensor/aabbcc_ld2450/config");
  std::cout << "MQTT: " << pubs[0].first << " = " << pubs[0].second << "\n";
  std::cout << "discovery: " << pubs[1].second << "\n";
  std::cout << "ALL TESTS PASSED\n";
}
