// Host-side test stub, NOT the real wled.h. Used only by test/test.cpp in CI.
// Host-side stub of the WLED APIs this usermod touches (signatures copied from WLED main)
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
using std::isnan;  // Arduino exposes isnan() globally
#include <string>
#include <strings.h>
typedef uint8_t byte;
#define PROGMEM
#define PSTR(s) (s)
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
#define FPSTR(p) (reinterpret_cast<const __FlashStringHelper*>(p))
#define snprintf_P snprintf
#define DEBUG_PRINTF_P(...) do {} while (0)
#define DEBUG_PRINTLN(...) do {} while (0)
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define SERIAL_8N1 0x800001c
inline void pinMode(uint8_t, uint8_t) {}
extern int g_pin[64];
inline void digitalWrite(uint8_t p, uint8_t v) { g_pin[p] = v; }
extern uint32_t g_now;
inline uint32_t millis() { return g_now; }
struct String : std::string {
  String() {}
  String(const char* s) : std::string(s) {}
  String(const __FlashStringHelper* s) : std::string((const char*)s) {}
  String& operator=(const __FlashStringHelper* s) { assign((const char*)s); return *this; }
  String& operator+=(const __FlashStringHelper* s) { append((const char*)s); return *this; }
  String& operator+=(const char* s) { append(s); return *this; }
  String& operator+=(const String& s) { append(s); return *this; }
  String& operator+=(unsigned long v) { append(std::to_string(v)); return *this; }
  String& operator+=(unsigned int v) { append(std::to_string(v)); return *this; }
  String& operator+=(int v) { append(std::to_string(v)); return *this; }
  String& operator+=(uint16_t v) { append(std::to_string(v)); return *this; }
  String& operator+=(uint8_t v) { append(std::to_string(v)); return *this; }
  friend String operator+(const String& a, const char* b) { String r(a); r.append(b); return r; }
  const char* c_str() const { return std::string::c_str(); }
  unsigned int length() const { return (unsigned int)size(); }
  bool concat(const char* s) { append(s); return true; }
  bool concat(char c) { push_back(c); return true; }
};
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
struct StringSumHelper : String {};
#define ARDUINOJSON_ENABLE_PROGMEM 1
#define ARDUINOJSON_ENABLE_STD_STRING 1
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define pgm_read_ptr(p) (*(void* const*)(p))
#define pgm_read_float(p) (*(const float*)(p))
#define pgm_read_dword(p) (*(const uint32_t*)(p))
#define memcpy_P memcpy
#define strlen_P strlen
#define strcmp_P strcmp
#define strncmp_P strncmp
#include "ArduinoJson-v6.h"
struct Print { size_t print(const __FlashStringHelper*) { return 0; } };
#include <deque>
#include <vector>
struct HardwareSerial {
  std::deque<uint8_t> rx; std::vector<uint8_t> tx; bool open = false;
  void setRxBufferSize(size_t) {}
  void begin(unsigned long, uint32_t, int8_t, int8_t) { open = true; }
  void end() { open = false; }
  int available() { return (int)rx.size(); }
  int read() { int b = rx.front(); rx.pop_front(); return b; }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  size_t write(const uint8_t* p, size_t n) { tx.insert(tx.end(), p, p+n); return n; }
};
extern HardwareSerial Serial1;
#define USERMOD_ID_UNSPECIFIED 1
enum struct PinOwner : uint8_t { None = 0, UM_Unspecified = USERMOD_ID_UNSPECIFIED };
namespace PinManager { bool allocatePin(byte gpio, bool output, PinOwner tag); bool deallocatePin(byte gpio, PinOwner tag);
  byte allocateLedc(byte channels); void deallocateLedc(byte pos, byte channels); }
uint32_t ledcSetup(uint8_t channel, uint32_t freq, uint8_t bits);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcDetachPin(uint8_t pin);
void ledcWrite(uint8_t channel, uint32_t duty);
uint32_t analogReadMilliVolts(uint8_t pin);
template<typename DestType> bool getJsonValue(const JsonVariant& element, DestType& destination) {
  if (element.isNull()) { return false; } destination = element.as<DestType>(); return true; }
template<typename DestType, typename DefaultType> bool getJsonValue(const JsonVariant& element, DestType& destination, const DefaultType defaultValue) {
  if (!getJsonValue(element, destination)) { destination = defaultValue; return false; } return true; }
#define CALL_MODE_BUTTON_PRESET 12
bool applyPreset(byte index, byte callMode);
extern byte bri;
#define MQTT_MAX_TOPIC_LEN 32
struct AsyncMqttClient { bool connected(); uint16_t publish(const char*, uint8_t, bool, const char* = nullptr, size_t = 0); uint16_t subscribe(const char*, uint8_t); };
extern AsyncMqttClient* mqtt;
#define WLED_MQTT_CONNECTED (mqtt != nullptr && mqtt->connected())
extern char mqttDeviceTopic[MQTT_MAX_TOPIC_LEN + 1], mqttStatusTopic[MQTT_MAX_TOPIC_LEN + 8], mqttClientID[41], serverDescription[33], versionString[];
extern String escapedMac;
#define WLED_BRAND "WLED"
#define WLED_PRODUCT_NAME "FOSS"
struct um_data_t;
class Usermod {
  protected: um_data_t *um_data;
  public:
    Usermod() : um_data(nullptr) {};
    virtual ~Usermod() {}
    virtual void setup() = 0;
    virtual void loop() = 0;
    virtual void handleOverlayDraw() {}
    virtual bool handleButton(uint8_t b) { return false; }
    virtual void connected() {}
    virtual void appendConfigData(Print& settingsScript) {}
    virtual void addToJsonState(JsonObject& obj) {}
    virtual void addToJsonInfo(JsonObject& obj) {}
    virtual void readFromJsonState(JsonObject& obj) {}
    virtual void addToConfig(JsonObject& obj) {}
    virtual bool readFromConfig(JsonObject& obj) { return true; }
    virtual void onMqttConnect(bool sessionPresent) {}
    virtual bool onMqttMessage(char* topic, char* payload) { return false; }
    virtual uint16_t getId() { return USERMOD_ID_UNSPECIFIED; }
  private: virtual void appendConfigData() {}
};
#define REGISTER_USERMOD(x) Usermod* um_##x = &x
