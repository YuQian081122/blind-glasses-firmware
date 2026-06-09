#include "ble_quick_link.h"
#include "config.h"

#if BLE_QUICK_LINK_ENABLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#endif

namespace BleQuickLink {

#if BLE_QUICK_LINK_ENABLE
  static BLEServer* server = nullptr;
  static BLECharacteristic* cWifiSsid = nullptr;
  static BLECharacteristic* cWifiPass = nullptr;
  static BLECharacteristic* cWifiApply = nullptr;
  static BLECharacteristic* cFindMe = nullptr;
  static BLECharacteristic* cMode = nullptr;
  static BLECharacteristic* cVolume = nullptr;
  static BLECharacteristic* cStatus = nullptr;
  static Preferences prefs;

  static String pendingSsid;
  static String pendingPass;
  static volatile bool requestWifiApply = false;
  static volatile bool requestFindMe = false;
  static volatile bool requestModeApply = false;
  static volatile bool requestVolumeApply = false;
  static uint8_t pendingOpMode = OP_MODE_DEFAULT;
  static String pendingTaskMode = "";
  static bool pendingHasTaskMode = false;
  static uint8_t pendingVolume = 21;
  static bool wifiConnected = false;
  static IPAddress wifiIp(0, 0, 0, 0);
  static unsigned long lastNotifyMs = 0;
  static bool bleStackInitialized = false;

  static bool parseWifiApplyPayload(const String& payload) {
    // 相容兩種格式：
    // 1) 原本：payload == "1"（只做觸發）
    // 2) JSON：{"ssid":"...","pwd":"...","wifiApply":1} 或 {"wifiApply":true}
    String p = payload;
    p.trim();

    if (p == "1" || p == "true") return true;
    if (p.length() < 2 || p.charAt(0) != '{') return false;

    StaticJsonDocument<384> doc;
    DeserializationError err = deserializeJson(doc, p);
    if (err) return false;

    bool doApply = false;
    if (!doc["wifiApply"].isNull()) {
      if (doc["wifiApply"].is<bool>()) doApply = doc["wifiApply"].as<bool>();
      else if (doc["wifiApply"].is<int>()) doApply = (doc["wifiApply"].as<int>() == 1);
      else if (doc["wifiApply"].is<const char*>()) {
        const char* s = doc["wifiApply"].as<const char*>();
        doApply = (s && String(s) == "1");
      } else if (doc["wifiApply"].is<long>()) {
        doApply = (doc["wifiApply"].as<long>() == 1L);
      }
    }
    if (!doApply) return false;

    // 可選：允許 Android 只寫 wifiApply 一次，就把 ssid/pwd 一次帶入
    if (!doc["ssid"].isNull()) pendingSsid = doc["ssid"].as<String>();
    if (!doc["pwd"].isNull()) pendingPass = doc["pwd"].as<String>();
    return true;
  }

  static bool parseFindMePayload(const String& payload) {
    String p = payload;
    p.trim();

    if (p == "1" || p == "true") return true;
    if (p.length() < 2 || p.charAt(0) != '{') return false;

    StaticJsonDocument<192> doc;
    DeserializationError err = deserializeJson(doc, p);
    if (err) return false;

    if (doc["findMe"].is<bool>()) return doc["findMe"].as<bool>();
    if (doc["findMe"].is<int>()) return doc["findMe"].as<int>() == 1;
    if (doc["findMe"].is<long>()) return doc["findMe"].as<long>() == 1L;
    if (doc["findMe"].is<const char*>()) {
      const char* s = doc["findMe"].as<const char*>();
      return s && String(s) == "1";
    }
    return false;
  }

  static bool parseOpModeValue(const JsonVariantConst& v, uint8_t& outMode) {
    if (v.is<int>()) {
      int iv = v.as<int>();
      if (iv == 0 || iv == 1) {
        outMode = (uint8_t)iv;
        return true;
      }
      return false;
    }
    if (v.is<const char*>()) {
      String s = String(v.as<const char*>());
      s.trim();
      s.toUpperCase();
      if (s == "SINGLE_BTN" || s == "0") {
        outMode = OP_MODE_SINGLE_BTN;
        return true;
      }
      if (s == "ALWAYS_ON" || s == "1") {
        outMode = OP_MODE_ALWAYS_ON;
        return true;
      }
    }
    return false;
  }

  static bool normalizeTaskMode(const JsonVariantConst& v, String& outMode) {
    if (!v.is<const char*>()) return false;
    String s = String(v.as<const char*>());
    s.trim();
    s.toLowerCase();
    if (s == "general" || s == "light" || s == "item_search") {
      outMode = s;
      return true;
    }
    return false;
  }

  static bool parseModePayload(const String& payload) {
    String p = payload;
    p.trim();
    if (p.isEmpty()) return false;

    uint8_t parsedOpMode = pendingOpMode;
    String parsedTaskMode = pendingTaskMode;
    bool hasMode = false;
    bool hasTask = false;

    if (p.charAt(0) == '{') {
      StaticJsonDocument<320> doc;
      DeserializationError err = deserializeJson(doc, p);
      if (err) return false;

      if (!doc["op_mode"].isNull()) {
        if (parseOpModeValue(doc["op_mode"], parsedOpMode)) hasMode = true;
      }
      if (!doc["task_mode"].isNull()) {
        if (normalizeTaskMode(doc["task_mode"], parsedTaskMode)) hasTask = true;
      }
    } else {
      // 相容非 JSON：直接寫入 "SINGLE_BTN" / "ALWAYS_ON" / "0" / "1"
      StaticJsonDocument<64> doc;
      doc["op_mode"] = p;
      if (parseOpModeValue(doc["op_mode"], parsedOpMode)) hasMode = true;
    }

    if (!hasMode && !hasTask) return false;
    if (hasMode) pendingOpMode = parsedOpMode;
    pendingHasTaskMode = hasTask;
    if (hasTask) pendingTaskMode = parsedTaskMode;
    return true;
  }

  class WriteCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
      std::string v = c->getValue();
      String value(v.c_str());
      if (c == cWifiSsid) {
        pendingSsid = value;
      } else if (c == cWifiPass) {
        pendingPass = value;
      } else if (c == cWifiApply && parseWifiApplyPayload(value)) {
        requestWifiApply = true;
      } else if (c == cFindMe && parseFindMePayload(value)) {
        requestFindMe = true;
      } else if (c == cMode && parseModePayload(value)) {
        requestModeApply = true;
      } else if (c == cVolume) {
        int vol = value.toInt();
        if (vol >= 0 && vol <= 21) {
          pendingVolume = (uint8_t)vol;
          requestVolumeApply = true;
        }
      }
    }
  };

  static WriteCallbacks callbacks;

  static String statusJson() {
    String s = "{";
    s += "\"wifi\":";
    s += (wifiConnected ? "true" : "false");
    s += ",\"ip\":\"";
    s += wifiIp.toString();
    s += "\",\"ssid\":\"";
    s += pendingSsid;
    s += "\"}";
    return s;
  }
#endif

  void initStackEarly() {
#if BLE_QUICK_LINK_ENABLE
    {
      Preferences p;
      if (p.begin("wifi_cfg", false)) {
        pendingSsid = p.getString("ssid", WIFI_SSID);
        pendingPass = p.getString("pass", WIFI_PASSWORD);
        p.end();
      } else {
        pendingSsid = String(WIFI_SSID);
        pendingPass = String(WIFI_PASSWORD);
      }
    }
    pendingSsid.trim();
    pendingPass.trim();
    Serial.println("[BLE] stack (BT controller) ready before WiFi");
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(517);
    bleStackInitialized = true;
#endif
  }

  void begin() {
#if BLE_QUICK_LINK_ENABLE
    if (!bleStackInitialized) {
      Preferences p;
      if (p.begin("wifi_cfg", false)) {
        pendingSsid = p.getString("ssid", WIFI_SSID);
        pendingPass = p.getString("pass", WIFI_PASSWORD);
        p.end();
      } else {
        pendingSsid = String(WIFI_SSID);
        pendingPass = String(WIFI_PASSWORD);
      }
      pendingSsid.trim();
      pendingPass.trim();
      Serial.println("[BLE] stack init on begin (after WiFi)");
      BLEDevice::init(BLE_DEVICE_NAME);
      BLEDevice::setMTU(517);
      bleStackInitialized = true;
    }
    server = BLEDevice::createServer();
    BLEService* service = server->createService(BLE_SERVICE_UUID);

    cWifiSsid = service->createCharacteristic(BLE_WIFI_SSID_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    cWifiPass = service->createCharacteristic(BLE_WIFI_PASS_UUID, BLECharacteristic::PROPERTY_WRITE);
    cWifiApply = service->createCharacteristic(BLE_WIFI_APPLY_UUID, BLECharacteristic::PROPERTY_WRITE);
    cFindMe = service->createCharacteristic(BLE_FIND_ME_UUID, BLECharacteristic::PROPERTY_WRITE);
    cMode = service->createCharacteristic(BLE_MODE_UUID, BLECharacteristic::PROPERTY_WRITE);
    cVolume = service->createCharacteristic(BLE_VOLUME_UUID, BLECharacteristic::PROPERTY_WRITE);
    cStatus = service->createCharacteristic(BLE_STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    cWifiSsid->setCallbacks(&callbacks);
    cWifiPass->setCallbacks(&callbacks);
    cWifiApply->setCallbacks(&callbacks);
    cFindMe->setCallbacks(&callbacks);
    cMode->setCallbacks(&callbacks);
    cVolume->setCallbacks(&callbacks);

    cWifiSsid->setValue(pendingSsid.c_str());
    cStatus->setValue("{\"wifi\":false}");

    service->start();
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->start();
    Serial.printf("[BLE] Quick link ready: %s\n", BLE_DEVICE_NAME);
#endif
  }

  void tick() {
#if BLE_QUICK_LINK_ENABLE
    const unsigned long now = millis();
    if (now - lastNotifyMs >= BLE_STATUS_NOTIFY_MS) {
      lastNotifyMs = now;
      if (cStatus) {
        String payload = statusJson();
        cStatus->setValue(payload.c_str());
        cStatus->notify();
      }
    }
#endif
  }

  bool hasWifiApplyRequest() {
#if BLE_QUICK_LINK_ENABLE
    return requestWifiApply;
#else
    return false;
#endif
  }

  bool consumeWifiApplyRequest(String& ssid, String& password) {
#if BLE_QUICK_LINK_ENABLE
    if (!requestWifiApply) return false;
    requestWifiApply = false;
    ssid = pendingSsid;
    password = pendingPass;
    if (prefs.begin("wifi_cfg", false)) {
      prefs.putString("ssid", ssid);
      prefs.putString("pass", password);
      prefs.end();
    }
    return true;
#else
    (void)ssid;
    (void)password;
    return false;
#endif
  }

  bool consumeFindMeRequest() {
#if BLE_QUICK_LINK_ENABLE
    if (!requestFindMe) return false;
    requestFindMe = false;
    return true;
#else
    return false;
#endif
  }

  bool consumeModeRequest(uint8_t& opMode, String& taskMode, bool& hasTaskMode) {
#if BLE_QUICK_LINK_ENABLE
    if (!requestModeApply) return false;
    requestModeApply = false;
    opMode = pendingOpMode;
    hasTaskMode = pendingHasTaskMode;
    taskMode = pendingTaskMode;
    return true;
#else
    (void)opMode;
    (void)taskMode;
    (void)hasTaskMode;
    return false;
#endif
  }

  bool consumeVolumeRequest(uint8_t& outVolume) {
#if BLE_QUICK_LINK_ENABLE
    if (!requestVolumeApply) return false;
    requestVolumeApply = false;
    outVolume = pendingVolume;
    return true;
#else
    (void)outVolume;
    return false;
#endif
  }

  void setRuntimeStatus(bool connected, const IPAddress& ip) {
#if BLE_QUICK_LINK_ENABLE
    wifiConnected = connected;
    wifiIp = ip;
#else
    (void)connected;
    (void)ip;
#endif
  }

}  // namespace BleQuickLink
