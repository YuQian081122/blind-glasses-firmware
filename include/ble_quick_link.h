#ifndef BLE_QUICK_LINK_H
#define BLE_QUICK_LINK_H

#include <Arduino.h>
#include <IPAddress.h>

namespace BleQuickLink {

  /** WiFi 前先 BLEDevice::init；begin() 僅建 GATT */
  void initStackEarly();
  void begin();
  void tick();

  bool hasWifiApplyRequest();
  bool consumeWifiApplyRequest(String& ssid, String& password);
  bool consumeFindMeRequest();
  bool consumeModeRequest(uint8_t& opMode, String& taskMode, bool& hasTaskMode);
  bool consumeVolumeRequest(uint8_t& outVolume);

  void setRuntimeStatus(bool wifiConnected, const IPAddress& ip);

}  // namespace BleQuickLink

#endif  // BLE_QUICK_LINK_H
