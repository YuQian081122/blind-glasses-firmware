/**
 * 智慧導盲眼鏡 - 主程式
 * 按鍵：電源鍵短按=紅綠燈、長按=開關機；切換鍵短按=模式、長按=語音助理
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include "esp_coexist.h"
#include "driver/rtc_io.h"

#include "config.h"
#include "op_mode.h"
#include "udp_discovery.h"
#include "camera_stream.h"
#include "button_handler.h"
#include "audio_player.h"
#include "server_api.h"
#include "http_task_queue.h"
#include "imu_stream.h"
#include "mic_upload.h"
#include "gps_stream.h"
#include "ble_quick_link.h"

#if CLOUD_MODE
  static inline bool serverReady() { return WiFi.status() == WL_CONNECTED; }
  static inline IPAddress serverAddr() { return IPAddress(1, 1, 1, 1); }
#else
  static inline bool serverReady() { return UdpDiscovery::hasServer(); }
  static inline IPAddress serverAddr() { return UdpDiscovery::getServerIP(); }
#endif

static unsigned long lastAudioTriggerTime = 0;
static const unsigned long AUDIO_FETCH_DELAY_MS = 500;
static const unsigned long TTS_GRACE_MS = 300;
static unsigned long taskCompletedTime = 0;
static bool waitingForTaskCompletion = false;
static unsigned long lastAutoAudioTestTime = 0;
#if MIC_AUTO_TEST_ENABLE
static unsigned long lastMicAutoTestTime = 0;
#endif
#if FRAME_PUSH_ENABLE && CAMERA_ENABLE
static unsigned long lastFramePushMs = 0;
#endif
static bool singleBtnVoiceThenMonitor = false;
static bool voicePlaybackStarted = false;
static unsigned long gpio1ToggleMs = 0;
static bool gpio1LevelHigh = false;

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    Serial.printf("[WiFi] Connected ch=%d\n", info.wifi_sta_connected.channel);
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf("[WiFi] Disconnected reason=%d\n",
                  info.wifi_sta_disconnected.reason);
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.printf("[WiFi] Got IP: %s\n", WiFi.localIP().toString().c_str());
  }
}

/** BLE 先啟動時，WiFi.begin 前必須開 modem sleep，否則 IDF 直接 abort */
static inline void wifiApplyRequiredSleepBeforeBegin() {
#if BLE_QUICK_LINK_ENABLE
  WiFi.setSleep(true);
#else
  WiFi.setSleep(false);
#endif
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  wifiApplyRequiredSleepBeforeBegin();
#if BLE_QUICK_LINK_ENABLE
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
#endif
  WiFi.setAutoReconnect(true);

  String cfgSsid = String(WIFI_SSID);
  cfgSsid.trim();
  if (!cfgSsid.isEmpty()) {
    Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    for (int i = 0; i < 10; i++) {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(500);
      esp_task_wdt_reset();
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
#if CLOUD_MODE
      Serial.printf("API host: https://%s\n", SERVER_HOST);
      Serial.printf("Monitor: https://%s/monitor\n", SERVER_HOST);
      Serial.printf("Local MJPEG: http://%s:%d%s\n",
                    WiFi.localIP().toString().c_str(), STREAM_PORT, STREAM_PATH);
#else
      Serial.printf("Stream URL: http://%s:%d%s\n",
                    WiFi.localIP().toString().c_str(), STREAM_PORT, STREAM_PATH);
#endif
    } else {
      Serial.println();
      Serial.println("[WiFi] Not connected yet (autoReconnect enabled)");
      Serial.println("[WiFi] Use BLE to send new WiFi credentials if needed");
    }
  } else {
    Serial.println("[WiFi] WIFI_SSID is empty, skipping");
  }

#if BLE_QUICK_LINK_ENABLE
  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
#endif
}

void applyWifiFromBle(const String& ssid, const String& pass) {
  if (ssid.isEmpty()) {
    Serial.println("[BLE] WiFi apply ignored: empty ssid");
    return;
  }
  Serial.printf("[BLE] Applying WiFi SSID=%s\n", ssid.c_str());
#if BLE_QUICK_LINK_ENABLE
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
#endif
  WiFi.disconnect(true, false);
  delay(500);
  WiFi.mode(WIFI_STA);
  wifiApplyRequiredSleepBeforeBegin();
  WiFi.setAutoReconnect(true);
  delay(200);
  WiFi.begin(ssid.c_str(), pass.c_str());
}

void triggerFindAlert(unsigned long durationMs) {
  (void)durationMs;
  if (serverReady()) {
    Serial.println("[BLE] Find-me alert: play latest server audio");
    AudioPlayer::playFromServer(serverAddr());
    return;
  }
  Serial.println("[BLE] Find-me alert requested, but server not ready");
}

void setup() {
  Serial.begin(115200);
  disableCore0WDT();
  delay(1000);
  Serial.println("\n=== Smart Blind Glasses ===");

#if GPIO1_TOGGLE_TEST_ENABLE
  pinMode(GPIO1_TOGGLE_TEST_PIN, OUTPUT);
  digitalWrite(GPIO1_TOGGLE_TEST_PIN, LOW);
  gpio1LevelHigh = false;
  gpio1ToggleMs = millis();
  Serial.printf("[GPIO-TEST] pin=%d enabled, interval=%lu ms\n",
                GPIO1_TOGGLE_TEST_PIN, (unsigned long)GPIO1_TOGGLE_INTERVAL_MS);
#endif

#if IMU_STANDALONE_TEST
  Serial.println("[IMU-TEST] IMU_STANDALONE_TEST=1, skip WiFi/Camera/GPS/Mic/Audio/UDP");
  ImuStream::beginStandalone();
  enableCore0WDT();
  return;
#endif

  OpMode::begin();
  Serial.printf("[OP] Mode: %s\n", OpMode::isSingleButton() ? "SINGLE_BTN" : "ALWAYS_ON");
  WiFi.onEvent(onWiFiEvent);

  setupWifi();

#if !CLOUD_MODE
  Serial.println("[INIT] UdpDiscovery");
  UdpDiscovery::begin();
#endif

  Serial.println("[INIT] ButtonHandler");
  ButtonHandler::begin();
  delay(100);

  Serial.println("[INIT] AudioPlayer");
  AudioPlayer::begin();
  delay(100);

  Serial.println("[INIT] MicUpload");
  MicUpload::begin();
  delay(100);

  Serial.println("[INIT] HttpTaskQueue");
  HttpTaskQueue::begin();
  delay(100);

#if !CAMERA_ENABLE
  Serial.println("[CAM] Disabled (CAMERA_ENABLE=0)");
#else
  {
    const bool startCam = OpMode::isAlwaysOn() || (CAMERA_START_ON_BOOT);
    if (!startCam) {
#if POWER_SAVE_ENABLE
      Serial.println("[PWR] Camera off in SINGLE_BTN (power save)");
#else
      Serial.println("[CAM] Skipped at boot (CAMERA_START_ON_BOOT=0, single-btn)");
#endif
    } else {
      Serial.println("[INIT] CameraStream");
      if (CameraStream::begin()) {
        Serial.println("[CAM] Ready");
      } else {
        Serial.println("[CAM] Init failed");
      }
    }
  }
  delay(100);
#endif

  Serial.println("[INIT] ImuStream");
  ImuStream::begin();
  delay(100);

  Serial.println("[INIT] GpsStream");
  GpsStream::begin();
  delay(100);

#if BLE_QUICK_LINK_ENABLE
  delay(200);
#endif
  Serial.println("[INIT] BleQuickLink");
  BleQuickLink::begin();

  Serial.println("[INIT] setup complete");
  enableCore0WDT();

#if AUDIO_AUTO_TEST_ENABLE
  lastAutoAudioTestTime = millis();
  Serial.printf("[AUDIO-TEST] enabled, interval=%lu ms\n",
                (unsigned long)AUDIO_AUTO_TEST_INTERVAL_MS);
#endif

#if MIC_AUTO_TEST_ENABLE
  lastMicAutoTestTime = millis();
  Serial.printf("[MIC-AUTO-TEST] enabled, interval=%lu ms (no button)\n",
                (unsigned long)MIC_AUTO_TEST_INTERVAL_MS);
#endif

#if POWER_SAVE_ENABLE && WIFI_MODEM_SLEEP
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(true);
    Serial.println("[PWR] WiFi modem sleep on");
  }
#endif
}

void loop() {
#if GPIO1_TOGGLE_TEST_ENABLE
  unsigned long now = millis();
  if (now - gpio1ToggleMs >= GPIO1_TOGGLE_INTERVAL_MS) {
    gpio1ToggleMs = now;
    gpio1LevelHigh = !gpio1LevelHigh;
    digitalWrite(GPIO1_TOGGLE_TEST_PIN, gpio1LevelHigh ? HIGH : LOW);
    Serial.printf("[GPIO-TEST] pin=%d level=%s\n",
                  GPIO1_TOGGLE_TEST_PIN, gpio1LevelHigh ? "HIGH" : "LOW");
  }
#endif

#if IMU_STANDALONE_TEST
  ImuStream::tick();
  delay(5);
  return;
#endif

#if !CLOUD_MODE
  UdpDiscovery::tick();
#endif
  BleQuickLink::tick();
  BleQuickLink::setRuntimeStatus(WiFi.status() == WL_CONNECTED, WiFi.localIP());
  HttpTaskQueue::tick();
  AudioPlayer::tick();
  MicUpload::tick();

#if MIC_AUTO_TEST_ENABLE
  if (serverReady() && WiFi.status() == WL_CONNECTED && !MicUpload::isRecording() &&
      !MicUpload::hasPendingAudioFetch() && !AudioPlayer::isPlaying() &&
      !HttpTaskQueue::isBusy() &&
      (millis() - lastMicAutoTestTime >= MIC_AUTO_TEST_INTERVAL_MS)) {
    lastMicAutoTestTime = millis();
    Serial.println("[MIC-AUTO-TEST] trigger startRecording");
    MicUpload::startRecording();
  }
#endif

  String bleSsid;
  String blePass;
  if (BleQuickLink::consumeWifiApplyRequest(bleSsid, blePass)) {
    applyWifiFromBle(bleSsid, blePass);
  }
  if (BleQuickLink::consumeFindMeRequest()) {
    triggerFindAlert(10000);
  }
  uint8_t bleOpMode = OP_MODE_DEFAULT;
  String bleTaskMode;
  bool hasBleTaskMode = false;
  if (BleQuickLink::consumeModeRequest(bleOpMode, bleTaskMode, hasBleTaskMode)) {
    OpMode::set(bleOpMode);
    Serial.printf("[BLE] Apply op_mode: %s\n", OpMode::isSingleButton() ? "SINGLE_BTN" : "ALWAYS_ON");
#if POWER_SAVE_ENABLE && CAMERA_ENABLE
    if (OpMode::isAlwaysOn() && !CameraStream::isReady() && WiFi.status() == WL_CONNECTED) {
      if (CameraStream::begin()) Serial.println("[CAM] Ready (ble mode)");
    }
#endif
    if (hasBleTaskMode) {
      if (serverReady()) {
        HttpTaskQueue::enqueueGemini(serverAddr(), bleTaskMode.c_str());
        lastAudioTriggerTime = millis();
        Serial.printf("[BLE] Trigger task_mode: %s\n", bleTaskMode.c_str());
      } else {
        Serial.printf("[BLE] task_mode=%s ignored: server not ready\n", bleTaskMode.c_str());
      }
    }
  }
  uint8_t newVolume = 0;
  if (BleQuickLink::consumeVolumeRequest(newVolume)) {
    AudioPlayer::setVolume(newVolume);
  }
  if (serverReady()) {
    IPAddress ip = serverAddr();
    ImuStream::setServerIP(ip);
    GpsStream::setServerIP(ip);
    ImuStream::tick();
    GpsStream::tick();

#if FRAME_PUSH_ENABLE && CAMERA_ENABLE
    if (CameraStream::isReady() &&
        (millis() - lastFramePushMs >= FRAME_PUSH_INTERVAL_MS)) {
      lastFramePushMs = millis();
      const uint8_t* jpegBuf = nullptr;
      size_t jpegLen = 0;
      if (CameraStream::captureJpeg(&jpegBuf, &jpegLen)) {
        ServerAPI::pushFrame(ip, jpegBuf, jpegLen);
        CameraStream::releaseFrame();
      }
    }
#endif
  }

  ButtonEvent evt = ButtonHandler::tick();
  if (evt == ButtonEvent::PowerToggle) {
    Serial.println("[PWR] Power off - deep sleep. Press power button to wake.");
    rtc_gpio_pulldown_dis((gpio_num_t)BTN_POWER_PIN);
    rtc_gpio_pullup_en((gpio_num_t)BTN_POWER_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_POWER_PIN, 0);
    esp_deep_sleep_start();
  } else if (evt == ButtonEvent::ModeSwitch) {
    OpMode::toggle();
#if POWER_SAVE_ENABLE && CAMERA_ENABLE
    if (OpMode::isAlwaysOn() && !CameraStream::isReady() && WiFi.status() == WL_CONNECTED) {
      if (CameraStream::begin()) Serial.println("[CAM] Ready (mode switch)");
    }
#endif
  } else if (evt != ButtonEvent::None && serverReady()) {
    IPAddress ip = serverAddr();
#if POWER_SAVE_ENABLE && CAMERA_ENABLE
    if (OpMode::isSingleButton() && !CameraStream::isReady() && WiFi.status() == WL_CONNECTED) {
      if (evt == ButtonEvent::TrafficShort || evt == ButtonEvent::SceneryLong) {
        if (CameraStream::begin()) Serial.println("[CAM] Ready (first use)");
      }
    }
#endif
    if (evt == ButtonEvent::TrafficShort) {
      HttpTaskQueue::clearCompleted();
      HttpTaskQueue::enqueueGemini(ip, "light");
      waitingForTaskCompletion = true;
      taskCompletedTime = 0;
      lastAudioTriggerTime = millis();
    } else if (evt == ButtonEvent::SceneryLong) {
      if (OpMode::isSingleButton()) singleBtnVoiceThenMonitor = true;
      MicUpload::startRecording();
    }
  }

  if (waitingForTaskCompletion && HttpTaskQueue::hasCompletedSinceEnqueue()) {
    taskCompletedTime = millis();
    waitingForTaskCompletion = false;
    Serial.printf("[LAT] Task completed, wait %dms for TTS\n", (int)TTS_GRACE_MS);
  }

  bool fromButton = false;
  if (lastAudioTriggerTime > 0 && !waitingForTaskCompletion) {
    if (taskCompletedTime > 0) {
      fromButton = (millis() - taskCompletedTime >= TTS_GRACE_MS);
    } else {
      fromButton = (millis() - lastAudioTriggerTime >= AUDIO_FETCH_DELAY_MS);
    }
  }
  bool fromAsr = MicUpload::hasPendingAudioFetch() && MicUpload::shouldFetchAudio();
  bool fromAutoTest = false;
#if AUDIO_AUTO_TEST_ENABLE
  if (serverReady() &&
      (millis() - lastAutoAudioTestTime >= AUDIO_AUTO_TEST_INTERVAL_MS)) {
    fromAutoTest = true;
    lastAutoAudioTestTime = millis();
    Serial.println("[AUDIO-TEST] trigger playFromServer");
  }
#endif
  bool canPlayNormal = (fromButton || fromAsr) && !AudioPlayer::isPlaying() && serverReady();
  bool canPlayAutoTest = fromAutoTest && serverReady();
  if (canPlayNormal || canPlayAutoTest) {
    if (singleBtnVoiceThenMonitor) voicePlaybackStarted = true;
    lastAudioTriggerTime = 0;
    MicUpload::clearPendingAudioFetch();
    if (canPlayAutoTest) {
      Serial.printf("[AUDIO-TEST] forcing playback (isPlaying=%d)\n", AudioPlayer::isPlaying() ? 1 : 0);
    }
    AudioPlayer::playFromServer(serverAddr());
  }

  if (singleBtnVoiceThenMonitor && voicePlaybackStarted && !AudioPlayer::isPlaying()) {
    singleBtnVoiceThenMonitor = false;
    voicePlaybackStarted = false;
    OpMode::set(OP_MODE_ALWAYS_ON);
#if POWER_SAVE_ENABLE && CAMERA_ENABLE
    if (!CameraStream::isReady() && CameraStream::begin()) {
      Serial.println("[CAM] Ready (power save -> ALWAYS_ON)");
    }
#endif
    Serial.println("[OP] Voice done -> ALWAYS_ON (monitoring)");
  }

  unsigned long idleMs = LOOP_IDLE_MS_NORMAL;
#if POWER_SAVE_ENABLE
  if (!AudioPlayer::isPlaying()) idleMs = LOOP_IDLE_MS;
#endif
  delay(idleMs);
}
