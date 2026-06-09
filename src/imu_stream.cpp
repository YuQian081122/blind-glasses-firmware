/**
 * IMU 串流實作 - ICM-20948 (I2C)
 */

#include "imu_stream.h"
#include "config.h"

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <stdarg.h>
#include <math.h>
#if SERVER_USE_HTTPS
#include <WiFiClientSecure.h>
#endif

#include "ICM_20948.h"
#include "imu_kalman.h"

namespace ImuStream {

  static ICM_20948_I2C imu;
#if IMU_KALMAN_ENABLE
  static ImuKalman::AngleFilter kalmanPitch;
  static ImuKalman::AngleFilter kalmanRoll;
  static unsigned long lastKalmanMs = 0;
#endif
  static IPAddress serverIP(0, 0, 0, 0);
  static unsigned long lastSendTime = 0;
  static bool ready = false;
  static bool loggedPostOk = false;
  static unsigned long lastImuDebugMs = 0;
  static unsigned long lastI2CScanMs = 0;
  static bool standaloneMode = false;
  static uint32_t imuReadOkCount = 0;
  static uint32_t imuReadFailCount = 0;

  // 最近一次 I2C scan 的判斷結果：用於 init fail 時的 CASE 類型輸出
  static bool i2cHadAnyAck = false;
  static bool i2cHad68or69 = false;

  static void scanI2CAndPrint() {
    Serial.printf("[IMU] I2C scan start (SDA=%d SCL=%d)\n", IMU_SDA_PIN, IMU_SCL_PIN);
    uint8_t found = 0;
    bool has68 = false;
    bool has69 = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
      if ((addr & 0x07) == 0) {
        esp_task_wdt_reset();
      }
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission(); // 0 = ACK
      if (err == 0) {
        Serial.printf("[IMU] I2C ACK addr=0x%02X\n", addr);
        found++;
        if (addr == 0x68) has68 = true;
        if (addr == 0x69) has69 = true;
      }
    }

    i2cHadAnyAck = (found > 0);
    i2cHad68or69 = (has68 || has69);

    if (found == 0) {
      Serial.println("[IMU] I2C scan: no devices ACK (check wiring/pullups/I2C mode)");
      Serial.println("[IMU][CASE_A] 0x00 ACK：優先檢查 VCC/GND、SDA/SCL 是否接反、是否共地與上拉電阻");
    } else {
      if (!i2cHad68or69) {
        Serial.println("[IMU][CASE_B] 有 ACK 但不在 0x68/0x69：可能接錯裝置或 I2C 總線干擾");
      } else {
        Serial.println("[IMU] I2C found IMU addr candidate (0x68/0x69)");
      }
      Serial.printf("[IMU] I2C scan done, found=%u\n", found);
    }
  }

  static void throttledWarn(const char* msg) {
    unsigned long now = millis();
    unsigned long throttle = standaloneMode ? IMU_TEST_FAIL_THROTTLE_MS : 5000;
    if (now - lastImuDebugMs < throttle) return;
    lastImuDebugMs = now;
    Serial.println(msg);
  }

  static void throttledPrintf(const char* fmt, ...) {
    unsigned long now = millis();
    unsigned long throttle = standaloneMode ? IMU_TEST_FAIL_THROTTLE_MS : 5000;
    if (now - lastImuDebugMs < throttle) return;
    lastImuDebugMs = now;

    char buf[240];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.println(buf);
  }

  void setServerIP(IPAddress ip) {
    serverIP = ip;
  }

  bool isReady() {
    return ready;
  }

  void beginStandalone() {
    standaloneMode = true;
    imuReadOkCount = 0;
    imuReadFailCount = 0;
    loggedPostOk = false;
    Serial.println("[IMU-TEST] Standalone mode begin (no WiFi/HTTP)");
    begin();
  }

  void begin() {
    // I2C 線若模組沒有提供 pull-up，掃描會完全沒有 ACK。
    // 先開啟 ESP32 的內建上拉，讓 SDA/SCL 至少能被拉到預設高電位。
    pinMode(IMU_SDA_PIN, INPUT_PULLUP);
    pinMode(IMU_SCL_PIN, INPUT_PULLUP);
    delay(10);
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    scanI2CAndPrint();
    // AD0=LOW -> 0x68；AD0=HIGH -> 0x69
    // 若你不確定 IMU 模組 AD0 腳接哪邊，改成「先嘗試設定的地址；失敗再自動嘗試另一個」會更快定位問題。
    bool firstAd0 = (IMU_I2C_ADDR == 0x69);
    uint8_t firstAddr = firstAd0 ? 0x69 : 0x68;
    uint8_t secondAddr = firstAd0 ? 0x68 : 0x69;

    uint8_t addrOk = 0;
    esp_task_wdt_reset();
    if (imu.begin(Wire, firstAd0, ICM_20948_ARD_UNUSED_PIN) == ICM_20948_Stat_Ok) {
      addrOk = firstAddr;
      esp_task_wdt_reset();
    } else if (imu.begin(Wire, !firstAd0, ICM_20948_ARD_UNUSED_PIN) == ICM_20948_Stat_Ok) {
      addrOk = secondAddr;
      esp_task_wdt_reset();
    } else {
      if (i2cHadAnyAck) {
        Serial.printf("[IMU][CASE_C] ACK 出現但 Init failed（請檢查焊接/供電/AD0 狀態/上拉品質）(try 0x%02X/0x%02X)\n",
                      firstAddr, secondAddr);
      } else {
        Serial.printf("[IMU] Init failed (no ACK; try 0x%02X/0x%02X). Check wiring/pullups\n", firstAddr, secondAddr);
      }
      return;
    }

    ready = true;
#if IMU_KALMAN_ENABLE
    kalmanPitch.configure(IMU_KALMAN_Q_ANGLE, IMU_KALMAN_Q_GYRO, IMU_KALMAN_R_ANGLE);
    kalmanRoll.configure(IMU_KALMAN_Q_ANGLE, IMU_KALMAN_Q_GYRO, IMU_KALMAN_R_ANGLE);
    kalmanPitch.reset();
    kalmanRoll.reset();
    lastKalmanMs = millis();
    Serial.println("[IMU] Kalman fusion enabled (pitch/roll)");
#endif
    Serial.printf("[IMU] Ready (ICM-20948) addr=0x%02X\n", addrOk);
  }

  void tick() {
    if (!ready) {
      unsigned long now = millis();
      if (now - lastI2CScanMs > 10000) {
        lastI2CScanMs = now;
        scanI2CAndPrint();
      }
      if (!standaloneMode) {
#if CLOUD_MODE
        throttledWarn("[IMU] Not sending: sensor init failed (see boot log [IMU] Init failed)");
#else
        if (serverIP != IPAddress(0, 0, 0, 0))
          throttledWarn("[IMU] Not sending: sensor init failed (see boot log [IMU] Init failed)");
#endif
      }
      return;
    }
#if !CLOUD_MODE
    if (!standaloneMode && serverIP == IPAddress(0, 0, 0, 0)) {
      throttledWarn("[IMU] Not sending: no server IP yet (wait for UDP)");
      return;
    }
#endif

    unsigned long now = millis();
    unsigned long interval = IMU_SEND_INTERVAL_MS;
#if POWER_SAVE_ENABLE
    interval = IMU_SEND_INTERVAL_PS_MS;
#endif
    if (standaloneMode) {
      interval = IMU_TEST_OUTPUT_INTERVAL_MS;
    }
    if (now - lastSendTime < interval) return;
    lastSendTime = now;

    imu.getAGMT();
    if (imu.status != ICM_20948_Stat_Ok) {
      imuReadFailCount++;
      if (standaloneMode) {
        throttledPrintf("[IMU][CASE_D] getAGMT failed, cnt=%lu, status=%d",
                        (unsigned long)imuReadFailCount, (int)imu.status);
      } else {
        throttledWarn("[IMU] I2C read failed (getAGMT); check wiring / IMU_I2C_ADDR");
      }
      return;
    }
    // acc 為 milli-g，換算成 g；gyr 為 dps
    float ax = imu.accX() / 1000.0f;
    float ay = imu.accY() / 1000.0f;
    float az = imu.accZ() / 1000.0f;
    float gx = imu.gyrX();
    float gy = imu.gyrY();
    float gz = imu.gyrZ();

    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    float gxK = gx;
    float gyK = gy;
#if IMU_KALMAN_ENABLE
    float pitchAccel = 0.0f;
    float rollAccel = 0.0f;
    ImuKalman::accelToPitchRoll(ax, ay, az, pitchAccel, rollAccel);
    const unsigned long kalmanNow = millis();
    float dtSec = (lastKalmanMs > 0) ? (kalmanNow - lastKalmanMs) / 1000.0f : (IMU_SEND_INTERVAL_MS / 1000.0f);
    lastKalmanMs = kalmanNow;
    const ImuKalman::Result pitchRes = kalmanPitch.update(pitchAccel, gy, dtSec);
    const ImuKalman::Result rollRes = kalmanRoll.update(rollAccel, gx, dtSec);
    pitchDeg = pitchRes.angleDeg;
    rollDeg = rollRes.angleDeg;
    gxK = rollRes.gyroCorrectedDps;
    gyK = pitchRes.gyroCorrectedDps;
#endif

    if (standaloneMode) {
      imuReadOkCount++;
#if IMU_KALMAN_ENABLE
      Serial.printf("[IMU-TEST] #%lu ax=%.2fg ay=%.2fg az=%.2fg | gx=%.1f gy=%.1f gz=%.1f | pitch=%.1f roll=%.1f\n",
                    (unsigned long)imuReadOkCount, ax, ay, az, gxK, gyK, gz, pitchDeg, rollDeg);
#else
      Serial.printf("[IMU-TEST] #%lu ax=%.2fg ay=%.2fg az=%.2fg | gx=%.2fdps gy=%.2fdps gz=%.2fdps\n",
                    (unsigned long)imuReadOkCount, ax, ay, az, gx, gy, gz);
#endif
      if (imuReadOkCount % 30 == 0) {
        if (fabsf(gx) < 0.5f && fabsf(gy) < 0.5f && fabsf(gz) < 0.5f) {
          Serial.println("[IMU][CASE_E] 連續讀值近乎不變，請手動轉動模組，若仍無變化可能為感測器或讀值路徑異常");
        }
      }
      return;
    }

    char url[128];
#if CLOUD_MODE
    snprintf(url, sizeof(url), "https://%s%s", SERVER_HOST, API_IMU_PATH);
#elif SERVER_HTTP_PORT == 80
    snprintf(url, sizeof(url), "http://%d.%d.%d.%d%s",
             serverIP[0], serverIP[1], serverIP[2], serverIP[3], API_IMU_PATH);
#else
    snprintf(url, sizeof(url), "http://%d.%d.%d.%d:%d%s",
             serverIP[0], serverIP[1], serverIP[2], serverIP[3], SERVER_HTTP_PORT, API_IMU_PATH);
#endif

    StaticJsonDocument<256> doc;
    doc["ax"] = round(ax * 100) / 100.0f;
    doc["ay"] = round(ay * 100) / 100.0f;
    doc["az"] = round(az * 100) / 100.0f;
    doc["gx"] = round(gx * 100) / 100.0f;
    doc["gy"] = round(gy * 100) / 100.0f;
    doc["gz"] = round(gz * 100) / 100.0f;
#if IMU_KALMAN_ENABLE
    doc["pitch"] = round(pitchDeg * 10) / 10.0f;
    doc["roll"] = round(rollDeg * 10) / 10.0f;
    doc["gx_k"] = round(gxK * 10) / 10.0f;
    doc["gy_k"] = round(gyK * 10) / 10.0f;
#endif

    char body[200];
    serializeJson(doc, body);

    HTTPClient http;
#if SERVER_USE_HTTPS
    WiFiClientSecure client;
    client.setInsecure();
    http.begin(client, url);
#else
    http.begin(url);
#endif
    http.addHeader("Content-Type", "application/json");
#if CLOUD_MODE
    const char* devToken = DEVICE_API_TOKEN;
    if (devToken && devToken[0]) http.addHeader("X-Device-Token", devToken);
    {
      const char* pubUrl = DEVICE_PUBLIC_STREAM_URL;
      if (pubUrl && pubUrl[0]) {
        http.addHeader("X-Device-Stream-Url", pubUrl);
      } else if (DEVICE_STREAM_REPORT_LAN_URL) {
        IPAddress lip = WiFi.localIP();
        if (lip != IPAddress(0, 0, 0, 0)) {
          char streamHdrBuf[96];
          snprintf(
              streamHdrBuf,
              sizeof(streamHdrBuf),
              "http://%u.%u.%u.%u:%d%s",
              lip[0],
              lip[1],
              lip[2],
              lip[3],
              STREAM_PORT,
              STREAM_PATH);
          http.addHeader("X-Device-Stream-Url", streamHdrBuf);
        }
      }
    }
#endif
    http.setTimeout(2000);
#if POWER_SAVE_ENABLE && WIFI_MODEM_SLEEP
    WiFi.setSleep(false);
#endif
    int code = http.POST(body);
#if POWER_SAVE_ENABLE && WIFI_MODEM_SLEEP
    WiFi.setSleep(true);
#endif
    http.end();
    if (code > 0 && code < 400) {
      if (!loggedPostOk) {
        loggedPostOk = true;
        Serial.println("[IMU] Server receiving IMU (gyro/accel) data");
      }
    } else {
#if CLOUD_MODE
      Serial.printf("[IMU] POST failed: %d (server %s)\n", code, SERVER_HOST);
#else
      Serial.printf("[IMU] POST failed: %d (server %d.%d.%d.%d)\n", code,
                    serverIP[0], serverIP[1], serverIP[2], serverIP[3]);
#endif
    }
  }

}  // namespace ImuStream
