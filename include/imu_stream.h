/**
 * IMU 資料串流 - ICM-20948
 * 定時讀取加速度、陀螺儀；可選卡爾曼融合 pitch/roll 後上傳伺服器
 */

#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <Arduino.h>
#include <IPAddress.h>

namespace ImuStream {

  void begin();
  // 只做 IMU 診斷：忽略伺服器 IP/HTTP，上傳不執行，直接在 Serial 輸出連續讀值與錯誤分類
  void beginStandalone();
  void tick();  // 在主迴圈呼叫，處理讀取與上傳

  bool isReady();
  void setServerIP(IPAddress ip);

}  // namespace ImuStream

#endif  // IMU_STREAM_H
