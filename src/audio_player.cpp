/**
 * 語音播放實作 - HTTP 下載 + I2S (MAX98357A)
 */

#include "audio_player.h"
#include "config.h"

#if AUDIO_I2S_ENABLE
#include "Audio.h"
#include <esp_task_wdt.h>
#endif

namespace AudioPlayer {

#if AUDIO_I2S_ENABLE
  static Audio* audio = nullptr;
#endif

  static void buildAudioUrl(IPAddress serverIP, char* out, size_t outLen) {
#if CLOUD_MODE
    snprintf(out, outLen, "https://%s%s", SERVER_HOST, API_AUDIO_PATH);
    (void)serverIP;
#elif SERVER_HTTP_PORT == 80
    snprintf(out, outLen, "http://%d.%d.%d.%d%s",
             serverIP[0], serverIP[1], serverIP[2], serverIP[3], API_AUDIO_PATH);
#else
    snprintf(out, outLen, "http://%d.%d.%d.%d:%d%s",
             serverIP[0], serverIP[1], serverIP[2], serverIP[3], SERVER_HTTP_PORT, API_AUDIO_PATH);
#endif
  }

  void begin() {
#if !AUDIO_I2S_ENABLE
    Serial.println("[AUDIO] I2S speaker disabled (no amp/speaker)");
#else
    if (audio) return;
    esp_task_wdt_reset();
    audio = new Audio(false, 3, I2S_NUM_1);
    esp_task_wdt_reset();
    if (!audio) {
      Serial.println("[AUDIO] alloc failed");
      return;
    }
    audio->setPinout(I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DOUT_PIN);
    esp_task_wdt_reset();
    audio->setVolume(21);  // 0-21
    Serial.println("[AUDIO] Ready");
#endif
  }

  void tick() {
#if AUDIO_I2S_ENABLE
    if (audio) {
      audio->loop();
    }
#endif
  }

  void playFromServer(IPAddress serverIP) {
#if AUDIO_I2S_ENABLE
    if (!audio) return;
    char url[96];
    buildAudioUrl(serverIP, url, sizeof(url));
    Serial.print("[AUDIO] I2S Playing: ");
    Serial.println(url);
    audio->connecttohost(url);
#else
    (void)serverIP;
#endif
  }

  void setVolume(uint8_t vol) {
#if AUDIO_I2S_ENABLE
    if (audio && vol <= 21) {
      audio->setVolume((int)vol);
      Serial.printf("[AUDIO] Volume changed to %d\n", (int)vol);
    }
#else
    (void)vol;
#endif
  }

  bool isPlaying() {
#if AUDIO_I2S_ENABLE
    if (!audio) return false;
    return audio->isRunning();
#else
    return false;
#endif
  }

}  // namespace AudioPlayer
