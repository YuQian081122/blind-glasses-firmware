/**
 * 麥克風錄音上傳 - XIAO ESP32-S3 Sense PDM 麥克風 (GPIO 41/42)
 */

#include "mic_upload.h"
#include "config.h"
#if !CLOUD_MODE
#include "udp_discovery.h"
#endif
#include "server_api.h"
#include "http_task_queue.h"
#include <WiFi.h>

#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

// XIAO Sense PDM: DATA=41, CLK=42
// ESP32-S3：PDM RX 僅支援 I2S0；喇叭 (ESP32-audioI2S) 須用 I2S1，見 audio_player.cpp
#define I2S_MIC_PORT       I2S_NUM_0
#define I2S_MIC_BCK_PIN    -1
#define I2S_MIC_WS_PIN     42
#define I2S_MIC_DATA_PIN   41

namespace MicUpload {

  static bool micDriverOk = false;
  static bool recording = false;
  static unsigned long recordStartTime = 0;
  static uint8_t* pcmBuffer = nullptr;
  static size_t pcmBytesRecorded = 0;
  static bool pendingAudioFetch = false;
  static unsigned long uploadCompleteTime = 0;
  static unsigned long taskDoneTime = 0;

  static void writeWavHeader(uint8_t* buf, uint32_t sampleRate, uint32_t numSamples) {
    uint32_t byteRate = sampleRate * 2;  // 16-bit mono
    uint32_t dataSize = numSamples * 2;
    uint32_t fileSize = 36 + dataSize;

    buf[0] = 'R'; buf[1] = 'I'; buf[2] = 'F'; buf[3] = 'F';
    buf[4] = fileSize & 0xff;
    buf[5] = (fileSize >> 8) & 0xff;
    buf[6] = (fileSize >> 16) & 0xff;
    buf[7] = (fileSize >> 24) & 0xff;
    buf[8] = 'W'; buf[9] = 'A'; buf[10] = 'V'; buf[11] = 'E';
    buf[12] = 'f'; buf[13] = 'm'; buf[14] = 't'; buf[15] = ' ';
    buf[16] = 16; buf[17] = 0; buf[18] = 0; buf[19] = 0;  // fmt chunk 16
    buf[20] = 1; buf[21] = 0;  // PCM
    buf[22] = 1; buf[23] = 0;  // mono
    buf[24] = sampleRate & 0xff;
    buf[25] = (sampleRate >> 8) & 0xff;
    buf[26] = (sampleRate >> 16) & 0xff;
    buf[27] = (sampleRate >> 24) & 0xff;
    buf[28] = byteRate & 0xff;
    buf[29] = (byteRate >> 8) & 0xff;
    buf[30] = (byteRate >> 16) & 0xff;
    buf[31] = (byteRate >> 24) & 0xff;
    buf[32] = 2; buf[33] = 0;  // block align
    buf[34] = 16; buf[35] = 0; // bits per sample
    buf[36] = 'd'; buf[37] = 'a'; buf[38] = 't'; buf[39] = 'a';
    buf[40] = dataSize & 0xff;
    buf[41] = (dataSize >> 8) & 0xff;
    buf[42] = (dataSize >> 16) & 0xff;
    buf[43] = (dataSize >> 24) & 0xff;
  }

  void begin() {
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    i2s_config.sample_rate = MIC_SAMPLE_RATE;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = 4;
    i2s_config.dma_buf_len = 512;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = false;
    i2s_config.fixed_mclk = 0;

    i2s_pin_config_t pin_config = {};
    pin_config.bck_io_num = I2S_MIC_BCK_PIN;
    pin_config.ws_io_num = I2S_MIC_WS_PIN;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num = I2S_MIC_DATA_PIN;

    esp_task_wdt_reset();
    if (i2s_driver_install(I2S_MIC_PORT, &i2s_config, 0, NULL) != ESP_OK) {
      Serial.println("[MIC] I2S install failed");
      return;
    }
    esp_task_wdt_reset();
    if (i2s_set_pin(I2S_MIC_PORT, &pin_config) != ESP_OK) {
      Serial.println("[MIC] I2S pin failed");
      i2s_driver_uninstall(I2S_MIC_PORT);
      return;
    }
    esp_task_wdt_reset();
    i2s_zero_dma_buffer(I2S_MIC_PORT);
    esp_task_wdt_reset();
    micDriverOk = true;
    Serial.println("[MIC] Ready");
  }

  bool isRecording() {
    return recording;
  }

  bool hasPendingAudioFetch() {
    return pendingAudioFetch;
  }

  void clearPendingAudioFetch() {
    pendingAudioFetch = false;
    taskDoneTime = 0;
  }

  bool shouldFetchAudio() {
    if (!pendingAudioFetch || uploadCompleteTime == 0) return false;
    if (HttpTaskQueue::isBusy()) return false;
    if (!HttpTaskQueue::hasCompletedSinceEnqueue()) {
      return (millis() - uploadCompleteTime) >= 8000;
    }
    if (taskDoneTime == 0) taskDoneTime = millis();
    return (millis() - taskDoneTime) >= 300;
  }

  void startRecording() {
#if CLOUD_MODE
    if (!micDriverOk || WiFi.status() != WL_CONNECTED || recording) return;
#else
    if (!micDriverOk || !UdpDiscovery::hasServer() || recording) return;
#endif

    const size_t pcmSize = (size_t)MIC_SAMPLE_RATE * MIC_RECORD_SEC * 2;
    pcmBuffer = (uint8_t*)heap_caps_malloc(pcmSize, MALLOC_CAP_SPIRAM);
    if (!pcmBuffer) pcmBuffer = (uint8_t*)malloc(pcmSize);
    if (!pcmBuffer) {
      Serial.println("[MIC] malloc failed");
      return;
    }
    pcmBytesRecorded = 0;
    recording = true;
    recordStartTime = millis();
    Serial.println("[MIC] Recording...");
  }

  void tick() {
    if (!micDriverOk || !recording || !pcmBuffer) return;

    const size_t pcmSize = (size_t)MIC_SAMPLE_RATE * MIC_RECORD_SEC * 2;
    size_t readBytes = 0;
    size_t toRead = 256;
    if (pcmBytesRecorded + toRead > pcmSize) toRead = pcmSize - pcmBytesRecorded;
    if (toRead == 0) return;

    i2s_read(I2S_MIC_PORT, pcmBuffer + pcmBytesRecorded, toRead, &readBytes, 10);
    pcmBytesRecorded += readBytes;

    unsigned long elapsed = millis() - recordStartTime;
    if (elapsed >= MIC_RECORD_SEC * 1000 || pcmBytesRecorded >= pcmSize) {
      recording = false;
      Serial.println("[MIC] Record done, uploading...");

      const uint32_t numSamples = pcmBytesRecorded / 2;
      const size_t wavLen = 44 + pcmBytesRecorded;
      uint8_t* wavBuffer = (uint8_t*)malloc(wavLen);
      if (!wavBuffer) {
        free(pcmBuffer);
        pcmBuffer = nullptr;
        return;
      }
      writeWavHeader(wavBuffer, MIC_SAMPLE_RATE, numSamples);
      memcpy(wavBuffer + 44, pcmBuffer, pcmBytesRecorded);

      HttpTaskQueue::clearCompleted();
#if CLOUD_MODE
      HttpTaskQueue::enqueueAsr(IPAddress(1, 1, 1, 1), wavBuffer, wavLen);
#else
      HttpTaskQueue::enqueueAsr(UdpDiscovery::getServerIP(), wavBuffer, wavLen);
#endif
      pendingAudioFetch = true;
      uploadCompleteTime = millis();
      taskDoneTime = 0;
      free(wavBuffer);
      free(pcmBuffer);
      pcmBuffer = nullptr;
    }
  }

}  // namespace MicUpload
