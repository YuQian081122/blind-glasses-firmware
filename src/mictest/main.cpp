#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>

#include "Audio.h"
#include "config.h"

namespace {

constexpr i2s_port_t kMicPort = I2S_NUM_0;
constexpr int kMicBckPin = -1;
constexpr int kMicWsPin = 42;
constexpr int kMicDataPin = 41;
constexpr uint32_t kRecordSec = 3;
constexpr uint32_t kDefaultIntervalMs = 0;
constexpr uint32_t kCommandPollIntervalMs = 1000;
constexpr uint32_t kReplyWaitTimeoutMs = 30000;
constexpr uint32_t kReplyPollIntervalMs = 1000;
constexpr const char* kMictestPath = "/api/mictest";
constexpr const char* kCommandPath = "/api/mictest/command";
constexpr const char* kReplyPath = "/api/mictest/reply.mp3";

bool micReady = false;
uint32_t intervalMs = kDefaultIntervalMs;
uint32_t lastRunMs = 0;
uint32_t lastCommandPollMs = 0;
uint32_t lastHandledRecordRequestId = 0;
String serialLine;
String lastReplyEtag;
Audio* audio = nullptr;

void buildUrl(char* url, size_t len, const char* path) {
#if SERVER_USE_HTTPS
    snprintf(url, len, "https://%s%s", SERVER_HOST, path);
#else
    snprintf(url, len, "http://%s:%u%s", SERVER_HOST, SERVER_HTTP_PORT, path);
#endif
}

bool beginHttp(HTTPClient& http, WiFiClient& plainClient, WiFiClientSecure& secureClient, const char* url) {
#if SERVER_USE_HTTPS
    secureClient.setInsecure();
    return http.begin(secureClient, url);
#else
    (void)secureClient;
    return http.begin(plainClient, url);
#endif
}

void addDeviceToken(HTTPClient& http) {
    if (DEVICE_API_TOKEN[0]) {
        http.addHeader("X-Device-Token", DEVICE_API_TOKEN);
    }
}

void writeWavHeader(uint8_t* buf, uint32_t sampleRate, uint32_t numSamples) {
    const uint32_t byteRate = sampleRate * 2;
    const uint32_t dataSize = numSamples * 2;
    const uint32_t fileSize = 36 + dataSize;

    buf[0] = 'R'; buf[1] = 'I'; buf[2] = 'F'; buf[3] = 'F';
    buf[4] = fileSize & 0xff;
    buf[5] = (fileSize >> 8) & 0xff;
    buf[6] = (fileSize >> 16) & 0xff;
    buf[7] = (fileSize >> 24) & 0xff;
    buf[8] = 'W'; buf[9] = 'A'; buf[10] = 'V'; buf[11] = 'E';
    buf[12] = 'f'; buf[13] = 'm'; buf[14] = 't'; buf[15] = ' ';
    buf[16] = 16; buf[17] = 0; buf[18] = 0; buf[19] = 0;
    buf[20] = 1; buf[21] = 0;
    buf[22] = 1; buf[23] = 0;
    buf[24] = sampleRate & 0xff;
    buf[25] = (sampleRate >> 8) & 0xff;
    buf[26] = (sampleRate >> 16) & 0xff;
    buf[27] = (sampleRate >> 24) & 0xff;
    buf[28] = byteRate & 0xff;
    buf[29] = (byteRate >> 8) & 0xff;
    buf[30] = (byteRate >> 16) & 0xff;
    buf[31] = (byteRate >> 24) & 0xff;
    buf[32] = 2; buf[33] = 0;
    buf[34] = 16; buf[35] = 0;
    buf[36] = 'd'; buf[37] = 'a'; buf[38] = 't'; buf[39] = 'a';
    buf[40] = dataSize & 0xff;
    buf[41] = (dataSize >> 8) & 0xff;
    buf[42] = (dataSize >> 16) & 0xff;
    buf[43] = (dataSize >> 24) & 0xff;
}

bool initMic() {
    i2s_config_t i2sConfig = {};
    i2sConfig.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    i2sConfig.sample_rate = MIC_SAMPLE_RATE;
    i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2sConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2sConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2sConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2sConfig.dma_buf_count = 4;
    i2sConfig.dma_buf_len = 512;
    i2sConfig.use_apll = false;
    i2sConfig.tx_desc_auto_clear = false;
    i2sConfig.fixed_mclk = 0;

    i2s_pin_config_t pinConfig = {};
    pinConfig.bck_io_num = kMicBckPin;
    pinConfig.ws_io_num = kMicWsPin;
    pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
    pinConfig.data_in_num = kMicDataPin;

    if (i2s_driver_install(kMicPort, &i2sConfig, 0, nullptr) != ESP_OK) {
        Serial.println("[MICTEST] mic i2s install failed");
        return false;
    }
    if (i2s_set_pin(kMicPort, &pinConfig) != ESP_OK) {
        Serial.println("[MICTEST] mic i2s pin failed");
        i2s_driver_uninstall(kMicPort);
        return false;
    }
    i2s_zero_dma_buffer(kMicPort);
    Serial.printf("[MICTEST] mic ready rate=%u data=GPIO%d clk=GPIO%d\n", MIC_SAMPLE_RATE, kMicDataPin, kMicWsPin);
    return true;
}

bool initAudio() {
    if (audio) return true;
    audio = new Audio(false, 3, I2S_NUM_1);
    if (!audio) {
        Serial.println("[MICTEST] audio alloc failed");
        return false;
    }
    audio->setPinout(I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DOUT_PIN);
    audio->setVolume(21);
    Serial.printf("[MICTEST] audio ready bclk=GPIO%d lrc=GPIO%d dout=GPIO%d\n",
                  I2S_BCLK_PIN,
                  I2S_LRC_PIN,
                  I2S_DOUT_PIN);
    return true;
}

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[MICTEST] wifi connecting ssid=%s\n", WIFI_SSID);

    uint32_t startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < 30000) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[MICTEST] wifi connected ip=%s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("[MICTEST] wifi connect timeout");
    }
}

bool recordWav(uint8_t*& wavBuffer, size_t& wavLen, size_t& pcmBytes) {
    wavBuffer = nullptr;
    wavLen = 0;
    pcmBytes = 0;

    const size_t pcmSize = static_cast<size_t>(MIC_SAMPLE_RATE) * kRecordSec * 2;
    uint8_t* pcmBuffer = static_cast<uint8_t*>(heap_caps_malloc(pcmSize, MALLOC_CAP_SPIRAM));
    if (!pcmBuffer) pcmBuffer = static_cast<uint8_t*>(malloc(pcmSize));
    if (!pcmBuffer) {
        Serial.println("[MICTEST] pcm malloc failed");
        return false;
    }

    Serial.printf("[MICTEST] recording sec=%u target_pcm=%u\n", kRecordSec, static_cast<unsigned>(pcmSize));
    const uint32_t startedMs = millis();
    while (pcmBytes < pcmSize && millis() - startedMs < (kRecordSec * 1000UL + 1000UL)) {
        size_t readBytes = 0;
        size_t toRead = min(static_cast<size_t>(512), pcmSize - pcmBytes);
        i2s_read(kMicPort, pcmBuffer + pcmBytes, toRead, &readBytes, pdMS_TO_TICKS(100));
        pcmBytes += readBytes;
        delay(1);
    }

    wavLen = 44 + pcmBytes;
    wavBuffer = static_cast<uint8_t*>(malloc(wavLen));
    if (!wavBuffer) {
        Serial.println("[MICTEST] wav malloc failed");
        free(pcmBuffer);
        return false;
    }

    writeWavHeader(wavBuffer, MIC_SAMPLE_RATE, pcmBytes / 2);
    memcpy(wavBuffer + 44, pcmBuffer, pcmBytes);
    free(pcmBuffer);

    Serial.printf("[MICTEST] record done pcm_bytes=%u wav_bytes=%u elapsed_ms=%u\n",
                  static_cast<unsigned>(pcmBytes),
                  static_cast<unsigned>(wavLen),
                  static_cast<unsigned>(millis() - startedMs));
    return pcmBytes > 0;
}

int postMictestWav(const uint8_t* data, size_t len, uint32_t& elapsedMs) {
    char url[160];
    buildUrl(url, sizeof(url), kMictestPath);
    WiFiClient plainClient;
    WiFiClientSecure secureClient;

    HTTPClient http;
    uint32_t startedMs = millis();
    if (!beginHttp(http, plainClient, secureClient, url)) {
        elapsedMs = millis() - startedMs;
        return -1000;
    }

    http.addHeader("Content-Type", "audio/wav");
    addDeviceToken(http);
    http.setTimeout(15000);
    int code = http.POST(const_cast<uint8_t*>(data), len);
    (void)http.getString();
    http.end();
    elapsedMs = millis() - startedMs;
    return code;
}

bool fetchRecordCommand(uint32_t& requestId, String& status, int& statusCode) {
    char url[160];
    buildUrl(url, sizeof(url), kCommandPath);
    WiFiClient plainClient;
    WiFiClientSecure secureClient;
    HTTPClient http;

    if (!beginHttp(http, plainClient, secureClient, url)) {
        statusCode = -1000;
        return false;
    }
    addDeviceToken(http);
    http.setTimeout(5000);
    statusCode = http.GET();
    String body = http.getString();
    http.end();

    if (statusCode != 200) {
        return false;
    }

    StaticJsonDocument<160> doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        Serial.printf("[MICTEST] command json error=%s\n", err.c_str());
        return false;
    }
    requestId = doc["record_request_id"] | 0;
    status = doc["record_status"] | "";
    return true;
}

bool replyReady(String& etag, uint32_t& elapsedMs, int& statusCode) {
    char url[160];
    buildUrl(url, sizeof(url), kReplyPath);
    const uint32_t startedMs = millis();

    while (millis() - startedMs < kReplyWaitTimeoutMs) {
        WiFiClient plainClient;
        WiFiClientSecure secureClient;
        HTTPClient http;
        if (!beginHttp(http, plainClient, secureClient, url)) {
            statusCode = -1000;
            elapsedMs = millis() - startedMs;
            return false;
        }
        addDeviceToken(http);
        if (lastReplyEtag.length() > 0) {
            http.addHeader("If-None-Match", lastReplyEtag);
        }
        const char* headerKeys[] = {"ETag"};
        http.collectHeaders(headerKeys, 1);
        http.setTimeout(8000);
        statusCode = http.GET();
        etag = http.header("ETag");
        http.end();

        if (statusCode == 200 && etag.length() > 0 && etag != lastReplyEtag) {
            elapsedMs = millis() - startedMs;
            return true;
        }
        Serial.printf("[MICTEST] reply wait code=%d elapsed_ms=%u\n",
                      statusCode,
                      static_cast<unsigned>(millis() - startedMs));
        delay(kReplyPollIntervalMs);
    }

    elapsedMs = millis() - startedMs;
    return false;
}

void playReply(const String& etag, uint32_t waitMs) {
    if (!audio) {
        Serial.println("[MICTEST] playback skipped: audio_not_ready");
        return;
    }

    char url[180];
    buildUrl(url, sizeof(url), kReplyPath);
    Serial.printf("[MICTEST] playback start wait_ms=%u etag=%s\n",
                  static_cast<unsigned>(waitMs),
                  etag.c_str());
    audio->connecttohost(url);

    const uint32_t startedMs = millis();
    bool sawRunning = false;
    while (millis() - startedMs < 20000) {
        audio->loop();
        if (audio->isRunning()) {
            sawRunning = true;
        } else if (sawRunning) {
            break;
        }
        delay(1);
    }
    lastReplyEtag = etag;
    Serial.printf("[MICTEST] playback done elapsed_ms=%u ran=%s\n",
                  static_cast<unsigned>(millis() - startedMs),
                  sawRunning ? "yes" : "no");
}

void runCaptureUpload() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWifi();
    }
    if (WiFi.status() != WL_CONNECTED || !micReady) {
        Serial.println("[MICTEST] skip capture: wifi_or_mic_not_ready");
        return;
    }

    uint8_t* wavBuffer = nullptr;
    size_t wavLen = 0;
    size_t pcmBytes = 0;
    if (!recordWav(wavBuffer, wavLen, pcmBytes)) {
        free(wavBuffer);
        return;
    }

    uint32_t uploadMs = 0;
    int code = postMictestWav(wavBuffer, wavLen, uploadMs);
    Serial.printf("[MICTEST] POST /api/mictest code=%d upload_ms=%u bytes=%u\n",
                  code,
                  static_cast<unsigned>(uploadMs),
                  static_cast<unsigned>(wavLen));
    free(wavBuffer);

    if (code > 0 && code < 400) {
        String replyEtag;
        uint32_t replyWaitMs = 0;
        int replyCode = 0;
        if (replyReady(replyEtag, replyWaitMs, replyCode)) {
            playReply(replyEtag, replyWaitMs);
        } else {
            Serial.printf("[MICTEST] reply timeout code=%d wait_ms=%u\n",
                          replyCode,
                          static_cast<unsigned>(replyWaitMs));
        }
    }
}

void pollRecordCommand() {
    if (intervalMs > 0) return;
    if (WiFi.status() != WL_CONNECTED || !micReady) return;
    if (audio && audio->isRunning()) return;
    if (millis() - lastCommandPollMs < kCommandPollIntervalMs) return;
    lastCommandPollMs = millis();

    uint32_t requestId = 0;
    String status;
    int code = 0;
    if (!fetchRecordCommand(requestId, status, code)) {
        Serial.printf("[MICTEST] command poll code=%d\n", code);
        return;
    }
    if (requestId > lastHandledRecordRequestId && status == "queued") {
        lastHandledRecordRequestId = requestId;
        Serial.printf("[MICTEST] command trigger id=%u\n", static_cast<unsigned>(requestId));
        runCaptureUpload();
        lastRunMs = millis();
    }
}

void handleSerialCommand(const String& line) {
    String cmd = line;
    cmd.trim();
    if (cmd == "r") {
        Serial.println("[MICTEST] serial trigger");
        runCaptureUpload();
        lastRunMs = millis();
        return;
    }
    if (cmd.startsWith("i ")) {
        long nextInterval = cmd.substring(2).toInt();
        if (nextInterval == 0) {
            intervalMs = 0;
            Serial.println("[MICTEST] interval disabled; waiting for web trigger");
        } else if (nextInterval >= 1000) {
            intervalMs = static_cast<uint32_t>(nextInterval);
            Serial.printf("[MICTEST] interval_ms=%u\n", static_cast<unsigned>(intervalMs));
        } else {
            Serial.println("[MICTEST] interval ignored, use 0 or >=1000");
        }
        return;
    }
    if (cmd.length() > 0) {
        Serial.println("[MICTEST] commands: r | i <ms>");
    }
}

void pollSerial() {
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            handleSerialCommand(serialLine);
            serialLine = "";
        } else {
            serialLine += c;
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[MICTEST] boot");
    connectWifi();
    micReady = initMic();
    initAudio();
    lastRunMs = millis();
    Serial.println("[MICTEST] web trigger ready; commands: r | i <ms> | i 0");
}

void loop() {
    pollSerial();
    if (audio) {
        audio->loop();
    }
    pollRecordCommand();
    if (intervalMs > 0 && millis() - lastRunMs >= intervalMs) {
        lastRunMs = millis();
        runCaptureUpload();
    }
    delay(10);
}
