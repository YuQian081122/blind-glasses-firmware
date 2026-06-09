/**
 * 智慧導盲眼鏡 - 韌體設定
 * 請依實際接線修改 WiFi 與 GPIO
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============ WiFi ============
#define WIFI_SSID       "奇異鳥吃飛魚"
#define WIFI_PASSWORD   "11091122"
#define WIFI_MAX_RETRY  35   // 每次嘗試約 0.5s；BLE 先開時建議 ≥30

// ============ UDP 探索 ============
#define UDP_DISCOVERY_PORT     9999
#define UDP_DISCOVERY_MSG      "WHO_IS_SERVER"
#define UDP_DISCOVERY_PREFIX   "SERVER_IP: "
#define UDP_DISCOVERY_INTERVAL_MS      3000   // 已找到伺服器時
#define UDP_DISCOVERY_INTERVAL_SLOW_MS 8000   // 省電且尚未找到伺服器時

// ============ HTTP 串流 ============
#define STREAM_PORT     81
#define STREAM_PATH     "/stream"

// ============ 相機 OV3660 SCCB（I2C，與板載腳位表一致）============
#define CAMERA_SIOD_PIN     40   // Camera I2C SDA
#define CAMERA_SIOC_PIN     39   // Camera I2C SCL

// ============ 操作模式 ============
// 0 = 單一按鈕模式 (短/長/雙擊/三擊區分功能)
// 1 = 持續監測模式 (按鈕僅語音觸發，避障由伺服器持續處理)
#define OP_MODE_SINGLE_BTN  0
#define OP_MODE_ALWAYS_ON   1
// NVS 尚無紀錄時的開機預設（曾用切換鍵切過則以 NVS 為準；若要強制改回持續監測可清除快閃或短按切換鍵切一次）
#define OP_MODE_DEFAULT     OP_MODE_ALWAYS_ON

// ============ 按鈕（與 Arduino/include/config.h 自訂接線一致；I2S 佔 D0–D2）============
#define BTN_POWER_PIN       35   // 電源鍵
#define BTN_MODE_PIN        37   // 切換鍵
#define BTN_SINGLE_PIN      BTN_POWER_PIN
#define BTN_DEBOUNCE_MS     50
#define BTN_POWER_HOLD_MS   5000  // 電源鍵長按 5 秒 = 開/關機；切換鍵長按 5 秒 = 語音助理

// ============ I2S (MAX98357A) ============
// 以下為目前實體接線；喇叭 I2S1 與板載 PDM 麥克風 I2S0 分開
#define I2S_DOUT_PIN    1
#define I2S_BCLK_PIN    3
#define I2S_LRC_PIN     2

// ============ GPIO LED 測試 ============
// 1=在 loop 中每 2 秒切換測試腳位高低電位，方便接 LED 觀察輸出
#define GPIO1_TOGGLE_TEST_ENABLE      0
#define GPIO1_TOGGLE_TEST_PIN         9    // D10 (GPIO9)
#define GPIO1_TOGGLE_INTERVAL_MS      2000

// ============ IMU (ICM-20948) ============
#define IMU_SDA_PIN        7
#define IMU_SCL_PIN        8
#define IMU_I2C_ADDR       0x68 // AD0=LOW；若模組 AD0 接 VCC 則為 0x69，begin 時 ad0val=true
#define IMU_SEND_INTERVAL_MS    100   // 一般
#define IMU_SEND_INTERVAL_PS_MS 200   // 省電時拉長（POWER_SAVE_ENABLE）

// ============ IMU 卡爾曼融合（加速度傾角 + 陀螺儀）============
// 1=啟用；0=僅上傳原始六軸
#define IMU_KALMAN_ENABLE       1
// 參數說明見 https://jasonblog.github.io/note/osvr/qia_er_man_lv_bo_pei_he_cheng_shi_jiang_jie.html
#define IMU_KALMAN_Q_ANGLE      0.01f   // 角度過程噪聲
#define IMU_KALMAN_Q_GYRO       0.01f   // 陀螺儀零偏過程噪聲
#define IMU_KALMAN_R_ANGLE      0.003f  // 加速度計傾角量測噪聲（愈大愈不信加速度）

// ============ IMU 單獨測試（診斷用） ============
// 1=開啟「只測 IMU」模式：不啟用 WiFi/伺服器 API，只做 I2C 掃描與連續 gyro/acc 讀值輸出
// 0=維持原本功能
#define IMU_STANDALONE_TEST    0
#define IMU_TEST_OUTPUT_INTERVAL_MS        100   // standalone 每次輸出間隔
#define IMU_TEST_FAIL_THROTTLE_MS          2000  // 讀取失敗/初始化失敗的節流輸出

// ============ 麥克風 (PDM) ============
// MIC_RECORD_SEC：錄音秒數，直接影響延遲與 ASR 辨識率
//   - 2 秒：低延遲模式，適合安靜/短指令（如「紅綠燈」「現在在哪」）
//   - 3 秒：均衡模式，噪音稍多時仍可辨識完整句子
//   - 4 秒：高穩定模式，嘈雜環境（馬路邊）建議使用
#define MIC_RECORD_SEC     2
#define MIC_SAMPLE_RATE    16000

// 無按鍵時自動測麥克風→ASR→TTS 全鏈路（1=啟用；正式使用請改 0）
#define MIC_AUTO_TEST_ENABLE        0
#define MIC_AUTO_TEST_INTERVAL_MS   15000

// ============ 按鈕 - 單一按鈕手勢 ============
#define BTN_ITEM_PIN       -1
#define BTN_DOUBLE_CLICK_MS  400
#define BTN_TRIPLE_CLICK_MS  600   // 三擊間隔

// ============ 伺服器 API ============
// --- 雲端模式（Cloudflare Tunnel） ---
// CLOUD_MODE=1：ESP32 透過 HTTPS 連到 blind-glasses.org（不需 UDP 探索）
// CLOUD_MODE=0：區網模式，維持 UDP 探索 + HTTP 直連 IP
#define CLOUD_MODE          1

#if CLOUD_MODE
  #define SERVER_HOST         "blind-glasses.org"
  #define SERVER_HTTP_PORT    443
  #define SERVER_USE_HTTPS    1
  #define DEVICE_API_TOKEN    "0QchQE-fzMg5yg-1GHu-3-J7tfgqtsDA2J-pKPcMBu4"   // 須與 server/.env 的 DEVICE_API_TOKEN 相同
  // 非空時：每次 IMU 帶此「網際網路可連」的 MJPEG 完整 URL（例：ngrok / trycloudflare 指到眼鏡 :81/stream）。勿填 blind-glasses.org/stream。
  // 空 + DEVICE_STREAM_REPORT_LAN_URL=1：自動帶 http://區網IP:81/stream（僅當「跑伺服器的電腦」與眼鏡同一區網時，家裡監控才拉得到畫面）。
  // 帶出門連別人 WiFi、又無法改伺服器：家裡伺服器無法連對方區網 IP；要畫面只能在此填「公開 tunnel URL」並重燒韌體，或接受僅 API/無監控影像。
  #define DEVICE_PUBLIC_STREAM_URL    ""
  // 1=且 DEVICE_PUBLIC_STREAM_URL 為空時，自動帶區網串流 URL 給伺服器
  #define DEVICE_STREAM_REPORT_LAN_URL  1
#else
  #define SERVER_HOST         ""   // 區網模式不使用 hostname（由 UDP 探索取得 IP）
  #define SERVER_HTTP_PORT    5000
  #define SERVER_USE_HTTPS    0
  #define DEVICE_API_TOKEN    "0QchQE-fzMg5yg-1GHu-3-J7tfgqtsDA2J-pKPcMBu4"
  #define DEVICE_PUBLIC_STREAM_URL    ""
  #define DEVICE_STREAM_REPORT_LAN_URL  0
#endif

#define API_GEMINI_PATH     "/api/gemini"
#define API_ASR_PATH        "/api/asr"
#define API_IMU_PATH        "/api/imu"
#define API_GPS_PATH        "/api/gps"
#define API_FRAME_PATH      "/api/frame"

// ============ 相機（推幀與 :81/stream 共用）============
// esp32-camera：jpeg_quality 數值越小畫質越好、單幀越大、上傳越慢（約 4–63，建議 8–15）
#define CAMERA_JPEG_QUALITY         10
// 1=VGA 640×480（較清晰）；0=QVGA 320×240（較省頻寬、推幀較快）
#define CAMERA_FRAMESIZE_VGA        1

// ============ 影像推幀（Frame Push）============
// 眼鏡主動 POST JPEG 到伺服器，不需伺服器能連到眼鏡區網；出門連任何有網的 WiFi 都有畫面
#define FRAME_PUSH_ENABLE         1
// 間隔越小幀率越高；實際 fps 受 JPEG 大小與 HTTPS 上傳時間限制（VGA 約 150–250ms 較常見）
#define FRAME_PUSH_INTERVAL_MS    150
// 高解析 JPEG + 雲端 TLS 單次 POST 可能較久，勿與 IMU 等短逾時共用
#define FRAME_PUSH_HTTP_TIMEOUT_MS  15000
// HardwareSerial.begin(baud, cfg, RX, TX) — 見 gps_stream.cpp
// D6/D7（Seeed XIAO ESP32-S3）：TX=GPIO43、RX=GPIO44；HardwareSerial.begin(..., RX, TX) 見 gps_stream.cpp
#define GPS_TX_PIN          43   // D6 → ESP UART TX → 模組 RX
#define GPS_RX_PIN          44   // D7 → ESP UART RX ← 模組 TX
#define GPS_BAUD            9600
#define GPS_SEND_INTERVAL_MS    5000   // 一般 5 秒
#define GPS_SEND_INTERVAL_PS_MS 15000  // 省電時 15 秒上傳一次

// 僅使用相機+陀螺儀時：可關閉 GPS UART；音訊測試時請開啟 I2S 喇叭
#define GPS_ENABLE          1   // 1=啟用 NEO-M8N 串流；0=不初始化 GPS UART
#define AUDIO_I2S_ENABLE    1   // 1=MAX98357A 播放；0=不佔用 I2S1（僅測 WiFi 時建議 0）

// ============ 音訊自動測試（不需按鈕） ============
// 1=每隔固定秒數自動抓一次 /audio/latest 測 MAX98357 播放鏈路
#define AUDIO_AUTO_TEST_ENABLE       0
#define AUDIO_AUTO_TEST_INTERVAL_MS  10000

#define API_AUDIO_PATH      "/audio/latest"
// API_TIMEOUT_MS：HTTP 請求超時
//   - 5000：低延遲模式，快速失敗重試（WiFi 良好時推薦）
//   - 8000：均衡模式，容許偶爾的網路抖動
//   - 12000：高穩定模式，弱網環境避免頻繁超時
#define API_TIMEOUT_MS      5000
#define AUDIO_POLL_INTERVAL_MS  500

// ============ BLE 快速連接（S3 可用） ============
// 出廠／實測／搭配 App 掃描配網：必須為 1，否則手機掃不到 BLE、無 GATT。
// 僅本機除錯「只要 WiFi、不要 BLE」時可暫改 0（少一個射頻、Serial 較乾淨）。
#define BLE_QUICK_LINK_ENABLE         1
#define BLE_DEVICE_NAME               "BlindGlasses-S3"
#define BLE_SERVICE_UUID              "6f2f6d30-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_WIFI_SSID_UUID            "6f2f6d31-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_WIFI_PASS_UUID            "6f2f6d32-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_WIFI_APPLY_UUID           "6f2f6d33-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_FIND_ME_UUID              "6f2f6d34-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_STATUS_UUID               "6f2f6d35-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_MODE_UUID                 "6f2f6d36-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_VOLUME_UUID               "6f2f6d37-4d57-4c76-a5dd-86f4d2a06340"
#define BLE_STATUS_NOTIFY_MS          1000

// ============ 相機 ============
// 0=完全不初始化相機（僅測 WiFi／除錯）；1=依下方與模式決定何時開
#define CAMERA_ENABLE         1
// 1=開機且 WiFi 連上後立即啟動串流（即使省電 + 單一按鈕模式）；0=維持省電延後開相機（按鍵或切換持續監測後才開）
#define CAMERA_START_ON_BOOT  1

// ============ 省電 ============
#define POWER_SAVE_ENABLE   0     // 1=啟用省電（單一按鈕時不開相機、WiFi modem sleep、閒置延遲）
// 若監控頁陀螺儀永遠為「-」，常因 modem sleep 導致 POST /api/imu 失敗；先設 0 再試
#define WIFI_MODEM_SLEEP    0     // 1=WiFi 閒置 modem sleep（易影響 IMU HTTP 上傳）
#define LOOP_IDLE_MS        50    // 無工作時 loop 延遲 ms（省電時拉長，預設 50）
#define LOOP_IDLE_MS_NORMAL 10    // 未開省電時延遲

#endif // CONFIG_H
