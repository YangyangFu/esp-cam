/**********************************************************************
  Filename    : Fridge CAM
  Description : ESP32 camera that records MJPEG video to SD card when
                a Shelly BLU Door/Window sensor detects the fridge
                door opening. Recording stops when the door closes.
                Serves a web UI with door status, live stream, and
                recorded video downloads.
  Board       : ESP32 Wrover (with PSRAM)
**********************************************************************/
#include "esp_camera.h"
#include <WiFi.h>
#include <NimBLEDevice.h>
#include "SD_MMC.h"
#include <time.h>
#include "board_config.h"
#include "avi_writer.h"

// ========================
// WiFi credentials
// ========================
const char *ssid_Router     = "xyz";
const char *password_Router = "yyf_yz_2024";

// ========================
// Shelly BLU Door/Window
// ========================
// Set this to your sensor's BLE MAC address (empty string = accept any SBDW device)
const char *SHELLY_BLU_MAC = "";

// BTHome v2 service UUID (16-bit: 0xFCD2)
static NimBLEUUID bthomeServiceUUID((uint16_t)0xFCD2);

// ========================
// SD card pins (1-bit SDMMC)
// ========================
#define SD_MMC_CMD 15
#define SD_MMC_CLK 14
#define SD_MMC_D0  2

// ========================
// Recording settings
// ========================
#define RECORDING_FPS          6
#define RECORDING_MAX_MS       (5 * 60 * 1000)  // 5-minute safety cap
#define RECORDING_FRAME_DELAY  (1000 / RECORDING_FPS)

// ========================
// Shared state (accessed from BLE callback + loop)
// ========================
volatile bool     doorOpen        = false;
volatile uint8_t  batteryPercent  = 0;
volatile uint32_t lastEventTime   = 0;
volatile bool     requestStart    = false;  // BLE sets true on door open
volatile bool     requestStop     = false;  // BLE sets true on door close

// Recording state (only accessed from loop)
bool     recording       = false;
uint32_t recordingStart  = 0;
char     recordingPath[48] = {0};

// BLE scan control
#define BLE_SCAN_DURATION  5  // seconds per scan cycle
NimBLEScan *pBLEScan = NULL;

// Last captured photo for web UI
camera_fb_t *lastCapture     = NULL;
SemaphoreHandle_t captureMux = NULL;

bool sdCardOK = false;
camera_config_t config;

// ========================
// Forward declarations
// ========================
void startCameraServer();
void camera_init();
void parseBTHomePayload(const uint8_t *data, int len);
void startRecording();
void stopRecording();
void recordFrame();
void initBLE();
void deinitBLE();

// ========================
// BLE scan callback
// ========================
class ShellyBLECallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
    // Filter by MAC if set
    if (strlen(SHELLY_BLU_MAC) > 0) {
      if (strcasecmp(advertisedDevice->getAddress().toString().c_str(), SHELLY_BLU_MAC) != 0) {
        return;
      }
    } else {
      // Accept devices whose name starts with "SBDW"
      std::string name = advertisedDevice->getName();
      if (name.empty() || name.rfind("SBDW", 0) != 0) {
        if (!advertisedDevice->haveServiceData()) return;
        std::string svcData = advertisedDevice->getServiceData(bthomeServiceUUID);
        if (svcData.empty()) return;
      }
    }

    if (!advertisedDevice->haveServiceData()) return;
    std::string svcData = advertisedDevice->getServiceData(bthomeServiceUUID);
    if (svcData.empty()) return;

    // Debug: print device info on first detection
    static bool firstSeen = true;
    if (firstSeen) {
      Serial.printf("[BLE] Found Shelly sensor — MAC: %s, Name: %s, RSSI: %d\n",
        advertisedDevice->getAddress().toString().c_str(),
        advertisedDevice->getName().c_str(),
        advertisedDevice->getRSSI());
      firstSeen = false;
    }

    const uint8_t *data = (const uint8_t *)svcData.data();
    int len = svcData.length();
    if (len < 2) return;

    uint8_t deviceInfo = data[0];
    bool encrypted = deviceInfo & 0x01;
    uint8_t version = deviceInfo >> 5;
    if (version != 2 || encrypted) return;

    parseBTHomePayload(data + 1, len - 1);
  }
};

// ========================
// Parse BTHome v2 objects
// ========================
void parseBTHomePayload(const uint8_t *data, int len) {
  int idx = 0;
  bool prevDoorOpen = doorOpen;

  while (idx < len) {
    uint8_t objId = data[idx++];
    if (idx > len) break;

    switch (objId) {
      case 0x00:  // Packet ID (uint8)
        if (idx < len) idx++;
        break;
      case 0x01:  // Battery (uint8, %)
        if (idx < len) batteryPercent = data[idx++];
        break;
      case 0x05:  // Illuminance (uint24, factor 0.01)
        if (idx + 2 < len) idx += 3;
        else idx = len;
        break;
      case 0x2D:  // Window (uint8): 0=closed, 1=open
        if (idx < len) {
          doorOpen = (data[idx++] != 0);
          lastEventTime = millis();
        }
        break;
      case 0x1A:  // Door (uint8): 0=closed, 1=open
        if (idx < len) {
          doorOpen = (data[idx++] != 0);
          lastEventTime = millis();
        }
        break;
      case 0x11:  // Opening (uint8): 0=closed, 1=open
        if (idx < len) {
          doorOpen = (data[idx++] != 0);
          lastEventTime = millis();
        }
        break;
      case 0x3F:  // Rotation (int16, factor 0.1)
        if (idx + 1 < len) idx += 2;
        else idx = len;
        break;
      default:
        return;
    }
  }

  if (doorOpen && !prevDoorOpen) {
    requestStart = true;
    Serial.println("[BLE] Door OPENED");
  } else if (!doorOpen && prevDoorOpen) {
    requestStop = true;
    Serial.println("[BLE] Door CLOSED");
  }
}

// ========================
// Recording control
// ========================
void startRecording() {
  if (!sdCardOK) {
    Serial.println("[REC] No SD card — skipping");
    return;
  }

  // Stop BLE scanning during recording to free memory
  deinitBLE();

  // Generate timestamped filename
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    snprintf(recordingPath, sizeof(recordingPath),
      "/sdcard/fridge_%04d%02d%02d_%02d%02d%02d.avi",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    snprintf(recordingPath, sizeof(recordingPath),
      "/sdcard/fridge_%lu.avi", millis());
  }

  // Get frame dimensions from current camera setting
  sensor_t *s = esp_camera_sensor_get();
  uint16_t w = 320, h = 240;  // QVGA default
  if (s) {
    // Capture one frame to get actual dimensions
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      w = fb->width;
      h = fb->height;

      // Save as lastCapture for web UI (first frame of recording)
      xSemaphoreTake(captureMux, portMAX_DELAY);
      if (lastCapture) {
        esp_camera_fb_return(lastCapture);
      }
      lastCapture = fb;  // don't return — kept for web UI
      xSemaphoreGive(captureMux);

      // Also write this first frame to AVI
      if (avi_start(SD_MMC, recordingPath, w, h, RECORDING_FPS)) {
        avi_add_frame(fb->buf, fb->len);
        recording = true;
        recordingStart = millis();
        Serial.printf("[REC] Started: %s (%ux%u)\n", recordingPath, w, h);
      } else {
        Serial.println("[REC] Failed to start AVI file");
      }
      return;
    }
  }

  // Fallback if we couldn't get a frame
  if (avi_start(SD_MMC, recordingPath, w, h, RECORDING_FPS)) {
    recording = true;
    recordingStart = millis();
    Serial.printf("[REC] Started: %s (%ux%u)\n", recordingPath, w, h);
  }
}

void stopRecording() {
  if (!recording) return;

  uint32_t frames = avi_frame_count();
  avi_end();
  recording = false;

  uint32_t elapsed = millis() - recordingStart;
  Serial.printf("[REC] Stopped: %u frames, %u ms, file: %s\n",
    frames, elapsed, recordingPath);

  // Restart BLE scanning after recording
  initBLE();
}

void recordFrame() {
  static uint32_t lastFrameTime = 0;
  uint32_t now = millis();

  // Throttle to target FPS
  if (now - lastFrameTime < RECORDING_FRAME_DELAY) return;
  lastFrameTime = now;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[REC] Frame capture failed");
    return;
  }

  if (!avi_add_frame(fb->buf, fb->len)) {
    Serial.println("[REC] Frame write failed or index full — stopping");
    esp_camera_fb_return(fb);
    stopRecording();
    return;
  }

  esp_camera_fb_return(fb);
}

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("=== Fridge CAM ===");

  captureMux = xSemaphoreCreateMutex();

  // Camera init
  camera_init();
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 1);
  s->set_saturation(s, -1);

  // SD card init (1-bit SDMMC)
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    sdCardOK = true;
    Serial.printf("[SD] Mounted — Size: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  } else {
    Serial.println("[SD] Mount failed — recording disabled");
  }

  // WiFi
  WiFi.begin(ssid_Router, password_Router);
  WiFi.setSleep(false);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

  // Sync time for filenames
  configTime(0, 0, "pool.ntp.org");

  // HTTP server
  startCameraServer();
  Serial.printf("[HTTP] Server ready at http://%s\n", WiFi.localIP().toString().c_str());

  // BLE init — after HTTP server so sockets are available for httpd
  initBLE();
}

// ========================
// Loop
// ========================
void loop() {
  // Handle door-open → start recording
  if (requestStart && !recording) {
    requestStart = false;
    startRecording();
  }

  // Handle door-close → stop recording
  if (requestStop && recording) {
    requestStop = false;
    stopRecording();
  }

  // Safety cap: auto-stop after max duration
  if (recording && (millis() - recordingStart > RECORDING_MAX_MS)) {
    Serial.println("[REC] Max duration reached — auto-stopping");
    stopRecording();
  }

  // Record frames while door is open
  if (recording) {
    recordFrame();
  }

  // BLE: run finite scan cycles
  if (pBLEScan && !recording) {
    pBLEScan->start(BLE_SCAN_DURATION, false);
    pBLEScan->clearResults();
  }
}

// ========================
// BLE init / deinit helpers
// ========================
void initBLE() {
  NimBLEDevice::init("");
  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(new ShellyBLECallbacks(), true);
  pBLEScan->setActiveScan(false);     // passive scan — less memory pressure
  pBLEScan->setInterval(160);         // 160ms interval
  pBLEScan->setWindow(30);            // 30ms window (~19% duty)
  Serial.println("[BLE] Scanning for Shelly BLU Door/Window sensor...");
}

void deinitBLE() {
  if (pBLEScan) {
    pBLEScan->stop();
    pBLEScan->clearResults();
    pBLEScan = NULL;
  }
  NimBLEDevice::deinit(true);
  Serial.println("[BLE] Scan stopped (recording)");
}

// ========================
// Camera hardware config
// ========================
void camera_init() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.frame_size   = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count     = 2;
}
