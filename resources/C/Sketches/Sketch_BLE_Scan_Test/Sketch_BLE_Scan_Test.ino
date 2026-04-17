/**********************************************************************
  Filename    : BLE Scan Test v3
  Description : Minimal BLE scanner that dumps raw advertisement data
                for ALL devices, and scans raw bytes for BTHome UUID
                (0xFCD2) and Shelly manufacturer ID (0x0BA9).
  Board       : ESP32 Wrover (with PSRAM)
**********************************************************************/
#include <NimBLEDevice.h>

// BTHome v2 service UUID (16-bit: 0xFCD2)
static NimBLEUUID bthomeServiceUUID((uint16_t)0xFCD2);

static uint32_t totalDevicesSeen = 0;
static uint32_t bthomeDevicesSeen = 0;

// Door state tracking
static bool doorOpen = false;
static bool doorStateKnown = false;
static uint32_t lastDoorEvent = 0;

// Helper: search for a 2-byte pattern in a byte array
bool findBytes(const uint8_t *data, int len, uint8_t b0, uint8_t b1) {
  for (int i = 0; i < len - 1; i++) {
    if (data[i] == b0 && data[i + 1] == b1) return true;
  }
  return false;
}

// ========================
// Parse BTHome v2 payload (from raw AD bytes, starting after UUID)
// ========================
void parseBTHome(const uint8_t *data, int len) {
  if (len < 1) return;

  uint8_t deviceInfo = data[0];
  bool encrypted = deviceInfo & 0x01;
  uint8_t version = (deviceInfo >> 5) & 0x07;

  if (version != 2 || encrypted) return;

  bool prevDoorOpen = doorOpen;
  bool gotDoorEvent = false;

  int idx = 1;
  while (idx < len) {
    uint8_t objId = data[idx++];
    if (idx > len) break;
    switch (objId) {
      case 0x00: if (idx < len) { idx++; } break;
      case 0x01: if (idx < len) { Serial.printf("      Battery: %u%%\n", data[idx]); idx++; } break;
      case 0x05: if (idx+2 < len) { idx+=3; } else idx=len; break;
      case 0x2D: if (idx < len) { doorOpen = (data[idx] != 0); gotDoorEvent = true; idx++; } break;
      case 0x1A: if (idx < len) { doorOpen = (data[idx] != 0); gotDoorEvent = true; idx++; } break;
      case 0x11: if (idx < len) { doorOpen = (data[idx] != 0); gotDoorEvent = true; idx++; } break;
      case 0x3F: if (idx+1<len) { idx+=2; } else idx=len; break;
      default: return;
    }
  }

  if (gotDoorEvent) {
    if (!doorStateKnown || doorOpen != prevDoorOpen) {
      doorStateKnown = true;
      lastDoorEvent = millis();
      Serial.println();
      Serial.println("############################################");
      Serial.printf("###  DOOR %s  (at %u ms)\n", doorOpen ? ">>> OPENED <<<" : ">>> CLOSED <<<", lastDoorEvent);
      Serial.println("############################################");
      Serial.println();
    }
  }
}

// ========================
// Scan callback — raw-byte approach
// ========================
class RawScanCallback : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    totalDevicesSeen++;

    const std::vector<uint8_t>& payload = dev->getPayload();
    const uint8_t *raw = payload.data();
    int rawLen = payload.size();

    // --- Search raw bytes for known signatures ---
    bool hasBTHomeRaw = findBytes(raw, rawLen, 0xD2, 0xFC);  // 0xFCD2 little-endian
    bool hasShellyMfr = findBytes(raw, rawLen, 0xA9, 0x0B);  // 0x0BA9 little-endian

    // Also check NimBLE's parsed service data
    bool hasBTHomeParsed = false;
    if (dev->haveServiceData()) {
      std::string sd = dev->getServiceData(bthomeServiceUUID);
      if (!sd.empty()) hasBTHomeParsed = true;
    }

    bool isInteresting = hasBTHomeRaw || hasShellyMfr || hasBTHomeParsed;

    if (isInteresting) {
      bthomeDevicesSeen++;
      std::string name = dev->getName();
      Serial.println();
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.printf("!!! SHELLY / BTHome DEVICE FOUND !!!\n");
      Serial.printf("  MAC: %s  RSSI: %d  Name: \"%s\"\n",
        dev->getAddress().toString().c_str(),
        dev->getRSSI(),
        name.empty() ? "(none)" : name.c_str());
      Serial.printf("  BTHome in raw bytes: %s\n", hasBTHomeRaw ? "YES" : "no");
      Serial.printf("  Shelly mfr in raw bytes: %s\n", hasShellyMfr ? "YES" : "no");
      Serial.printf("  BTHome via NimBLE API: %s\n", hasBTHomeParsed ? "YES" : "no");

      // Dump full raw payload
      Serial.printf("  RAW (%d bytes):", rawLen);
      for (int i = 0; i < rawLen; i++) {
        if (i % 20 == 0 && i > 0) Serial.printf("\n                ");
        Serial.printf(" %02X", raw[i]);
      }
      Serial.println();

      // Parse AD structures manually
      Serial.println("  AD structures:");
      int pos = 0;
      while (pos < rawLen - 1) {
        uint8_t adLen = raw[pos];
        if (adLen == 0 || pos + adLen >= rawLen) break;
        uint8_t adType = raw[pos + 1];
        Serial.printf("    type=0x%02X len=%u data:", adType, adLen - 1);
        for (int i = 0; i < adLen - 1 && (pos + 2 + i) < rawLen; i++) {
          Serial.printf(" %02X", raw[pos + 2 + i]);
        }
        Serial.println();

        // If this is Service Data 16-bit UUID (0x16) and contains FCD2
        if (adType == 0x16 && adLen >= 3) {
          uint16_t uuid16 = raw[pos+2] | (raw[pos+3] << 8);
          if (uuid16 == 0xFCD2) {
            Serial.println("    ^^^ BTHome v2 service data! Parsing:");
            parseBTHome(raw + pos + 4, adLen - 3);
          }
        }

        // If this is Manufacturer Specific (0xFF)
        if (adType == 0xFF && adLen >= 3) {
          uint16_t mfrId = raw[pos+2] | (raw[pos+3] << 8);
          Serial.printf("    ^^^ Manufacturer ID: 0x%04X%s\n",
            mfrId, mfrId == 0x0BA9 ? " (SHELLY!)" : "");
        }

        pos += adLen + 1;
      }
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println();
    }

    // For non-interesting devices, just print a compact line every 50th device
    if (!isInteresting && (totalDevicesSeen % 50 == 0)) {
      Serial.printf("[BLE] %u devices scanned so far, %u BTHome found...\n",
        totalDevicesSeen, bthomeDevicesSeen);
    }
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    Serial.printf("[SCAN] Cycle done — %u total, %u BTHome. Restarting...\n",
      totalDevicesSeen, bthomeDevicesSeen);
  }
};

// ========================
// Setup
// ========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== BLE Raw Scan Test v3 ===");
  Serial.println("Scanning raw bytes for BTHome (0xFCD2) and Shelly (0x0BA9)");
  Serial.println("Trigger the sensor (open/close door) while scanning!");
  Serial.println();

  NimBLEDevice::init("ESP32_BLE_Test");

  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new RawScanCallback(), false);
  pScan->setActiveScan(true);
  pScan->setInterval(100);   // 62.5ms
  pScan->setWindow(99);      // ~99% duty
  pScan->setDuplicateFilter(false);

  Serial.println("[BLE] Starting continuous scan...");
  pScan->start(0, false);
}

// ========================
// Loop
// ========================
void loop() {
  static uint32_t lastMsg = 0;
  uint32_t now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    Serial.printf("\n--- %us | %u scanned | %u BTHome/Shelly | Door: %s ---\n",
      now / 1000, totalDevicesSeen, bthomeDevicesSeen,
      doorStateKnown ? (doorOpen ? "OPEN" : "CLOSED") : "unknown");
    if (bthomeDevicesSeen == 0 && now > 30000) {
      Serial.println("[HINT] No BTHome devices yet. Open/close the door to trigger the sensor.");
    }
  }
  delay(100);
}
