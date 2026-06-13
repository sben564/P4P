#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static NimBLEClient *pClient             = nullptr;
static bool doConnect                    = false;
static bool connected                    = false;
static NimBLEAdvertisedDevice *pTargetDevice = nullptr;

// ── Display ───────────────────────────────────────────────────────────────────
void showStatus(const char *line1, const char *line2 = "")
{
    display.clearDisplay();
    display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 2);
    display.print("nRF52832 Monitor");

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.println(line1);
    display.setCursor(0, 30);
    display.println(line2);
    display.display();
}

void showAccelerometer(float ax, float ay, float az)
{
    display.clearDisplay();
    display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 2);
    display.print("nRF52832 Monitor");

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    char buf[16];

    display.setCursor(0, 16);
    display.print("X:");
    snprintf(buf, sizeof(buf), "%.2f", ax);
    display.setTextSize(2);
    display.setCursor(20, 13);
    display.println(buf);

    display.setTextSize(1);
    display.setCursor(0, 33);
    display.print("Y:");
    snprintf(buf, sizeof(buf), "%.2f", ay);
    display.setTextSize(2);
    display.setCursor(20, 30);
    display.println(buf);

    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("Z:");
    snprintf(buf, sizeof(buf), "%.2f", az);
    display.setTextSize(2);
    display.setCursor(20, 47);
    display.println(buf);

    display.display();
}

// ── Parser ────────────────────────────────────────────────────────────────────
// Expects: "ACCEL:X=0.12,Y=-9.81,Z=0.03"
bool parseAccel(const std::string &data, float &ax, float &ay, float &az)
{
    if (data.find("ACCEL:") == std::string::npos) return false;

    auto field = [&](const char *key) -> float {
        size_t p = data.find(key);
        if (p == std::string::npos) return 0.0f;
        return atof(data.substr(p + strlen(key)).c_str());
    };

    ax = field("X:");
    ay = field("Y:");
    az = field("Z:");
    return true;
}

// ── Notification callback ─────────────────────────────────────────────────────
void notifyCallback(NimBLERemoteCharacteristic *pChar,
                    uint8_t *pData, size_t length, bool isNotify)
{
    std::string received((char *)pData, length);
    Serial.println(received.c_str());

    float ax, ay, az;
    if (parseAccel(received, ax, ay, az)) {
        Serial.printf("Accel: X=%.2f Y=%.2f Z=%.2f\n", ax, ay, az);
        showAccelerometer(ax, ay, az);
    }
}

// ── Scan callback ─────────────────────────────────────────────────────────────
class ScanCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
    {
        if (advertisedDevice->isAdvertisingService(
                NimBLEUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"))) {
            Serial.println("Found NUS device, connecting...");
            NimBLEDevice::getScan()->stop();
            pTargetDevice = const_cast<NimBLEAdvertisedDevice *>(advertisedDevice);
            doConnect = true;
        }
    }
};

// ── Connect ───────────────────────────────────────────────────────────────────
bool connectToServer()
{
    showStatus("Connecting...", pTargetDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    if (!pClient->connect(pTargetDevice)) {
        showStatus("Connect failed", "Retrying...");
        return false;
    }

    pClient->updateConnParams(6, 6, 0, 400);
    Serial.println("Connection params updated: 7.5ms interval");

    NimBLERemoteService *pService = pClient->getService(NUS_SERVICE_UUID);
    if (!pService) { pClient->disconnect(); return false; }

    NimBLERemoteCharacteristic *pTxChar = pService->getCharacteristic(NUS_TX_CHAR_UUID);
    if (!pTxChar) { pClient->disconnect(); return false; }

    if (pTxChar->canNotify()) {
        pTxChar->subscribe(true, notifyCallback);
        showStatus("Connected!", "Waiting for data...");
    }

    connected = true;
    return true;
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // ← add this

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED init failed");
        while (true);
    }

    showStatus("Scanning...", "Looking for nRF");

    NimBLEDevice::init("ESP32_Central");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new ScanCallbacks());
    pScan->setActiveScan(true);
    pScan->setInterval(45);
    pScan->setWindow(15);
    pScan->start(0);
}

void loop()
{
    if (doConnect) {
        doConnect = false;
        if (!connectToServer()) {
            showStatus("Scanning...", "Looking for nRF");
            NimBLEDevice::getScan()->start(0);
        }
    }

    if (connected && pClient && !pClient->isConnected()) {
        connected = false;
        showStatus("Disconnected", "Scanning...");
        NimBLEDevice::getScan()->start(0);
    }

    delay(100);
}
