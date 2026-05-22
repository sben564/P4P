#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED Setup ────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── NUS UUIDs ─────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ── State ─────────────────────────────────────────────────────────────────────
static NimBLEClient *pClient             = nullptr;
static bool doConnect                    = false;
static bool connected                    = false;
static NimBLEAdvertisedDevice *pTargetDevice = nullptr;
static float lastTemperature             = 0.0f;
static bool hasReading                   = false;

// ── Display helpers ───────────────────────────────────────────────────────────
void showStatus(const char *line1, const char *line2 = "")
{
    display.clearDisplay();

    // Header bar
    display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 2);
    display.println("nRF52832 Monitor");

    // Status lines
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.println(line1);
    display.setCursor(0, 30);
    display.println(line2);

    display.display();
}

void showTemperature(float temp)
{
    display.clearDisplay();

    // Header bar
    display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 2);
    display.println("nRF52832 Monitor");

    // Label
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.println("Die Temperature:");

    // Big temperature number
    display.setTextSize(3);
    display.setCursor(8, 28);

    // Format with one decimal place
    int whole   = (int)temp;
    int decimal = abs((int)((temp - (float)whole) * 10));
    char buf[16];
    snprintf(buf, sizeof(buf), "%d.%01d C", whole, decimal);
    display.println(buf);

    display.display();
}

// ── Temperature parser ────────────────────────────────────────────────────────
float parseTemperature(const std::string &data)
{
    const std::string prefix = "TEMP:";
    size_t pos = data.find(prefix);
    if (pos == std::string::npos) return -999.0f;
    return atof(data.substr(pos + prefix.length()).c_str());
}

// ── Notification callback — fires every time nRF sends temperature ────────────
void notifyCallback(NimBLERemoteCharacteristic *pChar,
                    uint8_t *pData, size_t length, bool isNotify)
{
    std::string received((char *)pData, length);
    Serial.print("Raw: ");
    Serial.println(received.c_str());

    float temp = parseTemperature(received);
    if (temp > -999.0f) {
        lastTemperature = temp;
        hasReading = true;
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println(" C");
        showTemperature(temp);  // update OLED immediately
    }
}

// ── Scan callback ─────────────────────────────────────────────────────────────
class ScanCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
    {
        Serial.print("Found: '");
        Serial.print(advertisedDevice->getName().c_str());
        Serial.print("' Addr: ");
        Serial.println(advertisedDevice->getAddress().toString().c_str());

        if (advertisedDevice->isAdvertisingService(
                NimBLEUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"))) {
            Serial.println(">>> Found NUS! Connecting...");
            NimBLEDevice::getScan()->stop();
            pTargetDevice = const_cast<NimBLEAdvertisedDevice *>(advertisedDevice);
            doConnect = true;
        }
    }
};

// ── Connect and subscribe ─────────────────────────────────────────────────────
bool connectToServer()
{
    Serial.print("Connecting to: ");
    Serial.println(pTargetDevice->getAddress().toString().c_str());
    showStatus("Connecting...", pTargetDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    if (!pClient->connect(pTargetDevice)) {
        Serial.println("Connection failed");
        showStatus("Connect failed", "Retrying...");
        return false;
    }
    Serial.println("Connected!");

    NimBLERemoteService *pService = pClient->getService(NUS_SERVICE_UUID);
    if (!pService) {
        Serial.println("NUS service not found");
        pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic *pTxChar = pService->getCharacteristic(NUS_TX_CHAR_UUID);
    if (!pTxChar) {
        Serial.println("TX characteristic not found");
        pClient->disconnect();
        return false;
    }

    if (pTxChar->canNotify()) {
        pTxChar->subscribe(true, notifyCallback);
        Serial.println("Subscribed — waiting for data...");
        showStatus("Connected!", "Waiting for temp...");
    }

    connected = true;
    return true;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Init OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED not found — check wiring");
        while (true); // halt
    }
    showStatus("Starting...", "Init BLE...");

    // Init BLE
    NimBLEDevice::init("ESP32_Central");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new ScanCallbacks());
    pScan->setActiveScan(true);
    pScan->setInterval(45);
    pScan->setWindow(15);

    showStatus("Scanning...", "Looking for nRF");
    Serial.println("Scanning...");
    pScan->start(0);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop()
{
    if (doConnect) {
        doConnect = false;
        if (connectToServer()) {
            Serial.println("Handshake complete!");
        } else {
            Serial.println("Retrying scan...");
            showStatus("Scanning...", "Looking for nRF");
            NimBLEDevice::getScan()->start(0);
        }
    }

    if (connected && pClient && !pClient->isConnected()) {
        Serial.println("Disconnected — restarting scan...");
        connected = false;
        hasReading = false;
        showStatus("Disconnected", "Scanning...");
        NimBLEDevice::getScan()->start(0);
    }

    delay(1000);
}