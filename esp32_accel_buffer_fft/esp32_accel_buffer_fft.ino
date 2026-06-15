#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "arduinoFFT.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define MAX_SAMPLES_PER_PACKET 40 

// ── FFT config ────────────────────────────────────────────────────────────────
#define FFT_SAMPLES     128          // must be power of 2
#define SAMPLE_RATE_HZ  100          // must match nRF ACCEL_SAMPLE_RATE_HZ

static double vReal[FFT_SAMPLES];
static double vImag[FFT_SAMPLES];
static int fft_index = 0;
static float dominant_freq = 0.0f;
static bool fft_ready = false;

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE_HZ);

static NimBLEClient          *pClient        = nullptr;
static bool                   doConnect      = false;
static bool                   connected      = false;
static NimBLEAdvertisedDevice *pTargetDevice = nullptr;

// ── Binary packet struct ──────────────────────────────────────────────────────
struct AccelSample {
    int16_t x;
    int16_t y;
    int16_t z;
} __attribute__((packed));

// ── Sample playback buffer ────────────────────────────────────────────────────
static AccelSample  playback_buf[MAX_SAMPLES_PER_PACKET];
static uint8_t      playback_count    = 0;
static uint8_t      playback_index    = 0;
static uint16_t     playback_interval = 10;
static hw_timer_t  *sampleTimer       = NULL;
static portMUX_TYPE timerMux          = portMUX_INITIALIZER_UNLOCKED;
static volatile bool showNextSample   = false;

// -- FFT Calculation --
void PerformFFT() {

    // Apply AHmming Window to reduce spectral leakage
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    //Compute the FFT
    FFT.compute(FFTDirection::Forward); 

    //Convert complex output to magnitude
    FFT.complexToMagnitude();

    //Find the dominant frequency
    double peak = FFT.majorPeak();

    //Ignore DC component (0Hz) and very low frequencies
    if (peak < 1.0) peak = 0.0;

    dominant_freq = (float)peak;
    fft_ready = true;

    // Print full spectrum to Serial for debugging
    Serial.printf("FFT complete — dominant: %.2f Hz\n", dominant_freq);
    Serial.println("Spectrum (bin: freq Hz = magnitude):");
    for (int i = 1; i < FFT_SAMPLES / 2; i++) {
        float bin_freq = (float)i * SAMPLE_RATE_HZ / FFT_SAMPLES;
        if (vReal[i] > 10.0) {   // only print significant bins
            Serial.printf("  bin %3d: %5.1f Hz = %.1f\n", i, bin_freq, vReal[i]);
        }
    }

}

// ── Timer ISR — fires every interval_ms ──────────────────────────────────────
void IRAM_ATTR onSampleTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    if (playback_index < playback_count) {
        showNextSample = true;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ── Start/restart the playback timer ─────────────────────────────────────────
void startPlaybackTimer(uint16_t interval_ms)
{
    if (sampleTimer) {
        timerDetachInterrupt(sampleTimer);
        timerEnd(sampleTimer);
        sampleTimer = NULL;
    }

    // New API: timerBegin takes frequency in Hz
    // We want 1MHz (1 tick per microsecond)
    sampleTimer = timerBegin(1000000);                  // 1MHz
    timerAttachInterrupt(sampleTimer, &onSampleTimer);  // no 'true' arg
    timerAlarm(sampleTimer, interval_ms * 1000, true, 0); // replaces timerAlarmWrite + timerAlarmEnable
}

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

void showFFT(float freq, float ax, float ay ,float az) 
{
    display.clearDisplay();

    // Header bar
    display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(4, 2);
    display.print("nRF52832 Monitor");

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Dominant frequency — large text
    display.setCursor(0, 14);
    display.print("Vib:");
    display.setTextSize(2);
    display.setCursor(30, 12);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fHz", freq);
    display.println(buf);

    // Current acceleration values — small text below
    display.setTextSize(1);
    display.setCursor(0, 34);
    snprintf(buf, sizeof(buf), "X:%.2f", ax);
    display.print(buf);
    display.setCursor(44, 34);
    snprintf(buf, sizeof(buf), "Y:%.2f", ay);
    display.print(buf);
    display.setCursor(88, 34);
    snprintf(buf, sizeof(buf), "Z:%.2f", az);
    display.print(buf);

    // Progress bar showing FFT buffer fill level
    display.setCursor(0, 46);
    display.print("FFT buf:");
    int bar_width = (fft_index * 70) / FFT_SAMPLES;  // scale to 70 pixels
    display.fillRect(50, 47, bar_width, 8, SSD1306_WHITE);
    display.drawRect(50, 47, 70, 8, SSD1306_WHITE);   // outline

    display.display();

}

// ── Binary packet unpacker ────────────────────────────────────────────────────
bool parseBinaryPacket(const uint8_t *data, size_t length,
                       AccelSample *samples_out, uint8_t &count_out,
                       uint16_t &interval_out)
{
    if (length < 4) {
        Serial.println("Packet too short");
        return false;
    }
    if (data[0] != 0xAC) {
        Serial.printf("Bad magic: 0x%02X\n", data[0]);
        return false;
    }

    count_out    = data[1];
    interval_out = data[2] | ((uint16_t)data[3] << 8);

    size_t expected = 4 + count_out * sizeof(AccelSample);
    if (length < expected) {
        Serial.printf("Packet length mismatch: got %d, expected %d\n",
                      length, expected);
        return false;
    }

    memcpy(samples_out, data + 4, count_out * sizeof(AccelSample));
    return true;
}

// ── Notification callback — store samples and start timer ────────────────────
void notifyCallback(NimBLERemoteCharacteristic *pChar,
                    uint8_t *pData, size_t length, bool isNotify)
{
    AccelSample samples[MAX_SAMPLES_PER_PACKET];
    uint8_t  count       = 0;
    uint16_t interval_ms = 0;

    if (!parseBinaryPacket(pData, length, samples, count, interval_ms)) {
        return;
    }

    // ── Feed samples into FFT accumulation buffer ─────────────────────────
    for (int i = 0; i < count && fft_index < FFT_SAMPLES; i++) {

        // Convert int16 to m/s² float and feed into real part
        // Using X axis — change to .y or .z for different axis
        // or use magnitude: sqrt(x²+y²+z²) for total vibration
        float ax = samples[i].x / 100.0f;
        float ay = samples[i].y / 100.0f;
        float az = samples[i].z / 100.0f;

        // Total magnitude — captures vibration in any direction
        vReal[fft_index] = sqrt(ax*ax + ay*ay + az*az);
        vImag[fft_index] = 0.0;   // imaginary part always 0 for real signals
        fft_index++;
    }

    // When buffer full, run FFT then reset for next window
    if (fft_index >= FFT_SAMPLES) {
        PerformFFT();
        fft_index = 0;   // reset — start accumulating next window
    }
    // ─────────────────────────────────────────────────────────────────────

    // Load new samples into playback buffer under lock
    portENTER_CRITICAL(&timerMux);
    memcpy(playback_buf, samples, count * sizeof(AccelSample));
    playback_count    = count;
    playback_index    = 0;
    playback_interval = interval_ms;
    portEXIT_CRITICAL(&timerMux);

    startPlaybackTimer(interval_ms);

    Serial.printf("Packet received: %d samples @ %dms interval\n",
                  count, interval_ms);
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

    NimBLEDevice::setMTU(247);
    pClient = NimBLEDevice::createClient();

    if (!pClient->connect(pTargetDevice)) {
        showStatus("Connect failed", "Retrying...");
        return false;
    }

    // 100ms interval to match nRF side (80 × 1.25ms = 100ms)
    pClient->updateConnParams(80, 80, 0, 400);
    Serial.println("Connection params updated: 100ms interval");

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

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

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

// ── Loop ──────────────────────────────────────────────────────────────────────
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
        // Stop timer cleanly on disconnect
        if (sampleTimer) {
            timerDetachInterrupt(sampleTimer);
            timerEnd(sampleTimer);
            sampleTimer = NULL;
        }
        fft_index = 0;          // reset FFT buffer on disconnect
        fft_ready = false;
        showStatus("Disconnected", "Scanning...");
        NimBLEDevice::getScan()->start(0);
    }

    // Display next sample when timer fires
    if (showNextSample) {
        portENTER_CRITICAL(&timerMux);
        showNextSample = false;
        uint8_t idx = playback_index++;
        portEXIT_CRITICAL(&timerMux);

        if (idx < playback_count) {
            float ax = playback_buf[idx].x / 100.0f;
            float ay = playback_buf[idx].y / 100.0f;
            float az = playback_buf[idx].z / 100.0f;
            
            showFFT(dominant_freq, ax, ay, az);
            Serial.printf("[%2d/%2d] X=%.2f Y=%.2f Z=%.2f | Vib: %.1fHz\n",
                          idx + 1, playback_count, ax, ay, az, dominant_freq);
            //showAccelerometer(ax, ay, az);
            //Serial.printf("[%2d/%2d] X=%.2f Y=%.2f Z=%.2f\n",
                          //idx + 1, playback_count, ax, ay, az);
        }
    }

    delay(1);  // yield to BLE stack
}