#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <SparkFunLIS3DH.h>

// ── Pin config ───────────────────────────────────────────
#define SDA_PIN        20
#define SCL_PIN        21

// ── Hanging GPIO — pull these down to stop float noise ───
// Adjust list to match pins you're not using
#define HANGING_PINS { 3, 4, 5, 6, 8, 9, 10, 12, 13 }
#define LOW_POWER_MODE  false 

// ═══════════════════════════════════════════════════════
//   TEST MODE CONFIG — edit these before each flash
// ═══════════════════════════════════════════════════════
#define SEND_X          true
#define SEND_Y          true
#define SEND_Z          true
#define LED_ENABLED     false    // false = LED off during TX
#define SAMPLE_HZ       1      // 1|10|25|50|100|200|400 Hz
#define TX_INTERVAL_MS  1000        // ms between BLE UART sends
#define ACCEL_RANGE     2        // ±G: 2, 4, 8, or 16
#define DEVICE_NAME     "IMU-Test"
// ═══════════════════════════════════════════════════════

LIS3DH  myIMU;
bool    sensorOK = false;

BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;   // Nordic UART — immediately visible in Bluefruit/nRF Connect
BLEBas  blebas;
SoftwareTimer txTimer;

// ── Helpers ──────────────────────────────────────────────

void pullDownFloatingPins() {
  int pins[] = HANGING_PINS;
  for (int p : pins) {
    pinMode(p, INPUT_PULLDOWN);
  }
}

// Returns CTRL_REG1 value for desired ODR, all axes on, normal power
uint8_t odrByte(int hz, bool lowPower = false) {
  uint8_t lpen = lowPower ? 0x08 : 0x00;
  if (hz <=   1) return 0x17 | lpen;
  if (hz <=  10) return 0x27 | lpen;
  if (hz <=  25) return 0x37 | lpen;
  if (hz <=  50) return 0x47 | lpen;
  if (hz <= 100) return 0x57 | lpen;
  if (hz <= 200) return 0x67 | lpen;
  return 0x77 | lpen;
}

// Maps ACCEL_RANGE to CTRL_REG4 value
uint8_t rangeByte(int g) {
  if (g <=  2) return 0x00;
  if (g <=  4) return 0x10;
  if (g <=  8) return 0x20;
  return 0x30; // 16G
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);  // NUS UUID — apps recognise immediately
  Bluefruit.Advertising.addName();            // name in PRIMARY packet, not scan response

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160); // fast advertising, always discoverable
  Bluefruit.Advertising.setFastTimeout(0);
  Bluefruit.Advertising.start(0);
}

// ── Setup ────────────────────────────────────────────────
void setup() {
  // Floating pins first — before anything else touches the bus
  pullDownFloatingPins();
  txTimer.begin(TX_INTERVAL_MS, txTimerCallback, NULL, false); // false = one-shot

  // LED
            // we control the LED
  // pinMode(LED_BLUE, OUTPUT);
  // pinMode(LED_RED, OUTPUT);
  // digitalWrite(LED_BLUE, LOW);  // HIGH = off on SparkFun
  // digitalWrite(LED_RED,  HIGH);  // HIGH = off on SparkFun
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== IMU BLE UART Test ===");
  Serial.printf("Axes: X=%d Y=%d Z=%d\n", SEND_X, SEND_Y, SEND_Z);
  Serial.printf("Rate: %d Hz  Interval: %d ms  Range: +/-%dG  LED: %d\n",
                SAMPLE_HZ, TX_INTERVAL_MS, ACCEL_RANGE, LED_ENABLED);

  // BLE
  Bluefruit.autoConnLed(false);             // we control the LED
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(0);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);



  bledfu.begin();

  bledis.setManufacturer("Custom");
  bledis.setModel("nRF52840 LIS3DH");
  bledis.begin();

  bleuart.begin();

  blebas.begin();
  blebas.write(100);

  startAdv();
  Serial.println("BLE advertising...");

  // I2C
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  delay(200);

  // I2C scan — useful for debugging wiring
  Serial.println("I2C scan:");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found: 0x%02X\n", addr);
    }
  }

  // IMU init
  delay(200);
  if (myIMU.begin() != 0) {
    Serial.println("LIS3DH NOT FOUND — check wiring!");
    sensorOK = false;
  } else {
    myIMU.writeRegister(LIS3DH_CTRL_REG1, odrByte(SAMPLE_HZ, LOW_POWER_MODE));
    myIMU.writeRegister(LIS3DH_CTRL_REG4, rangeByte(ACCEL_RANGE));
    sensorOK = true;
    Serial.printf("LIS3DH ready: %dHz ±%dG\n", SAMPLE_HZ, ACCEL_RANGE);
  }

  Serial.println("Setup complete.\n");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  while (bleuart.available()) Serial.write((uint8_t)bleuart.read());
  while (Serial.available()) {
    uint8_t buf[64];
    int n = Serial.readBytes(buf, sizeof(buf));
    bleuart.write(buf, n);
  }

  if (!sensorOK || !Bluefruit.connected()) {
    delay(10);
    return;
  }

  char buf[80];
  int  n = 0;

#if LOW_POWER_MODE
  if (SEND_X) n += snprintf(buf+n, sizeof(buf)-n, "X:% .4f ", readLowPowerX());
  if (SEND_Y) n += snprintf(buf+n, sizeof(buf)-n, "Y:% .4f ", readLowPowerY());
  if (SEND_Z) n += snprintf(buf+n, sizeof(buf)-n, "Z:% .4f",  readLowPowerZ());
#else
  if (SEND_X) n += snprintf(buf+n, sizeof(buf)-n, "X:% .4f ", myIMU.readFloatAccelX());
  if (SEND_Y) n += snprintf(buf+n, sizeof(buf)-n, "Y:% .4f ", myIMU.readFloatAccelY());
  if (SEND_Z) n += snprintf(buf+n, sizeof(buf)-n, "Z:% .4f",  myIMU.readFloatAccelZ());
#endif

  bleuart.println(buf);

  // Start one-shot timer then suspend until it fires
  txTimer.start();
  suspendLoop();  // ← replaces delay(TX_INTERVAL_MS)
}

// ── BLE Callbacks ────────────────────────────────────────
void connect_callback(uint16_t conn_handle) {
  char name[32] = {0};
  Bluefruit.Connection(conn_handle)->getPeerName(name, sizeof(name));
  Serial.printf("Connected: %s\n", name);
  if (LED_ENABLED) digitalWrite(LED_BLUE, HIGH);  // LOW = on
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.printf("Disconnected (0x%02X)\n", reason);
  digitalWrite(LED_BLUE, HIGH);  // off
}

float readLowPowerX() {
  int8_t raw;
  myIMU.readRegister((uint8_t*)&raw, LIS3DH_OUT_X_H);
  return (float)raw / 64.0f;
}

float readLowPowerY() {
  int8_t raw;
  myIMU.readRegister((uint8_t*)&raw, LIS3DH_OUT_Y_H);
  return (float)raw / 64.0f;
}

float readLowPowerZ() {
  int8_t raw;
  myIMU.readRegister((uint8_t*)&raw, LIS3DH_OUT_Z_H);
  return (float)raw / 64.0f;
}

void txTimerCallback(TimerHandle_t xTimerID) {
  (void)xTimerID;
  resumeLoop();  // wake the loop task after TX_INTERVAL_MS
}