#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <SparkFunLIS3DH.h>

#define SDA_PIN 20
#define SCL_PIN 21

LIS3DH myIMU;
bool sensorOK = false;

// BLE Service
BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);
  while (!Serial) yield();
  delay(2000); // give Mac time to fully open the port
  Serial.println("Starting...");

#if CFG_DEBUG
  while (!Serial) yield();
#endif

  Serial.println("BLE UART + AHT20");

  // ---------------- BLE SETUP ----------------
  Serial.println("BLE: configuring...");
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Serial.println("BLE: begin...");
  Bluefruit.begin();
  Bluefruit.setTxPower(4);

  Serial.println("BLE: setTxPower...");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  Serial.println("BLE: bledfu...");
  bledfu.begin();

  Serial.println("BLE: bledis...");
  bledis.setManufacturer("Custom");
  bledis.setModel("nRF52840 AHT20");
  bledis.begin();

  Serial.println("BLE: bleuart...");
  bleuart.begin();

  Serial.println("BLE: blebas...");
  blebas.begin();
  blebas.write(100);

  Serial.println("BLE: startAdv...");
  startAdv();

  // ---------------- I2C ----------------
  Serial.println("I2C: begin...");
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  Serial.println("I2C: scanning...");
for (byte addr = 1; addr < 127; addr++) {
  Wire.beginTransmission(addr);
  byte err = Wire.endTransmission();
  if (err == 0) {
    Serial.print("Found device at 0x");
    Serial.println(addr, HEX);
  }
}
Serial.println("I2C: scan done");

  // ---------------- SENSOR ----------------
  Serial.println("Sensor: begin...");
  delay(500); 
  if (myIMU.begin() != 0) {
    Serial.println("LIS3DH not found! Check wiring.");
  } else {
    sensorOK = true;
    Serial.println("LIS3DH ready");
  }
  Serial.println("Setup complete.");
}

// ===================== LOOP =====================
void loop()
{
  while (Serial.available())
  {
    delay(2);

    uint8_t buf[64];
    int count = Serial.readBytes(buf, sizeof(buf));
    bleuart.write(buf, count);
  }

  while (bleuart.available())
  {
    uint8_t ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }

  static uint32_t lastTime = 0;

  if (millis() - lastTime > 1000)
  {
  lastTime = millis();

  Serial.print("notifyEnabled: ");
  Serial.println(bleuart.notifyEnabled() ? "YES" : "NO");

  if (!sensorOK) {
    Serial.println("Skipping - no sensor");
    return;
  }

  float x = myIMU.readFloatAccelX();
float y = myIMU.readFloatAccelY();
float z = myIMU.readFloatAccelZ();

Serial.print("X: "); Serial.print(x);
Serial.print(" Y: "); Serial.print(y);
Serial.print(" Z: "); Serial.println(z);

if (Bluefruit.connected()) {
    bleuart.print("X: "); bleuart.print(x);
    bleuart.print(" Y: "); bleuart.print(y);
    bleuart.print(" Z: "); bleuart.println(z);
}

  }
}

// ===================== BLE CALLBACKS =====================
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
}

// ===================== ADVERTISING =====================
void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}