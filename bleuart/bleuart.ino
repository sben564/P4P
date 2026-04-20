#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

#define SDA_PIN 20
#define SCL_PIN 21

Adafruit_AHTX0 aht;

// BLE Service
BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);

#if CFG_DEBUG
  while (!Serial) yield();
#endif

  Serial.println("BLE UART + AHT20");

  // ---------------- BLE SETUP ----------------
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bledfu.begin();

  bledis.setManufacturer("Custom");
  bledis.setModel("nRF52840 AHT20");
  bledis.begin();

  bleuart.begin();
  blebas.begin();
  blebas.write(100);

  startAdv();

  // ---------------- I2C ----------------
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  // ---------------- SENSOR ----------------
  if (!aht.begin()) {
    Serial.println("AHT20 not found!");
    while (1) delay(10);
  }

  Serial.println("AHT20 ready");
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

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float tempC = temp.temperature;
  float hum = humidity.relative_humidity;

  Serial.print("Temp: ");
  Serial.print(tempC);
  Serial.print(" C  Humidity: ");
  Serial.print(hum);
  Serial.println(" %");

  bleuart.print("Temp: ");
  bleuart.print(tempC);
  bleuart.print(" C  Hum: ");
  bleuart.print(hum);
  bleuart.println(" %");

  delay(1000);
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