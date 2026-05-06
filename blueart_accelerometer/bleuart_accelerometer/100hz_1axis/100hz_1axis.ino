#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include <SparkFunLIS3DH.h>
#include <nrf.h>

#define SDA_PIN 20
#define SCL_PIN 21

LIS3DH myIMU;
bool sensorOK = false;

// ===================== TIMER INTERRUPT (100ms) =====================
volatile bool timerInterrupt = false;

void setupTimer100ms()
{
  // Configure TIMER2 for 100ms interrupt
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
  
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER2->PRESCALER = 4;  // 1 MHz clock (16 MHz / 2^4)
  
  // 100ms @ 1MHz = 100,000 ticks
  NRF_TIMER2->CC[0] = 100000;
  
  // Enable interrupt and auto-reload
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
  NRF_TIMER2->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
  
  NVIC_SetPriority(TIMER2_IRQn, 7);
  NVIC_EnableIRQ(TIMER2_IRQn);
  
  NRF_TIMER2->TASKS_START = 1;
}

extern "C" void TIMER2_IRQHandler(void)
{
  if (NRF_TIMER2->EVENTS_COMPARE[0] != 0)
  {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    timerInterrupt = true;
  }
}

// BLE Service
BLEDfu  bledfu;
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);
  unsigned long t0 = millis();
  //while (!Serial && millis() - t0 < 3000) {
    //yield(); // wait max 3 seconds
  //}
  delay(2000); // give Mac time to fully open the port
  Serial.println("Starting...");

// #if CFG_DEBUG
//   while (!Serial) yield();
// #endif

  Serial.println("BLE UART + AHT20");

  // ---------------- BLE SETUP ----------------
  Serial.println("BLE: configuring...");
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);//Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Serial.println("BLE: begin...");
  Bluefruit.begin();
  Bluefruit.Periph.setConnInterval(40, 80); // ~50–100 ms
  // Stops LED blinking on connection
  // Bluefruit.autoConnLed(false);
  // pinMode(LED_RED, OUTPUT);
  // digitalWrite(LED_RED, HIGH); // or HIGH depending on active-low config

  // pinMode(LED_BLUE, OUTPUT);  
  // digitalWrite(LED_BLUE, LOW);
  Bluefruit.setTxPower(0);

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
    myIMU.writeRegister(LIS3DH_CTRL_REG1, 0x77); // 10Hz, z-axis only 
    Serial.println("LIS3DH ready");
  }

  // ---------------- TIMER ----------------
  Serial.println("Timer: begin...");
  setupTimer100ms();

  Serial.println("Setup complete.");
}

// ===================== LOOP =====================
void loop()
{
  // Sleep until timer interrupt fires
  if (!timerInterrupt)
  {
    // Enter low-power sleep mode, wake on timer interrupt
    __WFE();
    return;
  }

  // Timer fired: clear flag and do work
  noInterrupts();
  timerInterrupt = false;
  interrupts();

  // Service happens every 100ms
  if (Bluefruit.connected()) 
  {
    if (sensorOK) {
      // Read sensor
      float z = myIMU.readFloatAccelZ();

      // Format and send data
      char buffer[32];
      int len = snprintf(buffer, sizeof(buffer), "Z:%.3f\n", z);
      bleuart.write((uint8_t*)buffer, len);
    }
  }
  else
  {
    // DISCONNECTED STATE: Still checking every 100ms (lower power than delay(100))
    // because __WFE() puts the MCU to sleep between timer interrupts
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
  Bluefruit.Advertising.setInterval(160, 160);
  //Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// 18-25mv when not connected, 35mv when connected (more stable)
