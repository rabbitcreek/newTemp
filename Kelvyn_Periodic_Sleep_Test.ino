// Kelvyn Periodic BLE / System ON Sleep Test v2
// BMD-340 nRF52840, Adafruit nRF52 core 1.7.0, S140
// SHT40 SDA P0.26 / SCL P0.27; I2C address 0x44
// Keep the proven Kelvyn_LowPower_Test sketch as a backup.
// Test: new reading every 30 seconds; BLE window up to 15 seconds.
// Connected: notify every 2 seconds until window ends, then disconnect.
// Disconnected: stop advertising at window end and idle until next cycle.
// IMPORTANT: System ON idle; no claim of minimum possible power consumption.

#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>

#define SDA_PIN 26
#define SCL_PIN 27
#define SHT40_ADDR 0x44

const uint32_t SAMPLE_INTERVAL_MS = 30000UL; // Change to 300000UL after testing
const uint32_t BLE_WINDOW_MS       = 15000UL; // Up to 15 seconds, including connection
const uint32_t NOTIFY_INTERVAL_MS  = 2000UL;

TwoWire sensorWire(NRF_TWIM0, NRF_TWIS0,
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
    SDA_PIN, SCL_PIN);

BLEService kelvynService("A6E10001-7A4B-4E36-9F3C-123456789ABC");
BLECharacteristic kelvynTemperature("A6E10002-7A4B-4E36-9F3C-123456789ABC");

// Debug counters readable by J-Link, if desired.
// [0] signature, [1] successful samples, [2] sensor errors,
// [3] last temp C*100, [4] last VDD mV, [5] cycle count.
volatile uint32_t kelvynDebug[6] __attribute__((used)) = {0};

uint8_t packet[6] = {0};
uint32_t cycleStart = 0;
uint32_t lastNotify = 0;
bool windowOpen = false;
bool notifyImmediately = false;

bool readSHT40(float &temperatureC, float &humidity) {
  sensorWire.beginTransmission(SHT40_ADDR);
  sensorWire.write(0xFD);
  if (sensorWire.endTransmission() != 0) return false;
  delay(10);
  if (sensorWire.requestFrom(SHT40_ADDR, (uint8_t)6) != 6) return false;
  uint8_t data[6];
  for (int i = 0; i < 6; i++) data[i] = sensorWire.read();
  uint16_t rawTemp = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawHumidity = ((uint16_t)data[3] << 8) | data[4];
  temperatureC = -45.0f + 175.0f * rawTemp / 65535.0f;
  humidity = constrain(-6.0f + 125.0f * rawHumidity / 65535.0f, 0.0f, 100.0f);
  return true;
}

uint16_t readSupplyMillivolts() {
  uint32_t raw = analogReadVDD();
  return (uint16_t)((raw * 3600UL + 2048UL) / 4096UL);
}

void makeMeasurement() {
  float tempC, humidity;
  if (!readSHT40(tempC, humidity)) {
    kelvynDebug[2]++;
    return; // Preserve previous valid reading if sensor fails
  }
  int16_t temp100 = (int16_t)(tempC * 100.0f);
  uint16_t humidity100 = (uint16_t)(humidity * 100.0f);
  uint16_t mv = readSupplyMillivolts();
  packet[0] = (uint8_t)temp100;
  packet[1] = (uint8_t)(temp100 >> 8);
  packet[2] = (uint8_t)humidity100;
  packet[3] = (uint8_t)(humidity100 >> 8);
  packet[4] = (uint8_t)mv;
  packet[5] = (uint8_t)(mv >> 8);
  kelvynTemperature.write(packet, sizeof(packet));
  kelvynDebug[1]++;
  kelvynDebug[3] = (uint32_t)(int32_t)temp100;
  kelvynDebug[4] = mv;
}

void startAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(kelvynService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.start(0);
}

void beginCycle(uint32_t now) {
  cycleStart = now;
  kelvynDebug[5]++;
  makeMeasurement();
  windowOpen = true;
  notifyImmediately = true;
  lastNotify = now;
  startAdvertising();
}

void endWindow() {
  windowOpen = false;
  Bluefruit.Advertising.stop();
  // Disconnect any central still attached. Do NOT auto-advertise on disconnect.
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(Bluefruit.connHandle());
  }
}

void setup() {
  kelvynDebug[0] = 0x4B454C56UL;
  sensorWire.begin();
  sensorWire.setClock(100000);
  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  Bluefruit.begin();
  Bluefruit.setName("Kelvyn");
  kelvynService.begin();
  kelvynTemperature.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  kelvynTemperature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  kelvynTemperature.setFixedLen(6);
  kelvynTemperature.begin();
  kelvynTemperature.write(packet, sizeof(packet));

  Bluefruit.Advertising.restartOnDisconnect(false);
  beginCycle(millis());
}

void loop() {
  uint32_t now = millis();
  uint32_t elapsed = (uint32_t)(now - cycleStart);

  // End BLE availability even if the iPhone stays connected.
  if (windowOpen && elapsed >= BLE_WINDOW_MS) {
    endWindow();
  }

  // Next sample is measured relative to the beginning of the previous cycle.
  if (elapsed >= SAMPLE_INTERVAL_MS) {
    if (windowOpen) endWindow();
    beginCycle(now);
  } else if (windowOpen && Bluefruit.connected()) {
    // Keep notifying throughout the window. Notifications are accepted after
    // the iPhone enables CCCD; early calls may simply return false.
    if (notifyImmediately || (uint32_t)(now - lastNotify) >= NOTIFY_INTERVAL_MS) {
      kelvynTemperature.notify(packet, sizeof(packet));
      notifyImmediately = false;
      lastNotify = now;
    }
  }

  // Let the SoftDevice put the CPU in System ON idle until an interrupt.
  // System tick/BLE interrupts still occur; this is NOT System OFF.
  (void)sd_app_evt_wait();
}
