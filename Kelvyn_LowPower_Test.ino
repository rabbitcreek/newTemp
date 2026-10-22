// Kelvyn Low-Power Baseline v1
// BMD-340 (nRF52840), Adafruit nRF52 core 1.7.0, S140
// SHT40: P0.26 SDA, P0.27 SCL; 0x44
// Preserve your known-good sketch separately.
// This is SYSTEM ON idle; BLE advertising/connection remains active.

#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>

#define SDA_PIN 26
#define SCL_PIN 27
#define SHT40_ADDR 0x44

const uint32_t SAMPLE_INTERVAL_MS = 2000;

TwoWire sensorWire(NRF_TWIM0, NRF_TWIS0,
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
    SDA_PIN, SCL_PIN);

BLEService kelvynService("A6E10001-7A4B-4E36-9F3C-123456789ABC");
BLECharacteristic kelvynTemperature("A6E10002-7A4B-4E36-9F3C-123456789ABC");

// Inspect these through J-Link if desired; no serial port required.
volatile uint32_t kelvynDebug[6] __attribute__((used)) = {0};

bool readSHT40(float &temperatureC, float &humidity) {
  sensorWire.beginTransmission(SHT40_ADDR);
  sensorWire.write(0xFD);
  if (sensorWire.endTransmission() != 0) return false;
  delay(10);  // SHT40 high-precision measurement time
  if (sensorWire.requestFrom(SHT40_ADDR, (uint8_t)6) != 6) return false;

  uint8_t data[6];
  for (int i = 0; i < 6; ++i) data[i] = sensorWire.read();
  uint16_t rawTemp = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawHumidity = ((uint16_t)data[3] << 8) | data[4];
  temperatureC = -45.0f + 175.0f * rawTemp / 65535.0f;
  humidity = -6.0f + 125.0f * rawHumidity / 65535.0f;
  humidity = constrain(humidity, 0.0f, 100.0f);
  return true;
}

// Adafruit nRF52 core exposes the internal SAADC VDD channel.
// Default reference = 0.6 V, gain = 1/6, full scale = 3.6 V.
uint16_t readSupplyMillivolts() {
  uint32_t raw = analogReadVDD(); // 12-bit, 0..4095
  uint32_t mv = (raw * 3600UL + 2048UL) / 4096UL;
  return (uint16_t)mv;
}

void startAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(kelvynService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.start(0);
}

void setup() {
  kelvynDebug[0] = 0x4B454C56;
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
  uint8_t initial[6] = {0};
  kelvynTemperature.write(initial, 6);
  startAdvertising();
  kelvynDebug[2] = 1;
}

void loop() {
  uint32_t cycleStart = millis();
  float temperatureC, humidity;

  if (readSHT40(temperatureC, humidity)) {
    int16_t temp100 = (int16_t)(temperatureC * 100.0f);
    uint16_t humidity100 = (uint16_t)(humidity * 100.0f);
    uint16_t supplyMV = readSupplyMillivolts();

    uint8_t packet[6] = {
      (uint8_t)(temp100 & 0xFF), (uint8_t)((temp100 >> 8) & 0xFF),
      (uint8_t)(humidity100 & 0xFF), (uint8_t)((humidity100 >> 8) & 0xFF),
      (uint8_t)(supplyMV & 0xFF), (uint8_t)((supplyMV >> 8) & 0xFF)
    };

    kelvynTemperature.write(packet, 6);
    if (Bluefruit.connected()) kelvynTemperature.notify(packet, 6);

    kelvynDebug[1]++;
    kelvynDebug[3] = (uint32_t)(int32_t)temp100;
    kelvynDebug[4] = humidity100;
    kelvynDebug[5] = supplyMV;
  } else {
    kelvynDebug[2]++;
  }

  // SoftDevice-aware System ON idle between samples.
  // CPU sleeps until an interrupt (e.g. BLE or system tick) wakes it.
  // Unlike System OFF, RAM and the active BLE connection are retained.
  while ((uint32_t)(millis() - cycleStart) < SAMPLE_INTERVAL_MS) {
    (void)sd_app_evt_wait();
  }
}
