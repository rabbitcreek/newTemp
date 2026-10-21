#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>

// BMD-340 physical GPIO numbers
#define SDA_PIN 26
#define SCL_PIN 27

// SHT40 I2C address
#define SHT40_ADDR 0x44

// Dedicated I2C interface
TwoWire sensorWire(
  NRF_TWIM0,
  NRF_TWIS0,
  SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn,
  SDA_PIN,
  SCL_PIN
);

// Kelvyn BLE UUIDs
BLEService kelvynService(
  "A6E10001-7A4B-4E36-9F3C-123456789ABC"
);

BLECharacteristic kelvynTemperature(
  "A6E10002-7A4B-4E36-9F3C-123456789ABC"
);

// Debug information readable through J-Link
volatile uint32_t kelvynDebug[6] __attribute__((used)) = {0};

// Read temperature and humidity
bool readSHT40(float &temperatureC, float &humidity)
{
  sensorWire.beginTransmission(SHT40_ADDR);
  sensorWire.write(0xFD);  // High precision measurement

  if (sensorWire.endTransmission() != 0)
    return false;

  delay(10);

  if (sensorWire.requestFrom(SHT40_ADDR, 6) != 6)
    return false;

  uint8_t data[6];

  for (int i = 0; i < 6; i++)
    data[i] = sensorWire.read();

  uint16_t rawTemp =
      ((uint16_t)data[0] << 8) | data[1];

  uint16_t rawHumidity =
      ((uint16_t)data[3] << 8) | data[4];

  temperatureC =
      -45.0f + 175.0f * rawTemp / 65535.0f;

  humidity =
      -6.0f + 125.0f * rawHumidity / 65535.0f;

  humidity = constrain(humidity, 0.0f, 100.0f);

  return true;
}

void startAdvertising()
{
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();

  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
  );

  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(kelvynService);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);

  // Advertise continuously
  Bluefruit.Advertising.start(0);
}

void setup()
{
  kelvynDebug[0] = 0x4B454C56; // "KELV"
  kelvynDebug[2] = 1;

  // Initialize SHT40
  sensorWire.begin();
  sensorWire.setClock(100000);

  kelvynDebug[2] = 2;

  // Initialize Bluetooth
  Bluefruit.begin();
  kelvynDebug[2] = 3;

  Bluefruit.setName("Kelvyn");

  // Create BLE service
  kelvynService.begin();

  kelvynTemperature.setProperties(
    CHR_PROPS_READ | CHR_PROPS_NOTIFY
  );

  kelvynTemperature.setPermission(
    SECMODE_OPEN,
    SECMODE_NO_ACCESS
  );

  // Temperature + humidity + battery placeholder
  kelvynTemperature.setFixedLen(6);
  kelvynTemperature.begin();

  uint8_t initial[6] = {0};
  kelvynTemperature.write(initial, 6);

  kelvynDebug[2] = 4;

  startAdvertising();

  kelvynDebug[2] = 5;
}

void loop()
{
  float temperatureC;
  float humidity;

  if (readSHT40(temperatureC, humidity))
  {
    int16_t temp100 =
        (int16_t)(temperatureC * 100.0f);

    uint16_t humidity100 =
        (uint16_t)(humidity * 100.0f);

    uint16_t batteryMV = 0;

    uint8_t packet[6] = {
      (uint8_t)(temp100 & 0xFF),
      (uint8_t)((temp100 >> 8) & 0xFF),

      (uint8_t)(humidity100 & 0xFF),
      (uint8_t)((humidity100 >> 8) & 0xFF),

      (uint8_t)(batteryMV & 0xFF),
      (uint8_t)((batteryMV >> 8) & 0xFF)
    };

    kelvynTemperature.write(packet, 6);

    if (Bluefruit.connected())
    {
      kelvynTemperature.notify(packet, 6);
    }

    kelvynDebug[1]++;
    kelvynDebug[3] = (uint32_t)(int32_t)temp100;
    kelvynDebug[4] = humidity100;
  }
  else
  {
    kelvynDebug[5]++;
  }

  delay(2000);
}