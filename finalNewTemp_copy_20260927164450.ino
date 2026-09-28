#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// ======================================================
// KELVYN - SIMPLE TEMPERATURE TRANSMITTER
// XIAO nRF52840 + SHT40
//
// NO history
// NO record numbers
// NO flash logging
// NO Start/Stop commands
//
// Wake/read -> advertise -> iPhone connects ->
// notify temperature -> disconnect -> wait -> repeat
// ======================================================


// ------------------------------------------------------
// TEST SETTINGS
// ------------------------------------------------------

// 10 seconds for testing.
// Change to 300000UL for 5 minutes after testing.
#define SAMPLE_INTERVAL_MS 300000UL

// Give the iPhone plenty of time to find Kelvyn.
#define CONNECTION_WINDOW_MS 30000UL


// ------------------------------------------------------
// BLE UUIDs
// ------------------------------------------------------

#define KELVYN_SERVICE_UUID \
  "A6E10001-7A4B-4E36-9F3C-123456789ABC"

#define TEMPERATURE_CHAR_UUID \
  "A6E10002-7A4B-4E36-9F3C-123456789ABC"


// ------------------------------------------------------
// Objects
// ------------------------------------------------------

Adafruit_SHT4x sht4;

BLEService kelvynService(KELVYN_SERVICE_UUID);
BLECharacteristic temperatureCharacteristic(
  TEMPERATURE_CHAR_UUID
);


// ------------------------------------------------------
// State
// ------------------------------------------------------

volatile bool phoneConnected = false;

float currentTemperatureC = 0.0;

bool temperatureSent = false;


// ======================================================
// LOW POWER WAIT
// ======================================================

void lowPowerWait(unsigned long milliseconds) {

  unsigned long start = millis();

  while (millis() - start < milliseconds) {

    // Puts CPU to sleep until an interrupt/event occurs.
    // RTC/BLE interrupts wake it briefly.
    sd_app_evt_wait();
  }
}


// ======================================================
// BLE CALLBACKS
// ======================================================

void connectCallback(uint16_t connHandle) {

  phoneConnected = true;
  temperatureSent = false;

  Serial.println();
  Serial.println("*** IPHONE CONNECTED ***");
}


void disconnectCallback(
  uint16_t connHandle,
  uint8_t reason
) {

  phoneConnected = false;

  Serial.println("*** IPHONE DISCONNECTED ***");
}


// ======================================================
// READ TEMPERATURE
// ======================================================

bool readTemperature() {

  sensors_event_t humidity;
  sensors_event_t temp;

  if (!sht4.getEvent(&humidity, &temp)) {

    Serial.println("SHT40 read failed.");
    return false;
  }

  currentTemperatureC = temp.temperature;

  float temperatureF =
    currentTemperatureC * 9.0 / 5.0 + 32.0;

  Serial.print("Temperature: ");
  Serial.print(currentTemperatureC, 2);
  Serial.print(" C   ");
  Serial.print(temperatureF, 2);
  Serial.println(" F");

  return true;
}


// ======================================================
// PREPARE TEMPERATURE CHARACTERISTIC
// ======================================================

void prepareTemperatureCharacteristic() {

  int16_t temp100 =
    (int16_t)round(currentTemperatureC * 100.0);

  uint8_t data[2];

  data[0] = temp100 & 0xFF;
  data[1] = (temp100 >> 8) & 0xFF;

  temperatureCharacteristic.write(data, 2);
}


// ======================================================
// ADVERTISE
// ======================================================

void startAdvertising() {

  Bluefruit.Advertising.stop();

  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE
  );

  Bluefruit.Advertising.addTxPower();

  // Very important:
  // advertise the Kelvyn service so iOS can find it
  // while the app is in the background.
  Bluefruit.Advertising.addService(kelvynService);

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(false);

  // Advertising interval:
  // 32 = 20 ms fast advertising
  // 244 = 152.5 ms slower advertising
  Bluefruit.Advertising.setInterval(32, 244);

  Bluefruit.Advertising.setFastTimeout(5);

  // 0 means continue until we explicitly stop it.
  Bluefruit.Advertising.start(0);

  Serial.println("Advertising - waiting for iPhone...");
}


// ======================================================
// WAIT FOR IPHONE AND SEND TEMPERATURE
// ======================================================

void transmitTemperature() {

  phoneConnected = false;
  temperatureSent = false;

  startAdvertising();

  unsigned long startTime = millis();

  while (
    millis() - startTime <
    CONNECTION_WINDOW_MS
  ) {

    if (Bluefruit.connected()) {

      phoneConnected = true;

      // Give iOS time to discover the service,
      // discover the characteristic and enable
      // notifications.
      unsigned long connectionStart = millis();

      while (
        Bluefruit.connected() &&
        millis() - connectionStart < 5000
      ) {

        if (
          temperatureCharacteristic.notifyEnabled()
        ) {

          int16_t temp100 =
            (int16_t)round(
              currentTemperatureC * 100.0
            );

          uint8_t data[2];

          data[0] = temp100 & 0xFF;
          data[1] = (temp100 >> 8) & 0xFF;

          temperatureCharacteristic.notify(
            data,
            2
          );

          Serial.println(
            "*** TEMPERATURE SENT TO IPHONE ***"
          );

          temperatureSent = true;

          // Give BLE stack time to transmit.
          delay(300);

          Bluefruit.disconnect(
            Bluefruit.connHandle()
          );

          break;
        }

        sd_app_evt_wait();
      }

      break;
    }

    sd_app_evt_wait();
  }


  Bluefruit.Advertising.stop();


  if (!temperatureSent) {

    Serial.println(
      "No iPhone connection - reading skipped."
    );
  }

  Serial.println();
}


// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("       KELVYN SIMPLE BLE");
  Serial.println("==============================");


  // ----------------------------------------------------
  // I2C / SHT40
  // ----------------------------------------------------

  Wire.begin();

  if (!sht4.begin()) {

    Serial.println("SHT40 not found!");

    while (1) {
      delay(1000);
    }
  }

  sht4.setPrecision(
    SHT4X_HIGH_PRECISION
  );

  sht4.setHeater(
    SHT4X_NO_HEATER
  );

  Serial.println("SHT40 ready.");


  // ----------------------------------------------------
  // BLE
  // ----------------------------------------------------

  Bluefruit.begin();

  Bluefruit.setName("Kelvyn");

  Bluefruit.Periph.setConnectCallback(
    connectCallback
  );

  Bluefruit.Periph.setDisconnectCallback(
    disconnectCallback
  );


  // ----------------------------------------------------
  // Kelvyn service
  // ----------------------------------------------------

  kelvynService.begin();


  // Temperature:
  // signed Int16
  // Celsius x 100
  //
  // Example:
  // 21.35 C -> 2135
  // ----------------------------------------------------

  temperatureCharacteristic.setProperties(
    CHR_PROPS_READ |
    CHR_PROPS_NOTIFY
  );

  temperatureCharacteristic.setPermission(
    SECMODE_OPEN,
    SECMODE_NO_ACCESS
  );

  temperatureCharacteristic.setFixedLen(2);

  temperatureCharacteristic.begin();


  Serial.println("BLE ready.");
}


// ======================================================
// MAIN LOOP
// ======================================================

void loop() {

  Serial.println();
  Serial.println("----- NEW TEMPERATURE -----");

  if (readTemperature()) {

    prepareTemperatureCharacteristic();

    transmitTemperature();
  }


  Serial.print("Waiting ");
  Serial.print(
    SAMPLE_INTERVAL_MS / 1000
  );
  Serial.println(" seconds...");
  Serial.println();

  lowPowerWait(
    SAMPLE_INTERVAL_MS
  );
}