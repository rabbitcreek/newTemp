#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// ======================================================
// KELVYN - TEMPERATURE + HUMIDITY + BATTERY
// XIAO nRF52840 + SHT40
//
// NO history
// NO record numbers
// NO flash logging
// NO Start/Stop commands
//
// Wake/read:
//   battery voltage
//   temperature
//   humidity
//
// advertise -> iPhone connects ->
// notify all three values -> disconnect ->
// low-power wait -> repeat
// ======================================================


// ------------------------------------------------------
// XIAO BATTERY MONITOR
// ------------------------------------------------------

#define PIN_VBAT        32
#define PIN_VBAT_ENABLE 14
#define PIN_CHG         23


// ------------------------------------------------------
// SETTINGS
// ------------------------------------------------------

// 5 minutes
#define SAMPLE_INTERVAL_MS 60000UL

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
float currentHumidity = 0.0;
float currentBatteryVoltage = 0.0;

bool temperatureSent = false;


// ======================================================
// LOW POWER WAIT
// ======================================================

void lowPowerWait(unsigned long milliseconds) {

  unsigned long start = millis();

  while (millis() - start < milliseconds) {

    // Sleep CPU until next interrupt/event.
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
// READ BATTERY
// ======================================================

float readBatteryVoltage() {

  // LOW enables the XIAO battery measurement circuit.
  digitalWrite(PIN_VBAT_ENABLE, LOW);

  // Allow voltage divider / ADC input to settle.
  delay(5);

  // Throw away first ADC conversion after enabling.
  analogRead(PIN_VBAT);

  delay(1);

  int vbatt = analogRead(PIN_VBAT);

  // IMPORTANT:
  // Turn battery measurement circuit OFF again.
  // This prevents the divider from remaining enabled
  // throughout the 5-minute sleep period.
  digitalWrite(PIN_VBAT_ENABLE, HIGH);

  // Preserve the calibration formula that was working
  // in your current Arduino program.
  float voltage =
    2.961 * 3.6 * vbatt / 4096.0;

  return voltage;
}


// ======================================================
// READ TEMPERATURE + HUMIDITY
// ======================================================

bool readEnvironment() {

  sensors_event_t humidity;
  sensors_event_t temp;

  if (!sht4.getEvent(&humidity, &temp)) {

    Serial.println("SHT40 read failed.");

    return false;
  }

  currentTemperatureC =
    temp.temperature;

  currentHumidity =
    humidity.relative_humidity;

  float temperatureF =
    currentTemperatureC * 9.0 / 5.0 + 32.0;

  Serial.print("Temperature: ");
  Serial.print(currentTemperatureC, 2);
  Serial.print(" C   ");

  Serial.print(temperatureF, 2);
  Serial.print(" F   ");

  Serial.print("Humidity: ");
  Serial.print(currentHumidity, 2);
  Serial.println(" %");

  return true;
}


// ======================================================
// BUILD 6-BYTE ENVIRONMENT PACKET
//
// Bytes 0-1:
//   signed Int16
//   temperature C x 100
//
// Bytes 2-3:
//   unsigned UInt16
//   relative humidity % x 100
//
// Bytes 4-5:
//   unsigned UInt16
//   battery voltage in millivolts
//
// Examples:
//
//   21.35 C  -> 2135
//   48.27 %  -> 4827
//   3.87 V   -> 3870 mV
// ======================================================

void buildEnvironmentPacket(uint8_t *data) {

  int16_t temp100 =
    (int16_t)round(
      currentTemperatureC * 100.0
    );

  uint16_t humidity100 =
    (uint16_t)round(
      currentHumidity * 100.0
    );

  uint16_t batteryMV =
    (uint16_t)round(
      currentBatteryVoltage * 1000.0
    );


  // Temperature

  data[0] =
    temp100 & 0xFF;

  data[1] =
    (temp100 >> 8) & 0xFF;


  // Humidity

  data[2] =
    humidity100 & 0xFF;

  data[3] =
    (humidity100 >> 8) & 0xFF;


  // Battery voltage

  data[4] =
    batteryMV & 0xFF;

  data[5] =
    (batteryMV >> 8) & 0xFF;
}


// ======================================================
// PREPARE CHARACTERISTIC
// ======================================================

void prepareEnvironmentCharacteristic() {

  uint8_t data[6];

  buildEnvironmentPacket(data);

  temperatureCharacteristic.write(
    data,
    6
  );
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

  // Important for iOS background discovery.
  Bluefruit.Advertising.addService(
    kelvynService
  );

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(
    false
  );

  // 20 ms initially, then 152.5 ms.
  Bluefruit.Advertising.setInterval(
    32,
    244
  );

  Bluefruit.Advertising.setFastTimeout(5);

  // Continue until explicitly stopped.
  Bluefruit.Advertising.start(0);

  Serial.println(
    "Advertising - waiting for iPhone..."
  );
}


// ======================================================
// WAIT FOR IPHONE AND SEND ENVIRONMENT DATA
// ======================================================

void transmitEnvironment() {

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
      // characteristic and enable notifications.

      unsigned long connectionStart =
        millis();

      while (
        Bluefruit.connected() &&
        millis() - connectionStart < 5000
      ) {

        if (
          temperatureCharacteristic
            .notifyEnabled()
        ) {

          uint8_t data[6];

          buildEnvironmentPacket(data);

          temperatureCharacteristic.notify(
            data,
            6
          );

          Serial.println(
            "*** TEMP + HUMIDITY + BATTERY SENT TO IPHONE ***"
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

  // ----------------------------------------------------
  // Battery measurement
  // ----------------------------------------------------

  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_CHG, INPUT);

  // HIGH = battery measurement divider OFF.
  digitalWrite(PIN_VBAT_ENABLE, HIGH);

  analogReference(AR_DEFAULT);
  analogReadResolution(12);


  Serial.println();
  Serial.println("==============================");
  Serial.println("       KELVYN SIMPLE BLE");
  Serial.println(" TEMP + HUMIDITY + BATTERY");
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


  // ----------------------------------------------------
  // Environment characteristic
  //
  // 6 bytes total:
  //
  // Bytes 0-1 = signed temperature C x 100
  // Bytes 2-3 = unsigned RH % x 100
  // Bytes 4-5 = unsigned battery millivolts
  // ----------------------------------------------------

  temperatureCharacteristic.setProperties(
    CHR_PROPS_READ |
    CHR_PROPS_NOTIFY
  );

  temperatureCharacteristic.setPermission(
    SECMODE_OPEN,
    SECMODE_NO_ACCESS
  );

  temperatureCharacteristic.setFixedLen(6);

  temperatureCharacteristic.begin();

  Serial.println("BLE ready.");
}


// ======================================================
// MAIN LOOP
// ======================================================

void loop() {

  Serial.println();
  Serial.println("----- NEW READING -----");


  // ----------------------------------------------------
  // BATTERY
  // ----------------------------------------------------

  currentBatteryVoltage =
    readBatteryVoltage();

  int charging =
    digitalRead(PIN_CHG);

  Serial.print("Voltage: ");
  Serial.print(currentBatteryVoltage, 3);

  Serial.print(" V | Charging: ");

  Serial.println(
    charging == 0
      ? "Yes"
      : "No"
  );


  // ----------------------------------------------------
  // TEMPERATURE + HUMIDITY
  // ----------------------------------------------------

  if (readEnvironment()) {

    prepareEnvironmentCharacteristic();

    transmitEnvironment();
  }


  Serial.print("Waiting ");
  Serial.print(
    SAMPLE_INTERVAL_MS / 1000
  );
  Serial.println(" seconds...");
  Serial.println();


  // Battery measurement circuit is already OFF here.

  lowPowerWait(
    SAMPLE_INTERVAL_MS
  );
}