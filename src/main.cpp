#include "Arduino.h"
#include "ESP32AnalogRead.h"
#include "SSD1306Wire.h"
#include "BluetoothSerial.h"
#include "ArduinoJson.h"
#include "RunningAverage.h"
#include "SD.h"
#include <millisDelay.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

const unsigned long STATUS_DELAY_MS = 1000;
millisDelay statusDelay;
const unsigned long STREAMING_DELAY_MS = 33;
millisDelay streamingDelay;
const unsigned long LOGGING_DELAY_MS = 33;
millisDelay loggingDelay;

BluetoothSerial SerialBT;
ESP32AnalogRead ntcAdc;
ESP32AnalogRead batteryAdc;

const String dataLogFileName = "/datalog.jsonl";

const int NTC_PIN = 33;
const int SERVO_PIN = 34;
const int BATTERY_PIN = 35;

const double ESPVoltage = 3.3;
const double voltageDividerResistor = 21500.0; // voltage divider resistor value
const double NTCBeta = 3950.0;                 // Beta value
const double To = 298.15;                      // Temperature in Kelvin for 25 degree Celsius
const double nominalNTCResistance = 100000.0;  // Resistance of Thermistor at 25 degree Celsius

volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseDuration = 0;

int minServoDurationEver = 3000;
int maxServoDurationEver = 0;

SSD1306Wire display(0x3D, SDA, SCL);
RunningAverage averageServoPosition(24);
RunningAverage averageBatteryVoltage(24);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void writeFile(fs::FS &fs, StaticJsonDocument<256> json)
{
  File file = fs.open(dataLogFileName, FILE_APPEND, true);
  serializeJson(json, file);
  file.println();
  file.close();
}

float getBatteryVoltage()
{
  averageBatteryVoltage.add(batteryAdc.readRaw() / 4096.0 * 7.445);
  return averageBatteryVoltage.getAverage();
}

int getTempCelsious()
{
  double adcVolts = ntcAdc.readVoltage();
  double thermometerResistance = voltageDividerResistor * adcVolts / (ESPVoltage - adcVolts);
  return 1 / (1 / To + log(thermometerResistance / nominalNTCResistance) / NTCBeta) - 273.15;
}

int getMappedAverageServoPercent(long pulseDuration)
{
  const int deadBand = 2;
  const int upperLimit = 100;
  const int lowerLimit = 0;
  const int peakDetectionThreshold = 25;

  if (pulseDuration > maxServoDurationEver + peakDetectionThreshold)
  {
    maxServoDurationEver = pulseDuration;
  }
  else if (pulseDuration < minServoDurationEver - peakDetectionThreshold)
  {
    minServoDurationEver = pulseDuration;
  }

  int servoPercent = map(pulseDuration, minServoDurationEver, maxServoDurationEver, lowerLimit, upperLimit);
  averageServoPosition.add(servoPercent);
  int avgServoPosition = averageServoPosition.getAverage();
  if (avgServoPosition > upperLimit - deadBand)
  {
    return upperLimit;
  }
  else if (avgServoPosition < lowerLimit + deadBand)
  {
    return lowerLimit;
  }
  else
  {
    return avgServoPosition;
  }
}

void IRAM_ATTR buttonPinInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(SERVO_PIN) == HIGH)
  {
    // start measuring
    pulseInTimeBegin = micros();
  }
  else
  {
    // stop measuring
    pulseDuration = micros() - pulseInTimeBegin;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup()
{
  // Serial.begin(9600);
  SerialBT.begin("ESP32test");
  SD.begin();

  ntcAdc.attach(NTC_PIN);
  batteryAdc.attach(BATTERY_PIN);
  pinMode(SERVO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SERVO_PIN),
                  buttonPinInterrupt,
                  CHANGE);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  averageServoPosition.clear();
  averageBatteryVoltage.clear();
  statusDelay.start(STATUS_DELAY_MS);
  streamingDelay.start(STREAMING_DELAY_MS);
  loggingDelay.start(LOGGING_DELAY_MS);
}

void loop()
{

  display.clear();

  int tempCelsious = getTempCelsious();
  int avgServoPosition = getMappedAverageServoPercent(pulseDuration);
  float batteryVoltage = getBatteryVoltage();

  display.drawString(0, 0, String(tempCelsious) + "  " + avgServoPosition + "  " + String(pulseDuration) + "  " + String(batteryVoltage));
  if (SerialBT.connected())
  {
    display.drawString(0, 25, "BT Connected");
  }
  else
  {
    display.drawString(0, 25, "BT Disconnected");
  }

  if (statusDelay.justFinished())
  {
    statusDelay.repeat();
    StaticJsonDocument<256> statusJson;
    statusJson["isLogging"] = loggingDelay.isRunning();
    statusJson["isStreaming"] = streamingDelay.isRunning();
    serializeJson(statusJson, SerialBT);
    SerialBT.println();
  }

  if (SerialBT.available())
  {
    String btCommand = SerialBT.readStringUntil('\n');
    btCommand.trim();

    if (btCommand == "SEND_LOG")
    {
      File file = SD.open(dataLogFileName);
      while (file.available())
      {
        SerialBT.write(file.read());
      }
      file.close();
    }
    else if (btCommand == "CLEAR_LOG")
    {
      SD.remove(dataLogFileName);
    }
    else if (btCommand == "START_LOG")
    {
      loggingDelay.start(LOGGING_DELAY_MS);
    }
    else if (btCommand == "STOP_LOG")
    {
      loggingDelay.stop();
    }
    else if (btCommand == "START_STREAM")
    {
      streamingDelay.start(STREAMING_DELAY_MS);
    }
    else if (btCommand == "STOP_STREAM")
    {
      streamingDelay.stop();
    }
  }

  StaticJsonDocument<256> telemetryJson;
  telemetryJson["eTmp"] = tempCelsious;
  telemetryJson["trPrc"] = avgServoPosition;
  telemetryJson["ts"] = millis();

  if (loggingDelay.justFinished())
  {
    loggingDelay.repeat();
    writeFile(SD, telemetryJson);
  }

  if (streamingDelay.justFinished())
  {
    streamingDelay.repeat();
    serializeJson(telemetryJson, SerialBT);
    SerialBT.println();
  }

  display.display();
}
