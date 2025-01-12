/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 * 
 * DESCRIPTION
 * This is adapted from the (BME280-based) PressureSensor and BatteryPoweredSensor examples from the MySensors distribution.
 *
 */

// if you uncomment this, you can get test and debug updates about everything the sensor is doing by using the serial monitor tool.
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24                            // A 2.4Ghz transmitter and receiver, often used with MySensors.
// #define MY_RF24_PA_LEVEL RF24_PA_MIN              // This sets a low-power mode for the radio. Useful if you use the version with the bigger antenna, but don't want to power that from a separate power source. It can also fix problems with fake Chinese versions of the radio.
// #define MY_RADIO_RFM69                         // 433Mhz transmitter and reveiver.

// Do you want this sensor to also be a repeater?
// #define MY_REPEATER_FEATURE                    // Just remove the two slashes at the beginning of this line to also enable this sensor to act as a repeater for other sensors. If this node is on battery power, you probably shouldn't enable this.

// Are you using this sensor on battery power?
#define BATTERY_POWERED                        // Just remove the two slashes at the beginning of this line if your node is battery powered. It will then go into deep sleep as much as possible. While it's sleeping it can't work as a repeater!
#define BATTERY_SENSE_PIN A0

// Would you like the sensor to generate a weather forecast based on the barometric pressure? 
#define GENERATE_FORECAST


// LIBRARIES
#include <SPI.h>                                  // A communication backbone, the Serial Peripheral Interface.
#include <MySensors.h>                            // The MySensors library. Hurray!
#include <Wire.h>                                 // Enables the Wire communication protocol.
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>                    // alternative library you could try (DIY; no code for this is in here yet).
//#include <SparkFunBMP280.h>                     // alternative library you could try (DIY; no code for this is in here yet).


// VARIABLES YOU CAN CHANGE
const float ALTITUDE = 65;                        // Change this value to your location's altitude (in m). Use your smartphone GPS to get an accurate value, or use an online map.
unsigned long measurementInterval = 60000;  // Sleep time between reads (in ms). Keep this value at 60000 if you have enabled the forecast feature, as the forecast algorithm needs a sample every minute.

#define OMIT_IF_UNCHANGED false
float TemperatureThreshold = 0.1;                        // How big a temperature difference has to minimally  be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.
float HumidityThreshold = 0.1;                         // How big a humidity difference has to minimally be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.
float PressureThreshold = 0.1;                        // How big a barometric difference has to minimally be before an update is sent. Makes the sensor less precise, but also less jittery, and can save battery.            

// VARIABLES YOU PROBABLY SHOULDN'T CHANGE
#define AHT20_TEMP_CHILD_ID 0
#define AHT20_HUM_CHILD_ID  1

#define NO_DATA (-999.9)

float lastAHT20Temperature = NO_DATA;
float lastAHT20Humidity    = NO_DATA;

#define BMP280_TEMP_CHILD_ID 2
#define BMP280_BARO_CHILD_ID 3

float lastBMP280Temperature = NO_DATA;
float lastBMP280Pressure    = NO_DATA;

unsigned long measurementSleepTime = 0;  	// variable to store the calculated Sleep time if the node is battery powered.
bool metric = true;				// Variable that stores if the sensor will output the temperature in Fahrenheit of Celsius. The gateway sends this preference to the node, so you dont need to change it here.
bool receivedConfig = false;			// The MySensors gateway will tell the node if it should output in metric or not.

#define CONVERSION_FACTOR (1.0/10.0) 		// used by forecast algorithm to convert from Pa to kPa, by dividing hPa by 10.

#ifdef GENERATE_FORECAST 			//  Below you will find a lot of variables used by the forecast algorithm.
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,     				// "Stable Weather Pattern"
  SUNNY = 1,      				// "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,     				// "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,   				// "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4, 				// "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5     				// "Unknown (More Time needed)
};
int lastForecast = -1;				// Stores the previous forecast, so it can be compared with a new forecast.
const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
int minuteCount = 0;				// Helps the forecast algorithm keep time.
bool firstRound = true;				// Helps the forecast algorithm recognise if the sensor has just been powered up.
float pressureAvg;				// Average value is used in forecast algorithm.
float pressureAvg2;				// Average after 2 hours is used as reference value for the next iteration.
float dP_dt;					// Pressure delta over time
#endif

// MYSENSORS COMMUNICATION VARIABLES
MyMessage AHT20TemperatureMsg(AHT20_TEMP_CHILD_ID, V_TEMP);
MyMessage AHT20HumidityMsg   (AHT20_HUM_CHILD_ID, V_HUM);
MyMessage BMP280TemperatureMsg(BMP280_TEMP_CHILD_ID, V_TEMP);
MyMessage BMP280PressureMsg   (BMP280_BARO_CHILD_ID, V_PRESSURE);
#ifdef GENERATE_FORECAST
MyMessage BMP280ForecastMsg(BMP280_BARO_CHILD_ID, V_FORECAST);
#endif

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp; // I2C

void setup() {
  Wire.begin(); // Wire.begin(sda, scl) // starts the wire communication protocol, used to chat with the BMP280 sensor.
  Serial.begin(115200); // for serial debugging over USB.
  Serial.println("Temperature/Humidity/Pressure sensor using combined AHT20+BMP280 module");

#ifdef BATTERY_POWERED // If the node is battery powered, we'll let Sleep take over the scheduling.
   measurementSleepTime = measurementInterval;
   measurementInterval = 0; // When the Arduino is asleep, millis doesn't increment anymore (time stops as it were). To fix this, we'll set the measurement interval time to 1, so that when the arduino wakes up it will immediately try to measure again.
#endif

  // Initialize AHT

  if (!aht.begin()) {
    Serial.println(F("Could not find a valid AHTx0 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  // Initialize BMP

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  Serial.print(F("BMP sensor found, id="));
  Serial.println(bmp.sensorID());

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,      /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X4,      /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */

}


void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("AHT20+BMP280 Sensor", "1.0");

  // Tell the MySensors gateway what kind of sensors this node has, and what their ID's on the node are, as defined in the code above.
  present(AHT20_TEMP_CHILD_ID, S_TEMP);
  present(AHT20_HUM_CHILD_ID, S_HUM);
  present(BMP280_TEMP_CHILD_ID, S_TEMP);
  present(BMP280_BARO_CHILD_ID, S_BARO);
}

#define REPORT(sensor, quantity, unit) do { \
  Serial.print(F(#sensor " " #quantity ": "));     \
  Serial.print(current ## sensor ## quantity);                             \
  Serial.print(F(" " unit ": "));                           \
\
  if (OMIT_IF_UNCHANGED && (abs(current ## sensor ## quantity - last ## sensor ## quantity) < quantity ## Threshold)) { \
    Serial.println(F("omitted (too close to previous measurement)"));                                    \
  } else {                                                                                               \
    send(sensor ## quantity ## Msg.set(current ## sensor ## quantity, 1));                                                       \
    Serial.println(F("sent"));                                                                           \
  } \
} while(0);

void loop() {

  // You should not change these variables:
  static unsigned long previousMeasMillis = 0;  // Used to remember the time that the BMP280 sensor was asked for a measurement.
  unsigned long currentMeasMillis = millis();         // The time since the sensor started, counted in milliseconds. This script tries to avoid using the Sleep function, so that it could at the same time be a MySensors repeater.

  // PART 1. If enough time has passed, a new measurement should be taken:
  if (currentMeasMillis - previousMeasMillis >= measurementInterval) {
    previousMeasMillis = currentMeasMillis; // store the current time as the previous measurement start time.
  
    Serial.println(F("\nMeasuring..."));

    bmp.takeForcedMeasurement();
    // Blocks till measurement is ready

    float currentBMP280Temperature = bmp.readTemperature();
    float pressure_local = bmp.readPressure();
    float currentBMP280Pressure = bmp.seaLevelForAltitude(ALTITUDE, pressure_local);

    REPORT(BMP280, Temperature, "°C");
    REPORT(BMP280, Pressure, "hPa (sea level)");

#ifdef GENERATE_FORECAST      
    int forecast = sample(currentBMP280Pressure);						// Run the forecast function with a new pressure update.

    if (forecast != lastForecast) {
      Serial.print(F("BMP280 forecast: "));
      Serial.println(weather[forecast]);
      send(BMP280ForecastMsg.set(forecast));
    }
#endif
  // Measurement cycle ends here
  }

  // You can do other stuff here

#ifdef BATTERY_POWERED
  // Report battery percentage
	// 1M, 470K divider across battery and using internal ADC ref of 1.1V
	// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
	// ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
	// 3.44/1023 = Volts per bit = 0.003363075

  int batteryValue = analogRead(BATTERY_SENSE_PIN);
  int batteryPcnt = batteryValue / 10;

  #ifdef MY_DEBUG
    Serial.print(F("Battery level: read value="));
    Serial.print(batteryValue);
    Serial.print(F(" / voltage="));
    Serial.print((float) batteryValue * 0.003363075);
  	Serial.print(F(" V / percentage="));
	  Serial.print(batteryPcnt);
    Serial.println(F(" %"));
  #endif

  sendBatteryLevel(batteryPcnt);
  unsigned long timeInCycle = millis() - previousMeasMillis;
  Serial.print(F("Time in measurement cycle: "));
  Serial.print(timeInCycle);
  Serial.println(F(" ms"));

  unsigned long sleeptime = measurementInterval - timeInCycle;
  Serial.print(F("ZzzzZZZZzzzzZZZZzzzz for"));
  Serial.print(sleeptime);
  Serial.println(F(" ms"));

  sleep(sleeptime); // Note: millis() stops counting during sleep

  Serial.println(F("Waking up"));
#endif

} // end of main loop.



#ifdef GENERATE_FORECAST
// These functions are only included if the forecast function is enables. The are used to generate a weater prediction by checking if the barometric pressure is rising or falling over time.

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}


// Forecast algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure) {
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185) {
    minuteCount = 6;
  }

  if (minuteCount == 5) {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour 
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { //first time initial 3 hour
      dP_dt = change; //note this is for t = 1 hour
    }
    else {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    } 
    else {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 3; // note this is for t = 3 hour
    } 
    else {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) { //if time is less than 35 min on the first 3 hour interval.
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25)) {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25) {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05))) {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25)) {
    forecast = SUNNY;
  }
  else if ((dP_dt >(-0.05)) && (dP_dt < 0.05)) {
    forecast = STABLE;
  }
  else {
    forecast = UNKNOWN;
  }

  // uncomment when debugging
  //Serial.print(F("BMP280 - Forecast at minute "));
  //Serial.print(minuteCount);
  //Serial.print(F(" dP/dt = "));
  //Serial.print(dP_dt);
  //Serial.print(F("kPa/h --> "));
  //Serial.println(weather[forecast]);

  return forecast;
}
#endif
