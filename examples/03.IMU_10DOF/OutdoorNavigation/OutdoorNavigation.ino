/*!
 * @file OutdoorNavigation.ino
 * @brief Outdoor navigation system application example
 * @details This example demonstrates how to use 10DOF IMU sensor to implement outdoor navigation functionality.
 * @n The example configures the step counter interrupt and periodically reads the magnetometer and barometer data.
 * @n It calculates heading from magnetometer, counts steps from the step counter, and calculates altitude from pressure,
 * @n then outputs the direction name, heading angle, step count, and altitude at regular intervals.
 * @copyright Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [Martin](Martin@dfrobot.com)
 * @version V1.0.0
 * @date 2026-01-16
 * @url https://github.com/DFRobot/DFRobot_Multi_DOF_IMU
 */

#include <math.h>

#include "DFRobot_Multi_DOF_IMU.h"

/* Select communication mode (uncomment one): */
// #define IMU_COMM_UART
#define IMU_COMM_I2C

/* Pressure calibration (optional): */
#define CALIBRATE_ABSOLUTE_DIFFERENCE

const uint8_t ADDR = 0x4A;

#if defined(IMU_COMM_UART)
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  26/D3|     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  25/D2|     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial             mySerial(/*rx =*/4, /*tx =*/5);
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel10DOF, &mySerial, 9600, ADDR);
#elif defined(ESP32)
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel10DOF, &Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
#else
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel10DOF, &Serial1, 9600, ADDR);
#endif
#elif defined(IMU_COMM_I2C)
DFRobot_Multi_DOF_IMU_I2C imu(DFRobot_Multi_DOF_IMU::eSensorModel10DOF, &Wire, ADDR);
#else
#error
#endif

const char* directionNames[] = { "North", "Northeast", "East", "Southeast", "South", "Southwest", "West", "Northwest" };

/**
 * @brief Calculate heading based on magnetometer data
 * @param magX Magnetometer X-axis data (unit: uT, typically points east)
 * @param magY Magnetometer Y-axis data (unit: uT, typically points north)
 * @return float Heading angle (0-360 degrees, 0°=North, 90°=East, 180°=South, 270°=West)
 */
float calculateHeading(float magX, float magY)
{
  float heading = atan2(magX, magY);
  heading       = heading * 180.0f / PI;

  if (heading < 0) {
    heading += 360.0f;
  }

  const float declination = 0.0f;    // Magnetic declination (degrees)
  heading += declination;

  if (heading >= 360.0f) {
    heading -= 360.0f;
  } else if (heading < 0) {
    heading += 360.0f;
  }

  return heading;
}

/**
 * @brief Get direction name based on heading angle
 * @param heading Heading angle (0-360 degrees)
 * @return const char* Direction name string
 */
const char* getDirectionName(float heading)
{
  int index = (int)((heading + 22.5f) / 45.0f) % 8;
  return directionNames[index];
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\nOutdoor Navigation System Example");
  Serial.println("Features: Compass (magnetometer) + Step counter + Altimeter");
  Serial.println("Usage: Place sensor horizontally, start walking");
  Serial.println("Output: Direction | Heading(°) | Steps | Altitude(m)\n");

  Serial.print("[1] Initializing sensor... ");
  while (!imu.begin()) {
    Serial.println("Failed, please check device address and connections!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[2] Setting sensor mode to normal... ");
  while (!imu.setSensorMode(DFRobot_Multi_DOF_IMU::eNormalMode)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[3] Setting accelerometer range to ±2G... ");
  while (!imu.setAccelRange(DFRobot_Multi_DOF_IMU::eAccelRange2G)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[4] Configuring step counter (INT1)... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin1, DFRobot_Multi_DOF_IMU::eInt1_2StepCounter)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

#if defined(CALIBRATE_ABSOLUTE_DIFFERENCE)
  // Calibrate pressure sensor based on local altitude (540m in this example)
  Serial.print("[5] Calibrating pressure sensor (altitude 540m)... ");
  imu.calibrateAltitude(540.0);
  delay(100);
#endif

  Serial.println("Format: Direction | Heading(°) | Steps | Altitude(m)\n");

  delay(100);
}

void loop()
{
  static unsigned long lastUpdate = 0;
  unsigned long        now        = millis();

  if (now - lastUpdate >= 1000) {    // Update every 1 second
    lastUpdate = now;

    DFRobot_Multi_DOF_IMU::sSensorData_t accel, gyro, mag;
    float                                altitude;    // Altitude (unit: m)

    if (imu.get10dofData(&accel, &gyro, &mag, &altitude, true)) {
      float       heading   = calculateHeading(mag.x, mag.y);
      const char* direction = getDirectionName(heading);
      uint32_t    stepCount = imu.getStepCount();

      Serial.print("Direction: ");
      Serial.print(direction);
      Serial.print(" (");
      Serial.print(heading, 1);
      Serial.print("°)");
      Serial.print(" | Steps: ");
      Serial.print(stepCount);
      Serial.print(" | Altitude: ");
      Serial.print(altitude, 1);
      Serial.println(" m");
    } else {
      Serial.println("Failed to read sensor data!");
    }
  }

  delay(500);
}
