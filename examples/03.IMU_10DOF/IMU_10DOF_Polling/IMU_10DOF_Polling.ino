/*!
 * @file IMU_10DOF_Polling.ino
 * @brief Read 10DOF IMU data example (accelerometer + gyroscope + magnetometer + barometer)
 * @details This example demonstrates how to read 10DOF IMU sensor data using I2C/UART interface.
 * @n The example initializes the sensor, configures the accelerometer and gyroscope ranges,
 * @n optionally calibrates the pressure sensor based on local altitude,
 * @n then continuously reads and prints the 10DOF data (acceleration, angular velocity, magnetic field, and pressure) in the loop function.
 * @copyright Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [Martin](Martin@dfrobot.com)
 * @version V1.0.0
 * @date 2026-01-16
 * @url https://github.com/DFRobot/DFRobot_Multi_DOF_IMU
 */

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

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\n10DOF IMU Data Reading Example\n");

  Serial.print("[Init] Initializing sensor... ");
  while (!imu.begin()) {
    Serial.println("Failed, please check device address and connections!");
    delay(1000);
  }
  Serial.println("Success!");
  delay(1000);

  Serial.print("[Config] Setting sensor mode to normal... ");
  while (!imu.setSensorMode(DFRobot_Multi_DOF_IMU::eNormalMode)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[Config] Setting accelerometer range to ±2g... ");
  while (!imu.setAccelRange(DFRobot_Multi_DOF_IMU::eAccelRange2G)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[Config] Setting gyroscope range to ±250dps... ");
  while (!imu.setGyroRange(DFRobot_Multi_DOF_IMU::eGyroRange250DPS)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

#if defined(CALIBRATE_ABSOLUTE_DIFFERENCE)
  // Calibrate pressure sensor based on local altitude (540m in this example)
  Serial.print("[5] Calibrating pressure sensor (altitude 540m)... ");
  imu.calibratePress(540.0);
#endif

  delay(100);
  Serial.println("\nConfiguration complete, starting data reading...\n");
}

void loop()
{
  DFRobot_Multi_DOF_IMU::sSensorData_t accel, gyro, mag;
  float                                pressureValue;    // Pressure data (unit: Pa)

  if (imu.get10dofData(&accel, &gyro, &mag, &pressureValue)) {
    Serial.println("10DOF IMU Data");
    Serial.print("Acc: X=");
    Serial.print(accel.x, 3);
    Serial.print("g Y=");
    Serial.print(accel.y, 3);
    Serial.print("g Z=");
    Serial.print(accel.z, 3);
    Serial.println("g");
    Serial.print("Gyr: X=");
    Serial.print(gyro.x, 2);
    Serial.print("dps Y=");
    Serial.print(gyro.y, 2);
    Serial.print("dps Z=");
    Serial.print(gyro.z, 2);
    Serial.println("dps");
    Serial.print("Mag: X=");
    Serial.print(mag.x, 2);
    Serial.print("uT Y=");
    Serial.print(mag.y, 2);
    Serial.print("uT Z=");
    Serial.print(mag.z, 2);
    Serial.println("uT");
    Serial.print("Pressure: ");
    Serial.print(pressureValue, 2);
    Serial.print("Pa (");
    Serial.print(pressureValue / 100.0, 2);
    Serial.println("hPa)\n");
  } else {
    Serial.println("Error: Failed to read sensor data!");
    Serial.println();
  }

  delay(500);
}
