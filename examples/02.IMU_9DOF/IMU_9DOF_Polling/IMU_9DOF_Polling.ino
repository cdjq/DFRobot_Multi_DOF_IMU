/*!
 * @file IMU_9DOF_Polling.ino
 * @brief Read 9DOF IMU data example (accelerometer + gyroscope + magnetometer)
 * @details This example demonstrates how to read 9DOF IMU sensor data using I2C or UART interface.
 * @n The example initializes the sensor, configures the accelerometer and gyroscope ranges,
 * @n then continuously reads and prints the 9DOF data (acceleration, angular velocity, and magnetic field) in the loop function.
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
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel9DOF, &mySerial, 9600, ADDR);
#elif defined(ESP32)
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel9DOF, &Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
#else
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel9DOF, &Serial1, 9600, ADDR);
#endif
#elif defined(IMU_COMM_I2C)
DFRobot_Multi_DOF_IMU_I2C imu(DFRobot_Multi_DOF_IMU::eSensorModel9DOF, &Wire, ADDR);
#else
#error
#endif

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\n9DOF IMU Polling Example");

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

  Serial.print("[4] Setting gyroscope range to ±250dps... ");
  while (!imu.setGyroRange(DFRobot_Multi_DOF_IMU::eGyroRange250DPS)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(100);

  Serial.println("\nConfiguration complete, starting data reading");
  Serial.println("AccX(g), AccY(g), AccZ(g), GyrX(dps), GyrY(dps), GyrZ(dps), MagX(uT), MagY(uT), MagZ(uT)\n");
}

void loop()
{
  DFRobot_Multi_DOF_IMU::sSensorData_t accel, gyro, mag;

  if (imu.get9dofData(&accel, &gyro, &mag)) {
    Serial.print(accel.x, 3);
    Serial.print(", ");
    Serial.print(accel.y, 3);
    Serial.print(", ");
    Serial.print(accel.z, 3);
    Serial.print(", ");
    Serial.print(gyro.x, 2);
    Serial.print(", ");
    Serial.print(gyro.y, 2);
    Serial.print(", ");
    Serial.print(gyro.z, 2);
    Serial.print(", ");
    Serial.print(mag.x, 2);
    Serial.print(", ");
    Serial.print(mag.y, 2);
    Serial.print(", ");
    Serial.println(mag.z, 2);
  } else {
    Serial.println("Failed to read data!");
  }

  delay(500);
}
