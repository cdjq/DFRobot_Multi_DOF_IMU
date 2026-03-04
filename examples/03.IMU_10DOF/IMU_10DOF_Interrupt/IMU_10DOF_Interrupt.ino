/*!
 * @file IMU_10DOF_Interrupt.ino
 * @brief 10DOF IMU data ready interrupt example
 * @details This example demonstrates how to read 10DOF IMU data using interrupt mode.
 * @n The example configures the sensor to generate interrupts when data is ready from 6DOF, 9DOF, and 10DOF sensors,
 * @n then reads the 10DOF data (acceleration, angular velocity, magnetic field, and pressure) only when interrupts are triggered.
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

/* Altitude calibration (optional): */
#define CALIBRATE_ABSOLUTE_DIFFERENCE
#ifdef CALIBRATE_ABSOLUTE_DIFFERENCE
const bool CALCULATE_ALTITUDE = true;
#else
const bool CALCULATE_ALTITUDE = false;
#endif

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

volatile bool int1DataReady = false;
volatile bool int3DataReady = false;
volatile bool int4DataReady = false;

#if defined(ESP8266)
void IRAM_ATTR int1ISR()
#else
void int1ISR()
#endif
{
  int1DataReady = true;
}

#if defined(ESP8266)
void IRAM_ATTR int3ISR()
#else
void int3ISR()
#endif
{
  int3DataReady = true;
}

#if defined(ESP8266)
void IRAM_ATTR int4ISR()
#else
void int4ISR()
#endif
{
  int4DataReady = true;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\n10DOF IMU Data Ready Interrupt Example");

  Serial.print("\n[1] Initializing sensor... ");
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
  delay(1000);

#if defined(CALIBRATE_ABSOLUTE_DIFFERENCE)
  // Calibrate altitude data based on local altitude (540m in this example)
  Serial.print("[5] Calibrating altitude (reference 540m)... ");
  imu.calibrateAltitude(540.0);
  delay(1000);
#endif

  Serial.print("[6] Configuring INT1 data ready interrupt... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin1, DFRobot_Multi_DOF_IMU::eInt1_2DataReady)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[7] Configuring INT3 data ready interrupt... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin3, DFRobot_Multi_DOF_IMU::eInt3DataReady)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[8] Configuring INT4 data ready interrupt... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin4, DFRobot_Multi_DOF_IMU::eInt4DataReady)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[9] Configuring Arduino interrupt pins... ");

  // Configure INT1 interrupt pin
#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, int1ISR, RISING);
#elif defined(ESP8266)
#if defined(IMU_COMM_UART)
  const uint8_t interruptPin = 12;
#elif defined(IMU_COMM_I2C)
  const uint8_t interruptPin = 13;
#else
#error
#endif
  attachInterrupt(digitalPinToInterrupt(interruptPin), int1ISR, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 5 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  attachInterrupt(digitalPinToInterrupt(5) /* Query the interrupt number of the 5 pin */, int1ISR, RISING);
#else
  /* The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
     * ---------------------------------------------------------------------------------------
     * |                                        |  DigitalPin  | 2  | 3  |                   |
     * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  |                   |
     * |-------------------------------------------------------------------------------------|
     * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
     * |               Mega2560                 |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
     * |-------------------------------------------------------------------------------------|
     * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
     * |    Leonardo, other 32u4-based          |--------------------------------------------|
     * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
     * |--------------------------------------------------------------------------------------
     * ---------------------------------------------------------------------------------------------------------------------------------------------
     *                      The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
     * ---------------------------------------------------------------------------------------------------------------------------------------------
     * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
     * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
     * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
     * |-------------------------------------------------------------------------------------------------------------------------------------------|
     */
  attachInterrupt(/*Interrupt No*/ 0, int1ISR, RISING);    // Open the external interrupt 0, connect INT1 to the digital pin of the main control:
                                                           // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif

  // Configure INT3 interrupt pin
#if defined(ESP32)
  attachInterrupt(digitalPinToInterrupt(16 /*D11*/), int3ISR, RISING);
#elif defined(ESP8266)
  attachInterrupt(digitalPinToInterrupt(15), int3ISR, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  attachInterrupt(digitalPinToInterrupt(7), int3ISR, RISING);
#else
  attachInterrupt(/*Interrupt No*/ 1, int3ISR, RISING);
#endif

  // Configure INT4 interrupt pin
#if defined(ESP32)
  attachInterrupt(digitalPinToInterrupt(17 /*D10*/), int4ISR, RISING);
#elif defined(ESP8266)
  attachInterrupt(digitalPinToInterrupt(16), int4ISR, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  attachInterrupt(digitalPinToInterrupt(9), int4ISR, RISING);
#else
  attachInterrupt(/*Interrupt No*/ 2, int4ISR, RISING);
#endif
  Serial.println("Success");
  Serial.println("Trigger mode: Rising edge");

  Serial.println("\nConfiguration complete, starting data reading");
  Serial.println(CALCULATE_ALTITUDE ? "AccX(g), AccY(g), AccZ(g), GyrX(dps), GyrY(dps), GyrZ(dps), MagX(uT), MagY(uT), MagZ(uT), Altitude(m)" : "AccX(g), AccY(g), AccZ(g), GyrX(dps), GyrY(dps), GyrZ(dps), MagX(uT), MagY(uT), MagZ(uT), Pressure(Pa)");
  delay(100);
}

void loop()
{
  if (int1DataReady || int3DataReady || int4DataReady) {
    uint16_t int1Status = imu.getIntStatus(DFRobot_Multi_DOF_IMU::eImuIntPin1);
    uint16_t int3Status = imu.getIntStatus(DFRobot_Multi_DOF_IMU::eImuIntPin3);
    uint16_t int4Status = imu.getIntStatus(DFRobot_Multi_DOF_IMU::eImuIntPin4);

    bool int1IsDRDY = (int1Status & INT1_2_INT_STATUS_DRDY) != 0;
    bool int3IsDRDY = (int3Status & INT3_INT_STATUS_DRDY) != 0;
    bool int4IsDRDY = (int4Status & INT4_INT_STATUS_DRDY) != 0;

    if (int1IsDRDY && int3IsDRDY && int4IsDRDY) {
      int1DataReady = false;
      int3DataReady = false;
      int4DataReady = false;

      DFRobot_Multi_DOF_IMU::sSensorData_t accel, gyro, mag;
      float                                pressureValue;    // Pressure (Pa) or altitude (m), depends on CALCULATE_ALTITUDE

      if (imu.get10dofData(&accel, &gyro, &mag, &pressureValue, CALCULATE_ALTITUDE)) {
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
        Serial.print(mag.z, 2);
        Serial.print(", ");
        Serial.println(pressureValue, 2);
      } else {
        Serial.println("Failed to read 10DOF data!");
      }
    } else {
      if (int1DataReady && !int1IsDRDY) {
        int1DataReady = false;
      }
      if (int3DataReady && !int3IsDRDY) {
        int3DataReady = false;
      }
      if (int4DataReady && !int4IsDRDY) {
        int4DataReady = false;
      }
    }
  }
}
