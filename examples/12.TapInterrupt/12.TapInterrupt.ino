/*!
 * @file 12.TapInterrupt.ino
 * @brief Tap detection interrupt example
 * @details This example demonstrates how to configure and use tap detection interrupt.
 * @n The example configures the sensor to generate an interrupt when a tap is detected.
 * @n Tapping the sensor will trigger the interrupt and display the tap type (single, double, or triple)
 * @n and displays the tap type information.
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
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel6DOF, &mySerial, 9600, ADDR);
#elif defined(ESP32)
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel6DOF, &Serial1, 9600, ADDR, /*rx*/ D2, /*tx*/ D3);
#else
DFRobot_Multi_DOF_IMU_UART imu(DFRobot_Multi_DOF_IMU::eSensorModel6DOF, &Serial1, 9600, ADDR);
#endif
#elif defined(IMU_COMM_I2C)
DFRobot_Multi_DOF_IMU_I2C imu(DFRobot_Multi_DOF_IMU::eSensorModel6DOF, &Wire, ADDR);
#else
#error
#endif

volatile bool tapDetected = false;

#if defined(ESP8266)
void IRAM_ATTR int1ISR()
#else
void int1ISR()
#endif
{
  tapDetected = true;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\nTap Detection Interrupt Example");

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

  Serial.print("[4] Configuring INT1 tap detection interrupt... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin1, DFRobot_Multi_DOF_IMU::eInt1_2Tap)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[5] Configuring Arduino interrupt pin... ");

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
  // Pin 6 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  attachInterrupt(digitalPinToInterrupt(6) /* Query the interrupt number of the 6 pin */, int1ISR, RISING);
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
  Serial.println("Success");
  Serial.println("Trigger mode: Rising edge");
  delay(100);

  Serial.println("\nConfiguration complete, starting tap monitoring");
  Serial.println("Tip: Tap the sensor to trigger interrupt");
  Serial.println("     (Supports single, double, triple tap detection)");

  delay(100);
}

void loop()
{
  if (tapDetected) {
    tapDetected = false;

    uint16_t intStatus = imu.getIntStatus(DFRobot_Multi_DOF_IMU::eImuIntPin1);

    if (intStatus & INT1_2_INT_STATUS_TAP) {
      uint16_t tapData = imu.getTap();

      if (tapData == TAP_TYPE_SINGLE) {
        Serial.print("Single tap (Data: 0x");
        Serial.print(tapData, HEX);
        Serial.println(")");
      } else if (tapData == TAP_TYPE_DOUBLE) {
        Serial.print("Double tap (Data: 0x");
        Serial.print(tapData, HEX);
        Serial.println(")");
      } else if (tapData == TAP_TYPE_TRIPLE) {
        Serial.print("Triple tap (Data: 0x");
        Serial.print(tapData, HEX);
        Serial.println(")");
      } else {
        Serial.print("Unknown tap (Data: 0x");
        Serial.println(tapData, HEX);
      }

    } else if (intStatus != 0) {
      Serial.print("Other interrupt: 0x");
      Serial.println(intStatus, HEX);
    }
  }

  delay(200);
}
