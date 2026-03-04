/*!
 * @file 05.SmartWakeSleep.ino
 * @brief Smart wake/sleep example - combining significant motion and flat detection
 * @details This example demonstrates how to use both significant motion and flat detection to implement smart wake/sleep functionality.
 * @n The example configures INT1 for significant motion detection (wake) and INT2 for flat detection (sleep).
 * @n When significant motion is detected, the device wakes up, and when the device is placed flat, it enters sleep mode.
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

volatile bool gSigMotionFlag = false;
volatile bool gFlatFlag      = false;

enum DeviceState {
  STATE_SLEEP,
  STATE_AWAKE
};
DeviceState deviceState = STATE_SLEEP;

#if defined(ESP8266)
void IRAM_ATTR interruptSigMotion()
#else
void interruptSigMotion()
#endif
{
  gSigMotionFlag = true;
}

#if defined(ESP8266)
void IRAM_ATTR interruptFlat()
#else
void interruptFlat()
#endif
{
  gFlatFlag = true;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\nSmart Wake/Sleep Example");
  Serial.println("Pick up board to wake, place flat to sleep.\n");

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

  Serial.print("[3] Setting accelerometer range to ±8G... ");
  while (!imu.setAccelRange(DFRobot_Multi_DOF_IMU::eAccelRange8G)) {
    Serial.println("Failed, please check device communication!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[4] Configuring INT1 significant motion interrupt... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin1, DFRobot_Multi_DOF_IMU::eInt1_2SigMotion)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[5] Configuring INT2 flat detection interrupt... ");
  while (!imu.setInt(DFRobot_Multi_DOF_IMU::eImuIntPin2, DFRobot_Multi_DOF_IMU::eInt1_2Flat)) {
    Serial.println("Failed, please check pin and interrupt configuration!");
    delay(1000);
  }
  Serial.println("Success");
  delay(1000);

  Serial.print("[6] Configuring Arduino interrupt pins... ");

  // Configure INT1 interrupt pin (significant motion)
#if defined(ESP32)
  // D6 pin is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins.
  attachInterrupt(digitalPinToInterrupt(14 /*D6*/) /* Query the interrupt number of the D6 pin */, interruptSigMotion, RISING);
#elif defined(ESP8266)
#if defined(IMU_COMM_UART)
  const uint8_t interruptPin = 12;
#elif defined(IMU_COMM_I2C)
  const uint8_t interruptPin = 13;
#else
#error
#endif
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptSigMotion, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  // Pin 5 is used as interrupt pin by default, other non-conflicting pins can also be selected as external interrupt pins
  attachInterrupt(digitalPinToInterrupt(5) /* Query the interrupt number of the 5 pin */, interruptSigMotion, RISING);
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
  attachInterrupt(/*Interrupt No*/ 0, interruptSigMotion, RISING);    // Open the external interrupt 0, connect INT1 to the digital pin of the main control:
                                                                      // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif

  // Configure INT2 interrupt pin (flat detection)
#if defined(ESP32)
  attachInterrupt(digitalPinToInterrupt(13 /*D7*/), interruptFlat, RISING);
#elif defined(ESP8266)
  attachInterrupt(digitalPinToInterrupt(15), interruptFlat, RISING);
#elif defined(ARDUINO_SAM_ZERO)
  attachInterrupt(digitalPinToInterrupt(6), interruptFlat, RISING);
#else
  attachInterrupt(/*Interrupt No*/ 1, interruptFlat, RISING);
#endif
  Serial.println("Success");
  Serial.println("Trigger mode: Rising edge");
  delay(100);

  Serial.println("\nConfiguration complete, device initial state is sleep mode");
  Serial.println("Tip: Pick up board to wake device");
  Serial.println("     Place board flat to enter sleep mode\n");
  delay(100);
}

void loop()
{
  if (gSigMotionFlag) {
    gSigMotionFlag  = false;
    uint16_t status = imu.getIntStatus(DFRobot_Multi_DOF_IMU::eImuIntPin1);
    if (status & INT1_2_INT_STATUS_SIG_MOTION) {
      if (deviceState == STATE_SLEEP) {
        deviceState = STATE_AWAKE;
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.println(">>> Device Woke Up <<<");
      }
    }
  }

  if (gFlatFlag) {
    gFlatFlag       = false;
    uint16_t status = imu.getIntStatus(DFRobot_Multi_DOF_IMU::eImuIntPin2);
    if (status & INT1_2_INT_STATUS_FLAT) {
      if (deviceState == STATE_AWAKE) {
        deviceState = STATE_SLEEP;
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.println(">>> Device Entered Sleep Mode <<<");
      }
    }
  }

  if (deviceState == STATE_AWAKE) {
    // Can perform other tasks here, e.g., read sensor data, update display, etc.
  }
}
