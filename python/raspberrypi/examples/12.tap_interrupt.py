# -*- coding: utf-8 -*
'''!
@file  12.tap_interrupt.py
@brief  Tap detection interrupt example
@details  This example demonstrates how to configure and use tap detection interrupt.
@n  The example configures the sensor to generate an interrupt when a tap is detected.
@n  Tapping the sensor will trigger the interrupt and display the tap type (single, double, or triple).
@n  Connect the sensor's INT1 pin to GPIO 5.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author      [Martin](Martin@dfrobot.com)
@version     V1.0.0
@date        2026-01-16
@url         https://github.com/DFRobot/DFRobot_Multi_DOF_IMU
'''

import os
import sys
import time
import RPi.GPIO as GPIO

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_Multi_DOF_IMU import *

DEV_ADDR = 0x4A

'''!
  @brief Select communication mode
  @n     "I2C" - Use I2C interface (default)
  @n     "UART" - Use UART interface (Modbus RTU protocol)
'''
# mode = "UART"
mode = "I2C"

'''!
  @brief GPIO pin for interrupt (BCM mode)
'''
GPIO_INT1 = 5

'''!
  @brief Create sensor object based on selected mode
'''
imu: DFRobot_Multi_DOF_IMU
if mode == "I2C":
  I2C_BUS = 0x01
  imu = DFRobot_Multi_DOF_IMU_I2C(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_6DOF, I2C_BUS, DEV_ADDR)
elif mode == "UART":
  imu = DFRobot_Multi_DOF_IMU_UART(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_6DOF, 9600, DEV_ADDR)

'''!
  @brief Interrupt flag
'''
tap_detected = False


def int1_isr(channel):
  '''!
  @brief INT1 interrupt service routine
  '''
  global tap_detected
  tap_detected = True


def setup():
  print("\nTap Detection Interrupt Example (%s mode)\n" % mode)

  '''!
      @brief Initialize sensor
  '''
  print("[1] Initializing sensor... ", end="")
  while not imu.begin():
    print("Failed, please check device address and connections!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set sensor mode to normal
  '''
  print("[2] Setting sensor mode to normal... ", end="")
  while not imu.set_sensor_mode(imu.NORMAL_MODE):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set accelerometer range to ±2G
  '''
  print("[3] Setting accelerometer range to ±2G... ", end="")
  while not imu.set_accel_range(imu.ACCEL_RANGE_2G):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure INT1 tap detection interrupt
  '''
  print("[4] Configuring INT1 tap detection interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_TAP):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure Raspberry Pi GPIO interrupt
  '''
  print("[5] Configuring Raspberry Pi GPIO interrupt... ", end="")
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(GPIO_INT1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.add_event_detect(GPIO_INT1, GPIO.RISING, callback=int1_isr)
  print("Success")
  print("    GPIO pin: %d (BCM mode)" % GPIO_INT1)
  print("    Trigger mode: Rising edge")
  time.sleep(0.1)

  print("\nConfiguration complete, starting tap monitoring")
  print("Tip: Tap the sensor to trigger interrupt")
  print("     (Supports single, double, triple tap detection)\n")


def loop():
  '''!
  @brief Main loop - detect taps
  '''
  global tap_detected

  if tap_detected:
    tap_detected = False

    tap_data = imu.get_tap()

    if tap_data == imu.TAP_TYPE_SINGLE:
      print("Single tap (Data: 0x%04X)" % tap_data)
    elif tap_data == imu.TAP_TYPE_DOUBLE:
      print("Double tap (Data: 0x%04X)" % tap_data)
    elif tap_data == imu.TAP_TYPE_TRIPLE:
      print("Triple tap (Data: 0x%04X)" % tap_data)
    else:
      print("Unknown tap (Data: 0x%04X)" % tap_data)


if __name__ == "__main__":
  try:
    setup()
    while True:
      loop()
      time.sleep(0.2)
  except KeyboardInterrupt:
    print("\nExit.")
  finally:
    GPIO.cleanup()
