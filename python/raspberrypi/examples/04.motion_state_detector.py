# -*- coding: utf-8 -*
'''!
@file  04.motion_state_detector.py
@brief  Motion state detection example - combining any motion and no motion detection
@details  This example demonstrates how to use both any motion and no motion detection to determine device motion state.
@n  The example configures INT1 for any motion detection and INT2 for no motion detection.
@n  When any motion is detected, the device state changes to "Moving", and when no motion is detected, it changes to "Still".
@n  Connect the sensor's INT1 pin to GPIO 5, INT2 pin to GPIO 6.
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
  @brief GPIO pins for interrupt (BCM mode)
'''
GPIO_INT1 = 5
GPIO_INT2 = 6

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
  @brief Motion state enumeration
'''
STATE_UNKNOWN = 0
STATE_MOVING = 1
STATE_STILL = 2

'''!
  @brief Interrupt flags and current state
'''
any_motion_flag = False
no_motion_flag = False
current_state = STATE_UNKNOWN


def int1_any_motion_isr(channel):
  '''!
  @brief INT1 interrupt service routine (any motion)
  '''
  global any_motion_flag
  any_motion_flag = True


def int2_no_motion_isr(channel):
  '''!
  @brief INT2 interrupt service routine (no motion)
  '''
  global no_motion_flag
  no_motion_flag = True


def setup():
  print("\nMotion State Detection Example (%s mode)" % mode)
  print("Move the board to see state changes.\n")

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
      @brief Set accelerometer range to ±8G
  '''
  print("[3] Setting accelerometer range to ±8G... ", end="")
  while not imu.set_accel_range(imu.ACCEL_RANGE_8G):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure INT1 any motion interrupt
  '''
  print("[4] Configuring INT1 any motion interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_ANY_MOTION):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure INT2 no motion interrupt
  '''
  print("[5] Configuring INT2 no motion interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT2, imu.INT1_2_NO_MOTION):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure Raspberry Pi GPIO interrupts
  '''
  print("[6] Configuring Raspberry Pi GPIO interrupts... ", end="")
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(GPIO_INT1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.setup(GPIO_INT2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.add_event_detect(GPIO_INT1, GPIO.RISING, callback=int1_any_motion_isr)
  GPIO.add_event_detect(GPIO_INT2, GPIO.RISING, callback=int2_no_motion_isr)
  print("Success")
  print("    INT1 GPIO pin: %d (BCM mode) - Any motion" % GPIO_INT1)
  print("    INT2 GPIO pin: %d (BCM mode) - No motion" % GPIO_INT2)
  print("    Trigger mode: Rising edge")
  time.sleep(0.1)

  print("\nConfiguration complete, ready to detect motion state")
  print("Tip: Move the board to trigger any motion interrupt")
  print("     Keep still to trigger no motion interrupt\n")


def loop():
  '''!
  @brief Main loop - detect motion state changes
  '''
  global any_motion_flag, no_motion_flag, current_state

  if any_motion_flag:
    any_motion_flag = False
    if current_state != STATE_MOVING:
      current_state = STATE_MOVING
      print("[%.3f] I'm moving" % time.time())

  if no_motion_flag:
    no_motion_flag = False
    if current_state != STATE_STILL:
      current_state = STATE_STILL
      print("[%.3f] I've stopped" % time.time())


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
