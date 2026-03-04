# -*- coding: utf-8 -*
'''!
@file  05.smart_wake_sleep.py
@brief  Smart wake/sleep example - combining significant motion and flat detection
@details  This example demonstrates how to use both significant motion and flat detection to implement smart wake/sleep functionality.
@n  The example configures INT1 for significant motion detection (wake) and INT2 for flat detection (sleep).
@n  When significant motion is detected, the device wakes up, and when the device is placed flat, it enters sleep mode.
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
  @brief Device state enumeration
'''
STATE_SLEEP = 0
STATE_AWAKE = 1

'''!
  @brief Interrupt flags and current state
'''
sig_motion_flag = False
flat_flag = False
device_state = STATE_SLEEP


def int1_sig_motion_isr(channel):
  '''!
  @brief INT1 interrupt service routine (significant motion)
  '''
  global sig_motion_flag
  sig_motion_flag = True


def int2_flat_isr(channel):
  '''!
  @brief INT2 interrupt service routine (flat detection)
  '''
  global flat_flag
  flat_flag = True


def setup():
  print("\nSmart Wake/Sleep Example (%s mode)" % mode)
  print("Pick up board to wake, place flat to sleep.\n")

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
      @brief Configure INT1 significant motion interrupt
  '''
  print("[4] Configuring INT1 significant motion interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_SIG_MOTION):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure INT2 flat detection interrupt
  '''
  print("[5] Configuring INT2 flat detection interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT2, imu.INT1_2_FLAT):
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
  GPIO.add_event_detect(GPIO_INT1, GPIO.RISING, callback=int1_sig_motion_isr)
  GPIO.add_event_detect(GPIO_INT2, GPIO.RISING, callback=int2_flat_isr)
  print("Success")
  print("    INT1 GPIO pin: %d (BCM mode) - Significant motion (wake)" % GPIO_INT1)
  print("    INT2 GPIO pin: %d (BCM mode) - Flat detection (sleep)" % GPIO_INT2)
  print("    Trigger mode: Rising edge")
  time.sleep(0.1)

  print("\nConfiguration complete, device initial state is sleep mode")
  print("Tip: Pick up board to wake device")
  print("     Place board flat to enter sleep mode\n")


def loop():
  '''!
  @brief Main loop - detect wake/sleep state changes
  '''
  global sig_motion_flag, flat_flag, device_state

  if sig_motion_flag:
    sig_motion_flag = False
    status = imu.get_int_status(imu.IMU_INT_PIN_INT1)
    if status & imu.INT1_2_INT_STATUS_SIG_MOTION:
      if device_state == STATE_SLEEP:
        device_state = STATE_AWAKE
        print("[%.3f] >>> Device Woke Up <<<" % time.time())

  if flat_flag:
    flat_flag = False
    status = imu.get_int_status(imu.IMU_INT_PIN_INT2)
    if status & imu.INT1_2_INT_STATUS_FLAT:
      if device_state == STATE_AWAKE:
        device_state = STATE_SLEEP
        print("[%.3f] >>> Device Entered Sleep Mode <<<" % time.time())

  if device_state == STATE_AWAKE:
    # Can perform other tasks here, e.g., read sensor data, update display, etc.
    pass


if __name__ == "__main__":
  try:
    setup()
    while True:
      loop()
  except KeyboardInterrupt:
    print("\nExit.")
  finally:
    GPIO.cleanup()
