# -*- coding: utf-8 -*
'''!
@file  11.orientation_interrupt.py
@brief  Orientation detection interrupt example
@details  This example demonstrates how to configure and use orientation detection interrupt.
@n  The example configures the sensor to generate an interrupt when the device orientation changes.
@n  Rotating the sensor will trigger the interrupt and display the screen orientation (portrait/landscape)
@n  and face direction (up/down) information.
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
  @brief Interrupt flag and orientation change count
'''
orientation_detected = False
orientation_change_count = 0


def int1_isr(channel):
  '''!
  @brief INT1 interrupt service routine
  '''
  global orientation_detected
  orientation_detected = True


def get_orientation_string(orientation):
  '''!
  @fn get_orientation_string
  @brief Get orientation string
  @param orientation Orientation value
  @return Orientation description string
  '''
  if orientation == imu.ORIENT_TYPE_PORTRAIT_UP:
    return "Portrait Upright"
  elif orientation == imu.ORIENT_TYPE_LANDSCAPE_LEFT:
    return "Landscape Left"
  elif orientation == imu.ORIENT_TYPE_LANDSCAPE_RIGHT:
    return "Landscape Right"
  elif orientation == imu.ORIENT_TYPE_PORTRAIT_DOWN:
    return "Portrait Upside Down"
  else:
    return "Unknown"


def get_face_string(face):
  '''!
  @fn get_face_string
  @brief Get face direction string
  @param face Face direction value (0-1)
  @return Face direction description string
  '''
  return "Face Up" if face == imu.ORIENT_FACE_UP else "Face Down"


def setup():
  print("\nOrientation Detection Interrupt Example (%s mode)\n" % mode)

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
      @brief Configure INT1 orientation detection interrupt
  '''
  print("[4] Configuring INT1 orientation detection interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_ORIENTATION):
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

  print("\nConfiguration complete, starting orientation monitoring")
  print("Tip: Rotate sensor to trigger orientation change interrupt")
  print("     (Supports 4 orientations: Portrait up/down, Landscape left/right)")
  print("     (Supports 2 face directions: Face up/down)\n")


def loop():
  '''!
  @brief Main loop - detect orientation changes
  '''
  global orientation_detected, orientation_change_count

  if orientation_detected:
    orientation_detected = False

    orient_data = imu.get_orientation()

    orientation = (orient_data >> 8) & 0xFF  # High byte: orientation type
    face_up_down = orient_data & 0xFF  # Low byte: face direction

    orientation_change_count += 1

    print("")
    print("Orientation change #%d" % orientation_change_count)
    print("   Screen orientation: %s" % get_orientation_string(orientation))
    print("   Face direction: %s" % get_face_string(face_up_down))
    print("   Raw data: 0x%04X (Time: %.3fs)" % (orient_data, time.time()))


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
