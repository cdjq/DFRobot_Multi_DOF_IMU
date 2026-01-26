# -*- coding: utf-8 -*
'''!
@file  compass_step_counter.py
@brief  Compass + step counter application example
@details  This example demonstrates how to use 9DOF IMU sensor to implement compass + step counter functionality.
@n  The example configures the step counter interrupt and periodically reads the magnetometer data to calculate heading.
@n  It outputs the direction name, heading angle, and step count at regular intervals.
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
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
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
  @brief Create sensor object based on selected mode
'''
imu: DFRobot_Multi_DOF_IMU
if mode == "I2C":
  I2C_BUS = 0x01
  imu = DFRobot_Multi_DOF_IMU_I2C(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_9DOF, I2C_BUS, DEV_ADDR)
elif mode == "UART":
  imu = DFRobot_Multi_DOF_IMU_UART(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_9DOF, 9600, DEV_ADDR)

'''!
  @brief Direction names for compass
'''
DIRECTION_NAMES = ["North", "Northeast", "East", "Southeast", "South", "Southwest", "West", "Northwest"]


def calculate_heading(mag_x, mag_y):
  '''!
  @fn calculate_heading
  @brief Calculate heading based on magnetometer data
  @param mag_x Magnetometer X-axis data (unit: uT, typically points east)
  @param mag_y Magnetometer Y-axis data (unit: uT, typically points north)
  @return float Heading angle (0-360 degrees, 0°=North, 90°=East, 180°=South, 270°=West)
  '''
  heading = math.atan2(mag_x, mag_y)
  heading = heading * 180.0 / math.pi

  if heading < 0:
    heading += 360.0

  # Magnetic declination (degrees) - adjust for your location
  declination = 0.0
  heading += declination

  if heading >= 360.0:
    heading -= 360.0
  elif heading < 0:
    heading += 360.0

  return heading


def get_direction_name(heading):
  '''!
  @fn get_direction_name
  @brief Get direction name based on heading angle
  @param heading Heading angle (0-360 degrees)
  @return str Direction name string
  '''
  index = int((heading + 22.5) / 45.0) % 8
  return DIRECTION_NAMES[index]


def setup():
  print("\nCompass + Step Counter Example (%s mode)" % mode)
  print("Features: Compass (magnetometer) + Step counter")
  print("Usage: Place sensor horizontally, start walking")
  print("Output: Direction | Heading(°) | Steps\n")

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
      @brief Configure step counter (INT1)
  '''
  print("[4] Configuring step counter (INT1)... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_STEP_COUNTER):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(0.1)

  print("\nConfiguration complete, starting monitoring")
  print("Format: Direction | Heading(°) | Steps\n")


def loop():
  '''!
  @brief Main loop - read compass data and step count every second
  '''
  data = imu.get_9dof_data()

  if data is not None:
    heading = calculate_heading(data['mag']['x'], data['mag']['y'])
    direction = get_direction_name(heading)
    step_count = imu.get_step_count()

    print("Direction: %s (%.1f°) | Steps: %d" % (direction, heading, step_count))
  else:
    print("Failed to read sensor data!")

  time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
