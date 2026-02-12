# -*- coding: utf-8 -*
'''!
@file  imu_10dof_polling.py
@brief  Read 10DOF IMU data example (accelerometer + gyroscope + magnetometer + barometer)
@details  This example demonstrates how to read 10DOF IMU sensor data using I2C/UART interface.
@n  The example initializes the sensor, configures the accelerometer and gyroscope ranges,
@n  optionally calibrates altitude data based on local altitude,
@n  then continuously reads and prints the 10DOF data (acceleration, angular velocity, magnetic field, and pressure) in the loop function.
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
  @brief Altitude calibration (optional)
  @n     Set to True to calibrate altitude based on local altitude
'''
CALIBRATE_ABSOLUTE_DIFFERENCE = True
CALCULATE_ALTITUDE = True if CALIBRATE_ABSOLUTE_DIFFERENCE else False

'''!
  @brief Create sensor object based on selected mode
'''
imu: DFRobot_Multi_DOF_IMU
if mode == "I2C":
  I2C_BUS = 0x01
  imu = DFRobot_Multi_DOF_IMU_I2C(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_10DOF, I2C_BUS, DEV_ADDR)
elif mode == "UART":
  imu = DFRobot_Multi_DOF_IMU_UART(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_10DOF, 9600, DEV_ADDR)


def setup():
  print("\n10DOF IMU Data Reading Example (%s mode)\n" % mode)

  '''!
      @brief Initialize sensor
  '''
  print("[Init] Initializing sensor... ", end="")
  while not imu.begin():
    print("Failed, please check device address and connections!")
    time.sleep(1)
  print("Success!")
  time.sleep(1)

  '''!
      @brief Set sensor mode to normal
  '''
  print("[Config] Setting sensor mode to normal... ", end="")
  while not imu.set_sensor_mode(imu.NORMAL_MODE):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set accelerometer range to ±2G
  '''
  print("[Config] Setting accelerometer range to ±2g... ", end="")
  while not imu.set_accel_range(imu.ACCEL_RANGE_2G):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set gyroscope range to ±250dps
  '''
  print("[Config] Setting gyroscope range to ±250dps... ", end="")
  while not imu.set_gyro_range(imu.GYRO_RANGE_250DPS):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Calibrate altitude (optional)
      @n     Calibrate based on local altitude (540m in this example)
  '''
  if CALIBRATE_ABSOLUTE_DIFFERENCE:
    print("[Config] Calibrating altitude (reference 540m)... ", end="")
    imu.calibrate_altitude(540.0)
    print("Done")

  time.sleep(0.1)
  print("\nConfiguration complete, starting data reading...\n")


def loop():
  '''!
  @brief Read 10DOF IMU data
  @n     Returns dictionary with accel, gyro, mag data and pressure value
  '''
  data = imu.get_10dof_data(calc_altitude=CALCULATE_ALTITUDE)

  if data is not None:
    print("10DOF IMU Data")
    print("Acc: X=%.3fg Y=%.3fg Z=%.3fg" % (data['accel']['x'], data['accel']['y'], data['accel']['z']))
    print("Gyr: X=%.2fdps Y=%.2fdps Z=%.2fdps" % (data['gyro']['x'], data['gyro']['y'], data['gyro']['z']))
    print("Mag: X=%.2fuT Y=%.2fuT Z=%.2fuT" % (data['mag']['x'], data['mag']['y'], data['mag']['z']))
    if CALCULATE_ALTITUDE:
      print("Altitude: %.2f m\n" % data['pressure'])
    else:
      print("Pressure: %.2f Pa (%.2f hPa)\n" % (data['pressure'], data['pressure'] / 100.0))
  else:
    print("Error: Failed to read sensor data!\n")

  time.sleep(0.5)


if __name__ == "__main__":
  setup()
  while True:
    loop()
