# -*- coding: utf-8 -*
'''!
@file  imu_9dof_polling.py
@brief  Read 9DOF IMU data example (accelerometer + gyroscope + magnetometer)
@details  This example demonstrates how to read 9DOF IMU sensor data using I2C or UART interface.
@n  The example initializes the sensor, configures the accelerometer and gyroscope ranges,
@n  then continuously reads and prints the 9DOF data (acceleration, angular velocity, and magnetic field) in the loop function.
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
  @brief Create sensor object based on selected mode
'''
imu: DFRobot_Multi_DOF_IMU
if mode == "I2C":
  I2C_BUS = 0x01
  imu = DFRobot_Multi_DOF_IMU_I2C(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_9DOF, I2C_BUS, DEV_ADDR)
elif mode == "UART":
  imu = DFRobot_Multi_DOF_IMU_UART(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_9DOF, 9600, DEV_ADDR)


def setup():
  print("\n9DOF IMU Polling Example (%s mode)\n" % mode)

  '''!
      @brief Initialize sensor
      @n     Check device address and connections if initialization fails
  '''
  print("[1] Initializing sensor... ", end="")
  while not imu.begin():
    print("Failed, please check device address and connections!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set sensor mode to normal
      @n     Available modes:
      @n     - SLEEP_MODE:            Sleep mode (lowest power consumption)
      @n     - LOW_POWER_MODE:        Low power mode
      @n     - NORMAL_MODE:           Normal mode (recommended)
      @n     - HIGH_PERFORMANCE_MODE: High performance mode
  '''
  print("[2] Setting sensor mode to normal... ", end="")
  while not imu.set_sensor_mode(imu.NORMAL_MODE):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set accelerometer range to ±2G
      @n     Available ranges:
      @n     - ACCEL_RANGE_2G:  ±2g range
      @n     - ACCEL_RANGE_4G:  ±4g range
      @n     - ACCEL_RANGE_8G:  ±8g range
      @n     - ACCEL_RANGE_16G: ±16g range
  '''
  print("[3] Setting accelerometer range to ±2G... ", end="")
  while not imu.set_accel_range(imu.ACCEL_RANGE_2G):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Set gyroscope range to ±250dps
      @n     Available ranges:
      @n     - GYRO_RANGE_125DPS:  ±125dps range
      @n     - GYRO_RANGE_250DPS:  ±250dps range
      @n     - GYRO_RANGE_500DPS:  ±500dps range
      @n     - GYRO_RANGE_1000DPS: ±1000dps range
      @n     - GYRO_RANGE_2000DPS: ±2000dps range
  '''
  print("[4] Setting gyroscope range to ±250dps... ", end="")
  while not imu.set_gyro_range(imu.GYRO_RANGE_250DPS):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(0.1)

  print("\nConfiguration complete, starting data reading")
  print("AccX(g), AccY(g), AccZ(g), GyrX(dps), GyrY(dps), GyrZ(dps), MagX(uT), MagY(uT), MagZ(uT)\n")


def loop():
  '''!
  @brief Read 9DOF IMU data
  @n     Returns dictionary with accel, gyro, and mag data
  @n     accel: x, y, z in g
  @n     gyro: x, y, z in dps
  @n     mag: x, y, z in uT
  '''
  data = imu.get_9dof_data()

  if data is not None:
    print(
      "%.3f, %.3f, %.3f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f"
      % (data['accel']['x'], data['accel']['y'], data['accel']['z'], data['gyro']['x'], data['gyro']['y'], data['gyro']['z'], data['mag']['x'], data['mag']['y'], data['mag']['z'])
    )
  else:
    print("Failed to read data!")

  time.sleep(0.5)


if __name__ == "__main__":
  setup()
  while True:
    loop()
