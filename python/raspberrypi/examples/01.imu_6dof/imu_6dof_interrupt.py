# -*- coding: utf-8 -*
'''!
@file  imu_6dof_interrupt.py
@brief  6DOF IMU data ready interrupt example
@details  This example demonstrates how to read 6DOF IMU data using interrupt mode.
@n  The example configures the sensor to generate an interrupt when data is ready,
@n  then reads the 6DOF data (acceleration and angular velocity) only when the interrupt is triggered.
@n  Connect the sensor's INT1 pin to Raspberry Pi GPIO pin (default: GPIO 25).
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
  @brief GPIO pin for interrupt (BCM mode)
  @n     Connect the sensor's INT1 pin to this GPIO pin
'''
GPIO_INT = 5

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
data_ready = False


def int1_isr(channel):
  '''!
  @brief INT1 interrupt service routine
  @param channel GPIO channel that triggered the interrupt
  '''
  global data_ready
  data_ready = True


def setup():
  global data_ready

  print("\n6DOF IMU Data Ready Interrupt Example (%s mode)\n" % mode)

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
  time.sleep(1)

  '''!
      @brief Configure INT1 data ready interrupt
      @n     INT1_2_DATA_READY: Generate interrupt when new data is available
  '''
  print("[5] Configuring INT1 data ready interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_DATA_READY):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure Raspberry Pi GPIO interrupt
      @n     Set GPIO pin as input with pull-down resistor
      @n     Trigger on rising edge (INT1 is active high)
  '''
  print("[6] Configuring Raspberry Pi GPIO interrupt... ", end="")
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(GPIO_INT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.add_event_detect(GPIO_INT, GPIO.RISING, callback=int1_isr)
  print("Success")
  print("    GPIO pin: %d (BCM mode)" % GPIO_INT)
  print("    Trigger mode: Rising edge")

  print("\nConfiguration complete, starting data reading")
  print("AccX(g), AccY(g), AccZ(g), GyrX(dps), GyrY(dps), GyrZ(dps)\n")


def loop():
  '''!
  @brief Main loop - read data when interrupt is triggered
  @n     Only reads data when data_ready flag is set by ISR
  '''
  global data_ready

  if data_ready:
    data_ready = False

    '''!
        @brief Check interrupt status
        @n     INT1_2_INT_STATUS_DRDY indicates data ready interrupt occurred
    '''
    int_status = imu.get_int_status(imu.IMU_INT_PIN_INT1)

    if int_status & imu.INT1_2_INT_STATUS_DRDY:
      '''!
              @brief Read 6DOF IMU data
              @n     Returns dictionary with accel and gyro data
              @n     accel: x, y, z in g
              @n     gyro: x, y, z in dps
      '''
      data = imu.get_6dof_data()

      if data is not None:
        print("%.3f, %.3f, %.3f, %.2f, %.2f, %.2f" % (data['accel']['x'], data['accel']['y'], data['accel']['z'], data['gyro']['x'], data['gyro']['y'], data['gyro']['z']))
      else:
        print("Failed to read data!")

  time.sleep(0.01)


if __name__ == "__main__":
  try:
    setup()
    while True:
      loop()
  except KeyboardInterrupt:
    print("\nExit.")
  finally:
    GPIO.cleanup()
