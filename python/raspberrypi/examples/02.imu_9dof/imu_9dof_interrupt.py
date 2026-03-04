# -*- coding: utf-8 -*
'''!
@file  imu_9dof_interrupt.py
@brief  9DOF IMU data ready interrupt example
@details  This example demonstrates how to read 9DOF IMU data using interrupt mode.
@n  The example configures the sensor to generate interrupts when data is ready from both 6DOF and 9DOF sensors,
@n  then reads the 9DOF data (acceleration, angular velocity, and magnetic field) only when interrupts are triggered.
@n  Connect the sensor's INT1 pin to GPIO 5, INT3 pin to GPIO 6.
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
  @brief GPIO pins for interrupt (BCM mode)
  @n     Connect the sensor's INT1 pin to GPIO_INT1
  @n     Connect the sensor's INT3 pin to GPIO_INT3
'''
GPIO_INT1 = 5
GPIO_INT3 = 13

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
  @brief Interrupt flags
'''
int1_data_ready = False
int3_data_ready = False


def int1_isr(channel):
  '''!
  @brief INT1 interrupt service routine (6DOF data ready)
  @param channel GPIO channel that triggered the interrupt
  '''
  global int1_data_ready
  int1_data_ready = True


def int3_isr(channel):
  '''!
  @brief INT3 interrupt service routine (magnetometer data ready)
  @param channel GPIO channel that triggered the interrupt
  '''
  global int3_data_ready
  int3_data_ready = True


def setup():
  print("\n9DOF IMU Data Ready Interrupt Example (%s mode)\n" % mode)

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
      @brief Set gyroscope range to ±250dps
  '''
  print("[4] Setting gyroscope range to ±250dps... ", end="")
  while not imu.set_gyro_range(imu.GYRO_RANGE_250DPS):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure INT1 data ready interrupt (6DOF sensor)
  '''
  print("[5] Configuring INT1 data ready interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT1, imu.INT1_2_DATA_READY):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure INT3 data ready interrupt (magnetometer)
  '''
  print("[6] Configuring INT3 data ready interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT3, imu.INT3_DATA_READY):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure Raspberry Pi GPIO interrupts
  '''
  print("[7] Configuring Raspberry Pi GPIO interrupts... ", end="")
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(GPIO_INT1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.setup(GPIO_INT3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.add_event_detect(GPIO_INT1, GPIO.RISING, callback=int1_isr)
  GPIO.add_event_detect(GPIO_INT3, GPIO.RISING, callback=int3_isr)
  print("Success")
  print("    INT1 GPIO pin: %d (BCM mode)" % GPIO_INT1)
  print("    INT3 GPIO pin: %d (BCM mode)" % GPIO_INT3)
  print("    Trigger mode: Rising edge")

  print("\nConfiguration complete, starting data reading")
  print("AccX(g), AccY(g), AccZ(g), GyrX(dps), GyrY(dps), GyrZ(dps), MagX(uT), MagY(uT), MagZ(uT)\n")


def loop():
  '''!
  @brief Main loop - read data when both interrupts are triggered
  '''
  global int1_data_ready, int3_data_ready

  if int1_data_ready or int3_data_ready:
    '''!
          @brief Check interrupt status for both INT1 and INT3
    '''
    int1_status = imu.get_int_status(imu.IMU_INT_PIN_INT1)
    int3_status = imu.get_int_status(imu.IMU_INT_PIN_INT3)

    int1_is_drdy = (int1_status & imu.INT1_2_INT_STATUS_DRDY) != 0
    int3_is_drdy = (int3_status & imu.INT3_INT_STATUS_DRDY) != 0

    if int1_is_drdy and int3_is_drdy:
      int1_data_ready = False
      int3_data_ready = False

      '''!
              @brief Read 9DOF IMU data
      '''
      data = imu.get_9dof_data()

      if data is not None:
        print(
          "%.3f, %.3f, %.3f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f"
          % (data['accel']['x'], data['accel']['y'], data['accel']['z'], data['gyro']['x'], data['gyro']['y'], data['gyro']['z'], data['mag']['x'], data['mag']['y'], data['mag']['z'])
        )
      else:
        print("Failed to read 9DOF data!")
    else:
      if int1_data_ready and not int1_is_drdy:
        int1_data_ready = False
      if int3_data_ready and not int3_is_drdy:
        int3_data_ready = False


if __name__ == "__main__":
  try:
    setup()
    while True:
      loop()
  except KeyboardInterrupt:
    print("\nExit.")
  finally:
    GPIO.cleanup()
