# -*- coding: utf-8 -*
'''!
@file  interrupt_oor.py
@brief  Pressure out-of-range (OOR) interrupt example
@details  This example demonstrates how to configure and use 10DOF sensor pressure OOR interrupt.
@n  The example sets a pressure threshold and allowed range, then configures the sensor to generate an interrupt
@n  when the pressure goes out of the specified range. It continuously monitors pressure and displays the status.
@n  Connect the sensor's INT4 pin to GPIO 19.
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
  @brief GPIO pin for INT4 interrupt (BCM mode)
'''
GPIO_INT4 = 19

'''!
  @brief OOR configuration
  @n     PRESS_THRESHOLD: Pressure threshold (Pa)
  @n     PRESS_RANGE: Allowed range (Pa), valid range is THRESHOLD ± RANGE
'''
PRESS_THRESHOLD = 94658
PRESS_RANGE = 50

'''!
  @brief Create sensor object based on selected mode
'''
imu: DFRobot_Multi_DOF_IMU
if mode == "I2C":
  I2C_BUS = 0x01
  imu = DFRobot_Multi_DOF_IMU_I2C(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_10DOF, I2C_BUS, DEV_ADDR)
elif mode == "UART":
  imu = DFRobot_Multi_DOF_IMU_UART(DFRobot_Multi_DOF_IMU.SENSOR_MODEL_10DOF, 9600, DEV_ADDR)

'''!
  @brief Interrupt flag
'''
oor_interrupt = False


def int4_isr(channel):
  global oor_interrupt
  oor_interrupt = True


def setup():
  print("\nPressure Out-of-Range (OOR) Interrupt Example (%s mode)\n" % mode)

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
      @brief Configure OOR threshold and range
  '''
  print("[2] Configuring OOR threshold and range... ", end="")
  if imu.set_press_oor(PRESS_THRESHOLD, PRESS_RANGE, imu.PRESS_OOR_COUNT_LIMIT_1):
    print("Success")
    print("    Pressure threshold: %d Pa" % PRESS_THRESHOLD)
    print("    Allowed range: ±%d Pa" % PRESS_RANGE)
    print("    Valid range: %d ~ %d Pa" % (PRESS_THRESHOLD - PRESS_RANGE, PRESS_THRESHOLD + PRESS_RANGE))
    print("    Out-of-range will trigger interrupt")
    print("    Count limit: 1 time")
  else:
    print("Failed")
    print("Failed, please check device communication!")
    return False
  time.sleep(1)

  '''!
      @brief Configure INT4 OOR interrupt
  '''
  print("[3] Configuring INT4 OOR interrupt... ", end="")
  while not imu.set_int(imu.IMU_INT_PIN_INT4, imu.INT4_PRESSURE_OOR):
    print("Failed, please check pin and interrupt configuration!")
    time.sleep(1)
  print("Success")
  time.sleep(1)

  '''!
      @brief Configure Raspberry Pi GPIO interrupt
  '''
  print("[4] Configuring interrupt pin... ", end="")
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(GPIO_INT4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
  GPIO.add_event_detect(GPIO_INT4, GPIO.RISING, callback=int4_isr)
  print("Success")
  print("    GPIO pin: %d (BCM mode)" % GPIO_INT4)
  print("    Trigger mode: Rising edge")
  time.sleep(1)

  '''!
      @brief Set sensor mode to normal
  '''
  print("[5] Setting sensor mode to normal... ", end="")
  while not imu.set_sensor_mode(imu.NORMAL_MODE):
    print("Failed, please check device communication!")
    time.sleep(1)
  print("Success")
  time.sleep(0.1)

  print("\nConfiguration complete, waiting for OOR interrupt")
  print("Tip: Allowed range: %d ~ %d Pa" % (PRESS_THRESHOLD - PRESS_RANGE, PRESS_THRESHOLD + PRESS_RANGE))
  print("     OOR interrupt triggers when pressure exceeds this range")
  print("     Change altitude or pressure environment to test")
  print("Pressure(Pa), Deviation(Pa), Status, Interrupt Status\n")

  return True


def loop():
  '''!
  @brief Main loop - check for OOR interrupt
  '''
  global oor_interrupt

  if oor_interrupt:
    oor_interrupt = False

    int_status = imu.get_int_status(imu.IMU_INT_PIN_INT4)

    if int_status & imu.INT4_INT_STATUS_OOR:
      data = imu.get_10dof_data()

      if data is not None:
        pressure = data['pressure']
        deviation = pressure - PRESS_THRESHOLD
        in_range = (pressure >= PRESS_THRESHOLD - PRESS_RANGE) and (pressure <= PRESS_THRESHOLD + PRESS_RANGE)

        print("%.2f, %s%.2f, %s, 0x%04X" % (pressure, "+" if deviation > 0 else "", deviation, "In Range" if in_range else "Out of Range", int_status))
      else:
        print("Failed to read pressure data!")

  time.sleep(0.01)


if __name__ == "__main__":
  try:
    if setup():
      while True:
        loop()
  except KeyboardInterrupt:
    print("\nExit.")
  finally:
    GPIO.cleanup()
