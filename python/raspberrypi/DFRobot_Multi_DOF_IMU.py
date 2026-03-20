# -*- coding: utf-8 -*
'''!
@file  DFRobot_Multi_DOF_IMU.py
@brief  Define the infrastructure of DFRobot_Multi_DOF_IMU class
@n      This is a multi-axis IMU sensor that can be controlled through I2C/UART ports.
@n      It supports 6DOF (accelerometer + gyroscope), 9DOF (+ magnetometer), and 10DOF (+ barometer) sensors.
@copyright   Copyright (c) 2026 DFRobot Co.Ltd (http://www.dfrobot.com)
@license     The MIT License (MIT)
@author      [Martin](Martin@dfrobot.com)
@version     V1.0.0
@date        2026-01-16
@url         https://github.com/DFRobot/DFRobot_Multi_DOF_IMU
'''

import time
import math
from DFRobot_RTU import *


class DFRobot_Multi_DOF_IMU(object):
  '''!
  @brief DFRobot_Multi_DOF_IMU base class
  @details Provides multi-axis IMU sensor data reading functionality, supporting 6DOF (accelerometer+gyroscope), 9DOF (+magnetometer), and 10DOF (+barometer) sensors
  '''

  # Sensor model enumeration
  SENSOR_MODEL_6DOF = 0x0006  # 6DOF IMU (accelerometer + gyroscope)
  SENSOR_MODEL_9DOF = 0x0009  # 9DOF IMU (accelerometer + gyroscope + magnetometer)
  SENSOR_MODEL_10DOF = 0x0010  # 10DOF IMU (accelerometer + gyroscope + magnetometer + barometer)

  # Communication mode enumeration
  COMM_MODE_UART = 0  # UART communication mode (Modbus RTU)
  COMM_MODE_I2C = 1  # I2C communication mode

  # Register type enumeration (for UART Modbus mode)
  # In UART mode, input registers (function code 0x04) and holding registers (function code 0x03)
  # have overlapping address ranges, so this enum is needed to distinguish them.
  # In I2C mode, this parameter is ignored as there's only one register space.
  REG_TYPE_INPUT = 0  # Input register (read-only, Modbus function code 0x04)
  REG_TYPE_HOLDING = 1  # Holding register (read-write, Modbus function code 0x03/0x06)

  # Return codes
  RET_CODE_OK = 0  # Return code: success
  RET_CODE_ERROR = 1  # Return code: failure

  # Accelerometer range enumeration
  ACCEL_RANGE_2G = 0  # ±2g range
  ACCEL_RANGE_4G = 1  # ±4g range
  ACCEL_RANGE_8G = 2  # ±8g range
  ACCEL_RANGE_16G = 3  # ±16g range

  # Gyroscope range enumeration
  GYRO_RANGE_125DPS = 0  # ±125dps range
  GYRO_RANGE_250DPS = 1  # ±250dps range
  GYRO_RANGE_500DPS = 2  # ±500dps range
  GYRO_RANGE_1000DPS = 3  # ±1000dps range
  GYRO_RANGE_2000DPS = 4  # ±2000dps range

  # Sensor operating mode enumeration
  SLEEP_MODE = 0x00  # Sleep mode: lowest power consumption, sensor stops working
  LOW_POWER_MODE = 0x01  # Low power mode: reduced sampling rate, saves power
  NORMAL_MODE = 0x02  # Normal mode: balances power consumption and performance
  HIGH_PERFORMANCE_MODE = 0x03  # High performance mode: highest sampling rate and accuracy

  # IMU interrupt pin enumeration
  IMU_INT_PIN_NONE = 0x00  # Not mapped to any pin
  IMU_INT_PIN_INT1 = 0x01  # INT1 pin (6DOF sensor)
  IMU_INT_PIN_INT2 = 0x02  # INT2 pin (6DOF sensor)
  IMU_INT_PIN_INT3 = 0x03  # INT3 pin (9DOF sensor-magnetometer)
  IMU_INT_PIN_INT4 = 0x04  # INT4 pin (10DOF sensor-barometer)

  # INT1/INT2 interrupt types (0x00-0x09)
  INT1_2_DISABLE = 0x00  # Interrupt disabled (INT1/INT2)
  INT1_2_DATA_READY = 0x01  # Data ready interrupt (INT1/INT2)
  INT1_2_ANY_MOTION = 0x02  # Any motion interrupt (INT1/INT2)
  INT1_2_NO_MOTION = 0x03  # No motion interrupt (INT1/INT2)
  INT1_2_SIG_MOTION = 0x04  # Significant motion interrupt (INT1/INT2)
  INT1_2_STEP_COUNTER = 0x05  # Step counter interrupt (INT1/INT2)
  INT1_2_FLAT = 0x06  # Flat interrupt (INT1/INT2)
  INT1_2_ORIENTATION = 0x07  # Orientation interrupt (INT1/INT2)
  INT1_2_TAP = 0x08  # Tap interrupt (INT1/INT2)
  INT1_2_TILT = 0x09  # Tilt interrupt (INT1/INT2)

  # INT3 interrupt types (0x10-0x11)
  INT3_DISABLE = 0x10  # Interrupt disabled (INT3)
  INT3_DATA_READY = 0x11  # Data ready interrupt (INT3)

  # INT4 interrupt types (0x20-0x22)
  INT4_DISABLE = 0x20  # Interrupt disabled (INT4)
  INT4_DATA_READY = 0x21  # Data ready interrupt (INT4)
  INT4_PRESSURE_OOR = 0x22  # Pressure out-of-range interrupt (INT4)

  # IMU tap type return values
  TAP_TYPE_SINGLE = 0x0001  # Single tap interrupt
  TAP_TYPE_DOUBLE = 0x0002  # Double tap interrupt
  TAP_TYPE_TRIPLE = 0x0003  # Triple tap interrupt

  # IMU orientation type (high byte)
  ORIENT_TYPE_PORTRAIT_UP = 0x01  # Portrait upright
  ORIENT_TYPE_LANDSCAPE_LEFT = 0x02  # Landscape left
  ORIENT_TYPE_LANDSCAPE_RIGHT = 0x03  # Landscape right
  ORIENT_TYPE_PORTRAIT_DOWN = 0x04  # Portrait upside down

  # IMU face direction type (low byte)
  ORIENT_FACE_UP = 0x00  # Face forward
  ORIENT_FACE_DOWN = 0x01  # Face backward

  # Pressure out-of-range count limit enumeration
  PRESS_OOR_COUNT_LIMIT_1 = 0x00  # OOR count limit: 1 time
  PRESS_OOR_COUNT_LIMIT_3 = 0x01  # OOR count limit: 3 times
  PRESS_OOR_COUNT_LIMIT_7 = 0x02  # OOR count limit: 7 times
  PRESS_OOR_COUNT_LIMIT_15 = 0x03  # OOR count limit: 15 times

  # I2C mode register addresses
  REG_I2C_VID = 0x0000
  REG_I2C_PID = 0x0001
  REG_I2C_FW_VERSION = 0x0002
  REG_I2C_DEVICE_ADDR = 0x0003
  REG_I2C_SENSORS_RESET = 0x0004
  REG_I2C_SENSORS_MODE = 0x0005
  REG_I2C_ACC_RANGE_CONF = 0x0006
  REG_I2C_GYR_RANGE_CONF = 0x0007
  REG_I2C_PRESS_OOR_THR_LOW_WORD_CONF = 0x0008
  REG_I2C_PRESS_OOR_THR_HIGH_WORD_CONF = 0x0009
  REG_I2C_PRESS_OOR_RANGE_CONF = 0x000A
  REG_I2C_PRESS_OOR_CNT_LIM_CONF = 0x000B
  REG_I2C_INT1_CONF = 0x000C
  REG_I2C_INT2_CONF = 0x000D
  REG_I2C_INT3_CONF = 0x000E
  REG_I2C_INT4_CONF = 0x000F
  REG_I2C_ACC_DATA_X = 0x0010
  REG_I2C_ACC_DATA_Y = 0x0011
  REG_I2C_ACC_DATA_Z = 0x0012
  REG_I2C_GYR_DATA_X = 0x0013
  REG_I2C_GYR_DATA_Y = 0x0014
  REG_I2C_GYR_DATA_Z = 0x0015
  REG_I2C_MAG_DATA_X_LOW_WORD = 0x0016
  REG_I2C_MAG_DATA_X_HIGH_WORD = 0x0017
  REG_I2C_MAG_DATA_Y_LOW_WORD = 0x0018
  REG_I2C_MAG_DATA_Y_HIGH_WORD = 0x0019
  REG_I2C_MAG_DATA_Z_LOW_WORD = 0x001A
  REG_I2C_MAG_DATA_Z_HIGH_WORD = 0x001B
  REG_I2C_PRESS_DATA_LOW_WORD = 0x001C
  REG_I2C_PRESS_DATA_HIGH_WORD = 0x001D
  REG_I2C_INT_STEP_DATA_LOW_WORD = 0x001E
  REG_I2C_INT_STEP_DATA_HIGH_WORD = 0x001F
  REG_I2C_INT_TAP_DATA = 0x0020
  REG_I2C_INT_ORIENTATION_DATA = 0x0021

  # UART mode input registers (read-only) - Modbus function code 0x04
  REG_I_VID = 0x0000
  REG_I_PID = 0x0001
  REG_I_DEVICE_ADDR = 0x0002
  REG_I_RESERVED = 0x0003
  REG_I_VERIFY_AND_STOP = 0x0004
  REG_I_FW_VERSION = 0x0005
  REG_I_ACC_DATA_X = 0x0006
  REG_I_ACC_DATA_Y = 0x0007
  REG_I_ACC_DATA_Z = 0x0008
  REG_I_GYR_DATA_X = 0x0009
  REG_I_GYR_DATA_Y = 0x000A
  REG_I_GYR_DATA_Z = 0x000B
  REG_I_MAG_DATA_X_LOW_WORD = 0x000C
  REG_I_MAG_DATA_X_HIGH_WORD = 0x000D
  REG_I_MAG_DATA_Y_LOW_WORD = 0x000E
  REG_I_MAG_DATA_Y_HIGH_WORD = 0x000F
  REG_I_MAG_DATA_Z_LOW_WORD = 0x0010
  REG_I_MAG_DATA_Z_HIGH_WORD = 0x0011
  REG_I_PRESS_DATA_LOW_WORD = 0x0012
  REG_I_PRESS_DATA_HIGH_WORD = 0x0013
  REG_I_INT_STEP_DATA_LOW_WORD = 0x0014
  REG_I_INT_STEP_DATA_HIGH_WORD = 0x0015
  REG_I_INT_TAP_DATA = 0x0016
  REG_I_INT_ORIENTATION_DATA = 0x0017

  # UART mode holding registers (read-write) - Modbus function code 0x03/0x06
  REG_H_RESERVED = 0x0000
  REG_H_RESERVED1 = 0x0001
  REG_H_RESERVED2 = 0x0002
  REG_H_BAUDRATE = 0x0003
  REG_H_RESERVED3 = 0x0004
  REG_H_RESERVED4 = 0x0005
  REG_H_SENSORS_MODE = 0x0006
  REG_H_SENSORS_RESET = 0x0007
  REG_H_ACC_RANGE_CONF = 0x0008
  REG_H_GYR_RANGE_CONF = 0x0009
  REG_H_PRESS_OOR_THR_LOW_WORD_CONF = 0x000A
  REG_H_PRESS_OOR_THR_HIGH_WORD_CONF = 0x000B
  REG_H_PRESS_OOR_RANGE_CONF = 0x000C
  REG_H_PRESS_OOR_CNT_LIM_CONF = 0x000D
  REG_H_INT1_CONF = 0x000E
  REG_H_INT2_CONF = 0x000F
  REG_H_INT3_CONF = 0x0010
  REG_H_INT4_CONF = 0x0011

  def __init__(self, sensor_model=SENSOR_MODEL_10DOF):
    '''!
    @fn __init__
    @brief Constructor
    @param sensor_model Sensor model (see SENSOR_MODEL_*), default is 10DOF IMU
    '''
    self._accel_range = 8.0
    self._gyro_range = 2000.0
    self._sensor_model = sensor_model
    self._calibrated = False
    self._sealevel_altitude = 0.0

  # ==================== Public API Methods ====================

  def begin(self):
    '''!
    @fn begin
    @brief Initialize sensor
    @return True if initialization successful, False otherwise
    '''
    pid_reg = self._get_reg_addr(self.REG_I_PID, self.REG_I2C_PID)
    data = self._read_reg(pid_reg, 2)

    if data is None:
      return False

    pid = data[0] | (data[1] << 8)

    if pid < self._sensor_model:
      return False

    return True

  def reset(self):
    '''!
    @fn reset
    @brief Restore factory settings
    @details Restore sensor to factory default settings
    @n When called in I2C mode, it will restore UART mode baud rate to 9600bps
    @return True if factory reset successful, False otherwise
    '''
    reset_reg = self._get_reg_addr(self.REG_H_SENSORS_RESET, self.REG_I2C_SENSORS_RESET)
    data = [0x01, 0x00]
    self._write_reg(reset_reg, data)

    # Always update local range values to default after reset
    self._accel_range = 8.0
    self._gyro_range = 2000.0

    return True

  def set_accel_range(self, accel_range):
    '''!
    @fn set_accel_range
    @brief Set accelerometer range
    @param accel_range Accelerometer range (see ACCEL_RANGE_*)
    @n Available ranges:
    @n - ACCEL_RANGE_2G:  ±2g range
    @n - ACCEL_RANGE_4G:  ±4g range
    @n - ACCEL_RANGE_8G:  ±8g range
    @n - ACCEL_RANGE_16G: ±16g range
    @return True if setting successful, False otherwise
    '''
    if accel_range > self.ACCEL_RANGE_16G:
      return False

    range_map = {self.ACCEL_RANGE_2G: 2.0, self.ACCEL_RANGE_4G: 4.0, self.ACCEL_RANGE_8G: 8.0, self.ACCEL_RANGE_16G: 16.0}
    self._accel_range = range_map.get(accel_range, 2.0)

    accel_range_reg = self._get_reg_addr(self.REG_H_ACC_RANGE_CONF, self.REG_I2C_ACC_RANGE_CONF)
    data = [accel_range & 0xFF, (accel_range >> 8) & 0xFF]
    ret = self._write_reg(accel_range_reg, data)

    return ret == self.RET_CODE_OK

  def set_gyro_range(self, gyro_range):
    '''!
    @fn set_gyro_range
    @brief Set gyroscope range
    @param gyro_range Gyroscope range (see GYRO_RANGE_*)
    @n Available ranges:
    @n - GYRO_RANGE_125DPS:  ±125dps range
    @n - GYRO_RANGE_250DPS:  ±250dps range
    @n - GYRO_RANGE_500DPS:  ±500dps range
    @n - GYRO_RANGE_1000DPS: ±1000dps range
    @n - GYRO_RANGE_2000DPS: ±2000dps range
    @return True if setting successful, False otherwise
    '''
    if gyro_range > self.GYRO_RANGE_2000DPS:
      return False

    range_map = {self.GYRO_RANGE_125DPS: 125.0, self.GYRO_RANGE_250DPS: 250.0, self.GYRO_RANGE_500DPS: 500.0, self.GYRO_RANGE_1000DPS: 1000.0, self.GYRO_RANGE_2000DPS: 2000.0}
    self._gyro_range = range_map.get(gyro_range, 250.0)

    gyro_range_reg = self._get_reg_addr(self.REG_H_GYR_RANGE_CONF, self.REG_I2C_GYR_RANGE_CONF)
    data = [gyro_range & 0xFF, (gyro_range >> 8) & 0xFF]
    ret = self._write_reg(gyro_range_reg, data)

    return ret == self.RET_CODE_OK

  def set_sensor_mode(self, mode):
    '''!
    @fn set_sensor_mode
    @brief Set sensor operating mode
    @param mode Sensor operating mode (see *_MODE)
    @n Available modes:
    @n - SLEEP_MODE:            Sleep mode (lowest power consumption, sensor stops working)
    @n - LOW_POWER_MODE:        Low power mode (reduced sampling rate, saves power)
    @n - NORMAL_MODE:           Normal mode (balances power consumption and performance)
    @n - HIGH_PERFORMANCE_MODE: High performance mode (highest sampling rate and accuracy)
    @return True if setting successful, False otherwise
    '''
    if mode > self.HIGH_PERFORMANCE_MODE:
      return False

    sensors_mode_reg = self._get_reg_addr(self.REG_H_SENSORS_MODE, self.REG_I2C_SENSORS_MODE)
    data = [mode & 0xFF, (mode >> 8) & 0xFF]
    ret = self._write_reg(sensors_mode_reg, data)

    return ret == self.RET_CODE_OK

  def get_6dof_data(self):
    '''!
    @fn get_6dof_data
    @brief Read 6DOF IMU data (physical units)
    @return Dictionary with accel (g) and gyro (dps) data, or None if failed
    @n Example: {'accel': {'x': 0.0, 'y': 0.0, 'z': 1.0}, 'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    '''
    raw_data = self._get_6axis_raw_data()
    if raw_data is None:
      return None

    return {
      'accel': {'x': self._lsb_to_g(raw_data['accel_x'], self._accel_range), 'y': self._lsb_to_g(raw_data['accel_y'], self._accel_range), 'z': self._lsb_to_g(raw_data['accel_z'], self._accel_range)},
      'gyro': {'x': self._lsb_to_dps(raw_data['gyro_x'], self._gyro_range), 'y': self._lsb_to_dps(raw_data['gyro_y'], self._gyro_range), 'z': self._lsb_to_dps(raw_data['gyro_z'], self._gyro_range)},
    }

  def get_9dof_data(self):
    '''!
    @fn get_9dof_data
    @brief Read 9DOF IMU data (physical units)
    @details Read 6DOF data (accelerometer+gyroscope) and 3-axis magnetometer data
    @return Dictionary with accel (g), gyro (dps), and mag (uT) data, or None if failed
    @n Example: {'accel': {...}, 'gyro': {...}, 'mag': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    '''
    data_6dof = self.get_6dof_data()
    if data_6dof is None:
      return None

    mag_raw_data = self._read_mag_raw_bytes()
    if mag_raw_data is None:
      return None

    mag_x = self._to_signed_32(mag_raw_data[0], mag_raw_data[1], mag_raw_data[2], mag_raw_data[3])
    mag_y = self._to_signed_32(mag_raw_data[4], mag_raw_data[5], mag_raw_data[6], mag_raw_data[7])
    mag_z = self._to_signed_32(mag_raw_data[8], mag_raw_data[9], mag_raw_data[10], mag_raw_data[11])

    data_6dof['mag'] = {'x': mag_x * 0.01, 'y': mag_y * 0.01, 'z': mag_z * 0.01}

    return data_6dof

  def get_10dof_data(self, calc_altitude=False):
    '''!
    @fn get_10dof_data
    @brief Read 10DOF IMU data (physical units)
    @details Read 6DOF data (accelerometer+gyroscope), 3-axis magnetometer data and pressure data
    @param calc_altitude Whether to calculate altitude, default is False
    @n True: pressure stores altitude (unit: m)
    @n False: pressure stores pressure data (unit: Pa)
    @return Dictionary with accel (g), gyro (dps), mag (uT), and pressure (Pa or m) data, or None if failed
    @n Example: {'accel': {...}, 'gyro': {...}, 'mag': {...}, 'pressure': 101325.0}
    '''
    data_9dof = self.get_9dof_data()
    if data_9dof is None:
      return None

    press_raw_data = self._read_press_raw_bytes()
    if press_raw_data is None:
      return None

    press_raw = self._to_signed_32(press_raw_data[0], press_raw_data[1], press_raw_data[2], press_raw_data[3])
    pressure_value = press_raw

    # Pressure: always return raw. Altitude: when calibrated, use sea-level adjusted pressure for formula (same as BMP58X).
    if calc_altitude:
      pressure_for_alt = pressure_value
      if self._calibrated:
        STANDARD_SEA_LEVEL_PRESSURE_PA = 101325.0
        sea_level_press_pa = pressure_value / math.pow(1.0 - (self._sealevel_altitude / 44307.7), 5.255302)
        pressure_for_alt = pressure_value - sea_level_press_pa + STANDARD_SEA_LEVEL_PRESSURE_PA
      data_9dof['pressure'] = self._calculate_altitude(pressure_for_alt)
    else:
      data_9dof['pressure'] = pressure_value

    return data_9dof

  def calibrate_altitude(self, altitude):
    '''!
    @fn calibrate_altitude
    @brief Calibrate altitude data based on local altitude
    @param altitude Local altitude (unit: m)
    @n For example: 540.0 means altitude of 540 meters
    @n After calling this function, the altitude in get_10dof_data (when calc_altitude is True) will be calibrated to eliminate absolute errors
    @n If this function is not called, the altitude measurement will not eliminate absolute errors
    @return True if calibration successful (altitude > 0), False otherwise
    '''
    if altitude > 0:
      self._calibrated = True
      self._sealevel_altitude = altitude
      return True
    return False

  def set_press_oor(self, threshold, range_val, count_limit):
    '''!
    @fn set_press_oor
    @brief Configure pressure out-of-range (OOR) parameters
    @param threshold Pressure threshold (unit: Pa)
    @n For example: 100000 Pa (standard sea level pressure is approximately 101325 Pa)
    @param range_val Allowed range (unit: Pa)
    @n For example: 50 Pa, means allowed range is threshold ± range
    @n Actual allowed range: threshold - range ~ threshold + range
    @param count_limit Count limit (see PRESS_OOR_COUNT_LIMIT_*)
    @n Interrupt is triggered only after N consecutive out-of-range occurrences
    @n Available values:
    @n - PRESS_OOR_COUNT_LIMIT_1:  Trigger after 1 consecutive occurrence
    @n - PRESS_OOR_COUNT_LIMIT_3:  Trigger after 3 consecutive occurrences
    @n - PRESS_OOR_COUNT_LIMIT_7:  Trigger after 7 consecutive occurrences
    @n - PRESS_OOR_COUNT_LIMIT_15: Trigger after 15 consecutive occurrences
    @return True if configuration successful, False otherwise
    '''
    if count_limit > self.PRESS_OOR_COUNT_LIMIT_15:
      return False

    threshold_low = threshold & 0xFFFF
    thr_low_reg = self._get_reg_addr(self.REG_H_PRESS_OOR_THR_LOW_WORD_CONF, self.REG_I2C_PRESS_OOR_THR_LOW_WORD_CONF)
    data = [threshold_low & 0xFF, (threshold_low >> 8) & 0xFF]
    ret = self._write_reg(thr_low_reg, data)
    if ret != self.RET_CODE_OK:
      return False
    time.sleep(0.01)

    threshold_high = (threshold >> 16) & 0xFFFF
    thr_high_reg = self._get_reg_addr(self.REG_H_PRESS_OOR_THR_HIGH_WORD_CONF, self.REG_I2C_PRESS_OOR_THR_HIGH_WORD_CONF)
    data = [threshold_high & 0xFF, (threshold_high >> 8) & 0xFF]
    ret = self._write_reg(thr_high_reg, data)
    if ret != self.RET_CODE_OK:
      return False
    time.sleep(0.01)

    range_reg = self._get_reg_addr(self.REG_H_PRESS_OOR_RANGE_CONF, self.REG_I2C_PRESS_OOR_RANGE_CONF)
    data = [range_val & 0xFF, (range_val >> 8) & 0xFF]
    ret = self._write_reg(range_reg, data)
    if ret != self.RET_CODE_OK:
      return False
    time.sleep(0.01)

    cnt_lim_reg = self._get_reg_addr(self.REG_H_PRESS_OOR_CNT_LIM_CONF, self.REG_I2C_PRESS_OOR_CNT_LIM_CONF)
    data = [count_limit & 0xFF, (count_limit >> 8) & 0xFF]
    ret = self._write_reg(cnt_lim_reg, data)
    if ret != self.RET_CODE_OK:
      return False

    time.sleep(0.05)
    return True

  def set_int(self, pin, int_type):
    '''!
    @fn set_int
    @brief Configure interrupt (unified API with type-safe enumeration)
    @param pin Interrupt pin (see IMU_INT_PIN_*)
    @n Available pins:
    @n - IMU_INT_PIN_INT1: INT1 pin (6DOF sensor, supports multiple interrupt types)
    @n - IMU_INT_PIN_INT2: INT2 pin (6DOF sensor, supports multiple interrupt types)
    @n - IMU_INT_PIN_INT3: INT3 pin (9DOF sensor-magnetometer, only supports data ready interrupt)
    @n - IMU_INT_PIN_INT4: INT4 pin (10DOF sensor-barometer, supports data ready and pressure OOR interrupt)
    @param int_type Interrupt type (see INT1_2_*, INT3_*, INT4_*)
    @n INT1/INT2 supported interrupt types:
    @n - INT1_2_DISABLE (0x00):      Disable interrupt
    @n - INT1_2_DATA_READY (0x01):   Data ready interrupt
    @n - INT1_2_ANY_MOTION (0x02):   Any motion interrupt
    @n - INT1_2_NO_MOTION (0x03):    No motion interrupt
    @n - INT1_2_SIG_MOTION (0x04):   Significant motion interrupt
    @n - INT1_2_STEP_COUNTER (0x05): Step counter interrupt
    @n - INT1_2_FLAT (0x06):         Flat interrupt
    @n - INT1_2_ORIENTATION (0x07):  Orientation interrupt
    @n - INT1_2_TAP (0x08):          Tap interrupt
    @n - INT1_2_TILT (0x09):         Tilt interrupt
    @n INT3 supported interrupt types:
    @n - INT3_DISABLE (0x10):    Disable interrupt
    @n - INT3_DATA_READY (0x11): Data ready interrupt
    @n INT4 supported interrupt types:
    @n - INT4_DISABLE (0x20):      Disable interrupt
    @n - INT4_DATA_READY (0x21):   Data ready interrupt
    @n - INT4_PRESSURE_OOR (0x22): Pressure out-of-range interrupt
    @return True if configuration successful, False otherwise
    '''
    if pin == self.IMU_INT_PIN_NONE or pin > self.IMU_INT_PIN_INT4:
      return False

    success, macro_value = self._int_type_to_macro_value(pin, int_type)
    if not success:
      return False

    if pin == self.IMU_INT_PIN_INT1:
      conf_reg = self._get_reg_addr(self.REG_H_INT1_CONF, self.REG_I2C_INT1_CONF)
    elif pin == self.IMU_INT_PIN_INT2:
      conf_reg = self._get_reg_addr(self.REG_H_INT2_CONF, self.REG_I2C_INT2_CONF)
    elif pin == self.IMU_INT_PIN_INT3:
      conf_reg = self._get_reg_addr(self.REG_H_INT3_CONF, self.REG_I2C_INT3_CONF)
    elif pin == self.IMU_INT_PIN_INT4:
      conf_reg = self._get_reg_addr(self.REG_H_INT4_CONF, self.REG_I2C_INT4_CONF)
    else:
      return False

    data = [macro_value & 0xFF, (macro_value >> 8) & 0xFF]
    ret = self._write_reg(conf_reg, data)

    return ret == self.RET_CODE_OK

  def get_step_count(self):
    '''!
    @fn get_step_count
    @brief Read step counter data
    @details Read current cumulative step count
    @n After detecting step interrupt, call this function to read cumulative step count
    @return Cumulative step count (32-bit)
    @retval 0 No step data or read failed
    '''
    step_data_low_reg = self._get_reg_addr(self.REG_I_INT_STEP_DATA_LOW_WORD, self.REG_I2C_INT_STEP_DATA_LOW_WORD)
    data = self._read_reg(step_data_low_reg, 4)

    if data is None:
      return 0

    return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)

  def get_tap(self):
    '''!
    @fn get_tap
    @brief Read tap data
    @details When tap interrupt is detected, call this function to read the specific tap type
    @return Tap data
    @n Return values:
    @n - TAP_TYPE_SINGLE (0x0001): Single tap
    @n - TAP_TYPE_DOUBLE (0x0002): Double tap
    @n - TAP_TYPE_TRIPLE (0x0003): Triple tap
    @retval 0 No tap data or read failed
    '''
    tap_data_reg = self._get_reg_addr(self.REG_I_INT_TAP_DATA, self.REG_I2C_INT_TAP_DATA)
    data = self._read_reg(tap_data_reg, 2)

    if data is None:
      return 0

    return data[0] | (data[1] << 8)

  def get_orientation(self):
    '''!
    @fn get_orientation
    @brief Read orientation data
    @details When orientation interrupt is detected, call this function to read the specific orientation and face direction
    @return Orientation data
    @n High byte: Orientation type
    @n - ORIENT_TYPE_PORTRAIT_UP (0x01):     Portrait upright
    @n - ORIENT_TYPE_LANDSCAPE_LEFT (0x02):  Landscape left
    @n - ORIENT_TYPE_LANDSCAPE_RIGHT (0x03): Landscape right
    @n - ORIENT_TYPE_PORTRAIT_DOWN (0x04):   Portrait upside down
    @n Low byte: Face direction type
    @n - ORIENT_FACE_UP (0x00):   Face forward
    @n - ORIENT_FACE_DOWN (0x01): Face backward
    @retval 0 No orientation data or read failed
    '''
    orient_data_reg = self._get_reg_addr(self.REG_I_INT_ORIENTATION_DATA, self.REG_I2C_INT_ORIENTATION_DATA)
    data = self._read_reg(orient_data_reg, 2)

    if data is None:
      return 0

    return data[0] | (data[1] << 8)

  # ==================== Private Methods ====================

  def _get_comm_mode(self):
    '''!
    @fn _get_comm_mode
    @brief Get communication mode (pure virtual function, implemented by subclasses)
    @return Communication mode
    '''
    raise NotImplementedError("Subclass must implement _get_comm_mode")

  def _write_reg(self, reg, data):
    '''!
    @fn _write_reg
    @brief Write register (pure virtual function, implemented by subclasses)
    @param reg Register address
    @param data Data to write (list of bytes)
    @return Return code
    '''
    raise NotImplementedError("Subclass must implement _write_reg")

  def _read_reg(self, reg, length, reg_type=None):
    '''!
    @fn _read_reg
    @brief Read register (pure virtual function, implemented by subclasses)
    @param reg Register address
    @param length Data length to read
    @param reg_type Register type (see REG_TYPE_*), default is REG_TYPE_INPUT
    @n In UART mode: REG_TYPE_INPUT uses Modbus function code 0x04, REG_TYPE_HOLDING uses function code 0x03
    @n In I2C mode: this parameter is ignored
    @return Data read (list of bytes) or None if failed
    '''
    raise NotImplementedError("Subclass must implement _read_reg")

  def _get_reg_addr(self, uart_reg, i2c_reg):
    '''!
    @fn _get_reg_addr
    @brief Get correct register address based on communication mode
    @param uart_reg Register address for UART mode
    @param i2c_reg Register address for I2C mode
    @return Register address returned based on current communication mode
    '''
    if self._get_comm_mode() == self.COMM_MODE_I2C:
      return i2c_reg
    else:
      return uart_reg

  def _to_signed_16(self, low, high):
    '''!
    @fn _to_signed_16
    @brief Convert two bytes to signed 16-bit integer
    @param low Low byte
    @param high High byte
    @return Signed 16-bit integer
    '''
    val = low | (high << 8)
    if val >= 0x8000:
      val -= 0x10000
    return val

  def _to_signed_32(self, b0, b1, b2, b3):
    '''!
    @fn _to_signed_32
    @brief Convert four bytes to signed 32-bit integer
    @param b0 Byte 0 (LSB)
    @param b1 Byte 1
    @param b2 Byte 2
    @param b3 Byte 3 (MSB)
    @return Signed 32-bit integer
    '''
    val = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    if val >= 0x80000000:
      val -= 0x100000000
    return val

  def _lsb_to_g(self, val, g_range):
    '''!
    @fn _lsb_to_g
    @brief Convert accelerometer raw data to physical quantity (g)
    @param val Raw data (16-bit signed integer)
    @param g_range Accelerometer range
    @return Accelerometer physical quantity, unit: g
    '''
    half_scale = 32768.0
    return (val * g_range) / half_scale

  def _lsb_to_dps(self, val, dps_range):
    '''!
    @fn _lsb_to_dps
    @brief Convert gyroscope raw data to physical quantity (dps)
    @param val Raw data (16-bit signed integer)
    @param dps_range Gyroscope range
    @return Gyroscope physical quantity, unit: dps
    '''
    half_scale = 32768.0
    return (val * dps_range) / half_scale

  def _read_6axis_raw_bytes(self):
    '''!
    @fn _read_6axis_raw_bytes
    @brief Read raw 6DOF data byte stream
    @return 12 bytes data or None if failed
    '''
    acc_data_x_reg = self._get_reg_addr(self.REG_I_ACC_DATA_X, self.REG_I2C_ACC_DATA_X)
    return self._read_reg(acc_data_x_reg, 12)

  def _get_6axis_raw_data(self):
    '''!
    @fn _get_6axis_raw_data
    @brief Read 6DOF IMU raw data (16-bit integer)
    @return Dictionary with raw data or None if failed
    '''
    raw_data = self._read_6axis_raw_bytes()
    if raw_data is None:
      return None

    return {
      'accel_x': self._to_signed_16(raw_data[0], raw_data[1]),
      'accel_y': self._to_signed_16(raw_data[2], raw_data[3]),
      'accel_z': self._to_signed_16(raw_data[4], raw_data[5]),
      'gyro_x': self._to_signed_16(raw_data[6], raw_data[7]),
      'gyro_y': self._to_signed_16(raw_data[8], raw_data[9]),
      'gyro_z': self._to_signed_16(raw_data[10], raw_data[11]),
    }

  def _read_mag_raw_bytes(self):
    '''!
    @fn _read_mag_raw_bytes
    @brief Read raw magnetometer data byte stream
    @return 12 bytes data or None if failed
    '''
    mag_data_x_low_reg = self._get_reg_addr(self.REG_I_MAG_DATA_X_LOW_WORD, self.REG_I2C_MAG_DATA_X_LOW_WORD)
    return self._read_reg(mag_data_x_low_reg, 12)

  def _read_press_raw_bytes(self):
    '''!
    @fn _read_press_raw_bytes
    @brief Read raw pressure data byte stream
    @return 4 bytes data or None if failed
    '''
    press_data_low_reg = self._get_reg_addr(self.REG_I_PRESS_DATA_LOW_WORD, self.REG_I2C_PRESS_DATA_LOW_WORD)
    return self._read_reg(press_data_low_reg, 4)

  def _calculate_altitude(self, pressure):
    '''!
    @fn _calculate_altitude
    @brief Calculate altitude based on pressure
    @param pressure Pressure value, unit: Pa
    @return Altitude, unit: m
    '''
    # Barometric formula (align with BMP58X): h = 44307.7 * (1 - (P/P0)^0.190284)
    sea_level_pressure = 101325.0
    return 44307.7 * (1.0 - math.pow(pressure / sea_level_pressure, 0.190284))

  def _int_type_to_macro_value(self, pin, int_type):
    '''!
    @fn _int_type_to_macro_value
    @brief Convert unified interrupt type enumeration to corresponding macro value
    @param pin Interrupt pin
    @param int_type Unified interrupt type enumeration value
    @return Tuple (success, macro_value)
    '''
    if pin == self.IMU_INT_PIN_INT1 or pin == self.IMU_INT_PIN_INT2:
      if int_type < self.INT1_2_DISABLE or int_type > self.INT1_2_TILT:
        return (False, 0)
      return (True, int_type)
    elif pin == self.IMU_INT_PIN_INT3:
      if int_type < self.INT3_DISABLE or int_type > self.INT3_DATA_READY:
        return (False, 0)
      return (True, int_type - 0x10)
    elif pin == self.IMU_INT_PIN_INT4:
      if int_type < self.INT4_DISABLE or int_type > self.INT4_PRESSURE_OOR:
        return (False, 0)
      return (True, int_type - 0x20)
    else:
      return (False, 0)


class DFRobot_Multi_DOF_IMU_I2C(DFRobot_Multi_DOF_IMU):
  '''!
  @brief DFRobot_Multi_DOF_IMU I2C communication subclass
  @details Communicate with IMU sensor through I2C interface
  @note This class requires smbus2 for proper 16-bit register address support.
  @n    Install using: pip install smbus2
  '''

  def __init__(self, sensor_model=DFRobot_Multi_DOF_IMU.SENSOR_MODEL_10DOF, bus=1, addr=0x4A):
    '''!
    @fn __init__
    @brief Constructor
    @param sensor_model Sensor model (see SENSOR_MODEL_*), default is 10DOF IMU
    @param bus I2C bus number, default is 1
    @param addr I2C device address, default is 0x4A
    '''
    super().__init__(sensor_model)
    self._bus = bus
    self._i2c_addr = addr
    self._i2c = None
    self._i2c_msg = None

  def begin(self):
    '''!
    @fn begin
    @brief Initialize I2C communication and sensor
    @return True if initialization successful, False otherwise
    '''
    try:
      import smbus2
      from smbus2 import i2c_msg

      self._i2c = smbus2.SMBus(self._bus)
      self._i2c_msg = i2c_msg
    except ImportError:
      print("Error: smbus2 module not found. Please install it using 'pip install smbus2'")
      return False
    except Exception as e:
      print("Error initializing I2C: {}".format(e))
      return False

    return super().begin()

  def _get_comm_mode(self):
    '''!
    @fn _get_comm_mode
    @brief Get communication mode
    @return Returns I2C communication mode
    '''
    return self.COMM_MODE_I2C

  def _write_reg(self, reg, data):
    '''!
    @fn _write_reg
    @brief Write register via I2C
    @param reg Register address (16-bit)
    @param data Data to write (list of bytes)
    @return Return code
    '''
    if self._i2c is None:
      return self.RET_CODE_ERROR

    try:
      # Send: [reg_low, reg_high, data...]
      write_data = [reg & 0xFF, (reg >> 8) & 0xFF] + list(data)
      write_msg = self._i2c_msg.write(self._i2c_addr, write_data)
      self._i2c.i2c_rdwr(write_msg)
      return self.RET_CODE_OK
    except Exception as e:
      print("I2C write error: {}".format(e))
      return self.RET_CODE_ERROR

  def _read_reg(self, reg, length, reg_type=None):
    '''!
    @fn _read_reg
    @brief Read register via I2C
    @details Uses two-phase I2C transaction with delay for firmware processing.
    @n       Phase 1: Write register address (2 bytes: low, high)
    @n       Phase 2: Wait 10ms for firmware to prepare data
    @n       Phase 3: Read data (length bytes)
    @param reg Register address (16-bit)
    @param length Data length to read
    @param reg_type Register type (ignored in I2C mode, kept for API compatibility)
    @return Data read (list of bytes) or None if failed
    '''
    # reg_type is ignored in I2C mode, all registers share the same address space
    if self._i2c is None:
      return None

    try:
      # Phase 1: Write register address (2 bytes: low, high)
      write_msg = self._i2c_msg.write(self._i2c_addr, [reg & 0xFF, (reg >> 8) & 0xFF])
      self._i2c.i2c_rdwr(write_msg)

      # Phase 2: Wait for firmware to prepare data (required for magnetometer/barometer)
      time.sleep(0.01)

      # Phase 3: Read data
      read_msg = self._i2c_msg.read(self._i2c_addr, length)
      self._i2c.i2c_rdwr(read_msg)
      return list(read_msg)
    except Exception as e:
      print("I2C read error: {}".format(e))
      return None


class DFRobot_Multi_DOF_IMU_UART(DFRobot_Multi_DOF_IMU, DFRobot_RTU):
  '''!
  @brief DFRobot_Multi_DOF_IMU UART communication subclass
  @details Communicate with IMU sensor through UART interface using Modbus RTU protocol
  @note Uses DFRobot_RTU library for Modbus RTU communication
  '''

  def __init__(self, sensor_model=DFRobot_Multi_DOF_IMU.SENSOR_MODEL_10DOF, baud=9600, addr=0x4A):
    '''!
    @fn __init__
    @brief Constructor
    @param sensor_model Sensor model (see SENSOR_MODEL_*), default is 10DOF IMU
    @param baud Baud rate, default is 9600
    @param addr Device address, default is 0x4A (same as I2C address, Modbus range: 1-247)
    '''
    self._baud = baud
    self._device_addr = addr
    DFRobot_Multi_DOF_IMU.__init__(self, sensor_model)
    DFRobot_RTU.__init__(self, baud, 8, 'N', 1)

  def begin(self):
    '''!
    @fn begin
    @brief Initialize UART communication and sensor
    @return True if initialization successful, False otherwise
    '''
    # Use default timeout (0.1s) for normal operation
    # Note: DFRobot_RTU uses _timeout in _send_package for each transaction
    return DFRobot_Multi_DOF_IMU.begin(self)

  def _get_comm_mode(self):
    '''!
    @fn _get_comm_mode
    @brief Get communication mode
    @return Returns UART communication mode
    '''
    return self.COMM_MODE_UART

  def _write_reg(self, reg, data):
    '''!
    @fn _write_reg
    @brief Write register via UART (using Modbus RTU protocol)
    @details Writes to holding registers using Modbus function code 0x06
    @param reg Register address (holding register)
    @param data Data to write (list of bytes, 2 bytes per register)
    @return Return code (RET_CODE_OK or RET_CODE_ERROR)
    '''
    try:
      # Data is byte list, need to convert to Modbus format
      # Each Modbus register is 16-bit, data is 8-bit byte list
      num_regs = (len(data) + 1) // 2

      if num_regs == 1:
        # Single register write (function code 0x06)
        low_byte = data[0] if len(data) >= 1 else 0
        high_byte = data[1] if len(data) >= 2 else 0
        val = (high_byte << 8) | low_byte
        ret = self.write_holding_register(self._device_addr, reg, val)
        return self.RET_CODE_OK if ret == 0 else self.RET_CODE_ERROR
      else:
        # Multiple registers write (write sequentially like C++ version)
        for i in range(num_regs):
          byte_offset = i * 2
          low_byte = data[byte_offset] if byte_offset < len(data) else 0
          high_byte = data[byte_offset + 1] if byte_offset + 1 < len(data) else 0
          val = (high_byte << 8) | low_byte
          ret = self.write_holding_register(self._device_addr, reg + i, val)
          if ret != 0:
            return self.RET_CODE_ERROR
          time.sleep(0.01)  # 10ms delay between writes
        return self.RET_CODE_OK
    except Exception as e:
      print("UART write error: {}".format(e))
      return self.RET_CODE_ERROR

  def _read_reg(self, reg, length, reg_type=None):
    '''!
    @fn _read_reg
    @brief Read register via UART (using Modbus RTU protocol)
    @details Uses reg_type parameter to determine which Modbus function code to use
    @n       REG_TYPE_INPUT: Read from input registers (Modbus function code 0x04)
    @n       REG_TYPE_HOLDING: Read from holding registers (Modbus function code 0x03)
    @param reg Register address
    @param length Data length to read (in bytes)
    @param reg_type Register type (see REG_TYPE_*), default is REG_TYPE_INPUT
    @return Data read (list of bytes) or None if failed
    '''
    try:
      # Use reg_type parameter to determine which Modbus function code to use
      # Default to input register if not specified
      if reg_type is None:
        reg_type = self.REG_TYPE_INPUT
      is_input_register = reg_type == self.REG_TYPE_INPUT

      # Calculate number of registers to read (each register is 2 bytes)
      num_regs = (length + 1) // 2

      # Use batch read for efficiency (single Modbus transaction)
      if is_input_register:
        result = self.read_input_registers(self._device_addr, reg, num_regs)
      else:
        result = self.read_holding_registers(self._device_addr, reg, num_regs)

      # Check for errors (result[0] is error code)
      if result[0] != 0:
        return None

      # Convert Modbus format to byte list
      # result format: [error_code, high1, low1, high2, low2, ...]
      data = []
      for i in range(num_regs):
        high_byte = result[i * 2 + 1]
        low_byte = result[i * 2 + 2]
        data.append(low_byte)  # Little-endian: low byte first
        data.append(high_byte)  # Then high byte

      return data[:length]
    except Exception as e:
      print("UART read error: {}".format(e))
      return None
