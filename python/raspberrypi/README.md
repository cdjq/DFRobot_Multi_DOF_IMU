# DFRobot_Multi_DOF_IMU
- [Chinese Version](./README_CN.md)

DFRobot_Multi_DOF_IMU is a multi-DOF IMU sensor Raspberry Pi Python library that supports reading sensor data through I2C/UART interfaces. This library supports 6DOF (accelerometer + gyroscope), 9DOF (+ magnetometer), and 10DOF (+ barometer) sensors, providing comprehensive motion detection and attitude sensing capabilities.

This sensor is very suitable for wearable devices, smart watches, fitness trackers, drones, robot navigation, and IoT applications that require high performance in motion sensing, attitude detection, and energy efficiency. Through configurable interrupt pins (INT1, INT2, INT3, INT4), the sensor can efficiently notify the host system of various motion events, making it ideal for battery-powered applications.

**Key Features:**

- Multi-DOF motion sensing (6DOF/9DOF/10DOF)
- Supports both I2C and UART (Modbus RTU) communication interfaces
- Hardware step counter with interrupt support
- Multiple motion detection modes (any motion, no motion, significant motion)
- Gesture recognition (tap, tilt, orientation, flat detection)
- Barometric pressure detection and altitude calculation
- Pressure out-of-range (OOR) interrupt
- Multiple operating modes (sleep, low power, normal, high performance)
- Configurable accelerometer and gyroscope ranges
- Multiple interrupt pins for flexible event handling

## Product Link (https://www.dfrobot.com)

```
SKU: SEN0692/SEN0694/SEN0696
```

## Table of Contents

  * [Overview](#overview)
  * [Installation](#installation)
  * [Methods](#methods)
  * [Compatibility](#compatibility)
  * [History](#history)
  * [Credits](#credits)

## Overview

This Python library provides a comprehensive interface for multi-DOF IMU sensors. It supports:

**Basic Functions:**

- Initialize sensor through I2C or UART interface
- Configure sensor operating modes (sleep, low power, normal, high performance)
- Configure accelerometer and gyroscope ranges
- Read 6DOF sensor data (accelerometer + gyroscope)
- Read 9DOF sensor data (accelerometer + gyroscope + magnetometer)
- Read 10DOF sensor data (accelerometer + gyroscope + magnetometer + barometer)
- Pressure calibration (based on local altitude)
- Pressure out-of-range (OOR) detection configuration

**Advanced Functions:**

- Step counter with interrupt support
- Any motion detection interrupt
- No motion detection interrupt
- Significant motion detection interrupt
- Flat detection interrupt
- Orientation detection interrupt (portrait/landscape, face up/down)
- Tap detection interrupt (single/double/triple tap)
- Tilt detection interrupt
- Pressure data ready interrupt
- Pressure out-of-range interrupt

## Installation

1. Download the library to Raspberry Pi using `git clone https://github.com/DFRobot/DFRobot_Multi_DOF_IMU`
2. Open the `python/raspberrypi/examples` folder and run the example programs

## Methods

```python
  '''!
    @fn begin
    @brief Initialize sensor
    @return True if initialization successful, False otherwise
  '''
  def begin(self):

  '''!
    @fn reset
    @brief Restore factory settings
    @return True if factory reset successful, False otherwise
  '''
  def reset(self):

  '''!
    @fn set_sensor_mode
    @brief Set sensor operating mode
    @param mode Sensor operating mode
    @n Available modes:
    @n - SLEEP_MODE:            Sleep mode (lowest power consumption, sensor stops working)
    @n - LOW_POWER_MODE:        Low power mode (reduced sampling rate, saves power)
    @n - NORMAL_MODE:           Normal mode (balances power consumption and performance)
    @n - HIGH_PERFORMANCE_MODE: High performance mode (highest sampling rate and accuracy)
    @return True if setting successful, False otherwise
  '''
  def set_sensor_mode(self, mode):

  '''!
    @fn set_accel_range
    @brief Set accelerometer range
    @param accel_range Accelerometer range
    @n Available ranges:
    @n - ACCEL_RANGE_2G:  ±2g range
    @n - ACCEL_RANGE_4G:  ±4g range
    @n - ACCEL_RANGE_8G:  ±8g range
    @n - ACCEL_RANGE_16G: ±16g range
    @return True if setting successful, False otherwise
  '''
  def set_accel_range(self, accel_range):

  '''!
    @fn set_gyro_range
    @brief Set gyroscope range
    @param gyro_range Gyroscope range
    @n Available ranges:
    @n - GYRO_RANGE_125DPS:  ±125dps range
    @n - GYRO_RANGE_250DPS:  ±250dps range
    @n - GYRO_RANGE_500DPS:  ±500dps range
    @n - GYRO_RANGE_1000DPS: ±1000dps range
    @n - GYRO_RANGE_2000DPS: ±2000dps range
    @return True if setting successful, False otherwise
  '''
  def set_gyro_range(self, gyro_range):

  '''!
    @fn get_6dof_data
    @brief Read 6DOF IMU data (physical units)
    @return dict containing accelerometer and gyroscope data
    @n Return format: {'accel': {'x': float, 'y': float, 'z': float}, 'gyro': {'x': float, 'y': float, 'z': float}}
    @n accel: x, y, z accelerometer data (unit: g)
    @n gyro: x, y, z gyroscope data (unit: dps)
    @retval None if read failed
  '''
  def get_6dof_data(self):

  '''!
    @fn get_9dof_data
    @brief Read 9DOF IMU data (physical units)
    @return dict containing accelerometer, gyroscope and magnetometer data
    @n Return format: {'accel': {...}, 'gyro': {...}, 'mag': {'x': float, 'y': float, 'z': float}}
    @n mag: x, y, z magnetometer data (unit: uT)
    @retval None if read failed
  '''
  def get_9dof_data(self):

  '''!
    @fn get_10dof_data
    @brief Read 10DOF IMU data (physical units)
    @param as_altitude Whether to calculate altitude, default is False
    @n True: pressure stores altitude (unit: m)
    @n False: pressure stores pressure data (unit: Pa)
    @return dict containing accelerometer, gyroscope, magnetometer and pressure/altitude data
    @n Return format: {'accel': {...}, 'gyro': {...}, 'mag': {...}, 'pressure': float}
    @retval None if read failed
  '''
  def get_10dof_data(self, as_altitude=False):

  '''!
    @fn calibrate_altitude
    @brief Calibrate altitude data based on local altitude
    @param altitude Local altitude (unit: m)
    @n For example: 540.0 means altitude of 540 meters
    @n After calling this function, altitude in get_10dof_data (when calc_altitude is True) will be calibrated
    @return True if calibration successful (altitude > 0), False otherwise
  '''
  def calibrate_altitude(self, altitude):

  '''!
    @fn set_press_oor
    @brief Configure pressure out-of-range (OOR) parameters
    @param threshold Pressure threshold (unit: Pa)
    @param range_val Allowed range (unit: Pa)
    @n Actual allowed range: threshold - range_val ~ threshold + range_val
    @param count_limit Count limit
    @n Interrupt is triggered only after N consecutive out-of-range occurrences
    @n Available values:
    @n - PRESS_OOR_COUNT_LIMIT_1:  Trigger after 1 consecutive occurrence
    @n - PRESS_OOR_COUNT_LIMIT_3:  Trigger after 3 consecutive occurrences
    @n - PRESS_OOR_COUNT_LIMIT_7:  Trigger after 7 consecutive occurrences
    @n - PRESS_OOR_COUNT_LIMIT_15: Trigger after 15 consecutive occurrences
    @return True if configuration successful, False otherwise
  '''
  def set_press_oor(self, threshold, range_val, count_limit):

  '''!
    @fn set_int
    @brief Configure interrupt (unified API)
    @param pin Interrupt pin
    @n Available pins:
    @n - IMU_INT_PIN_INT1: INT1 pin (6DOF sensor, supports multiple interrupt types)
    @n - IMU_INT_PIN_INT2: INT2 pin (6DOF sensor, supports multiple interrupt types)
    @n - IMU_INT_PIN_INT3: INT3 pin (9DOF sensor-magnetometer, only supports data ready)
    @n - IMU_INT_PIN_INT4: INT4 pin (10DOF sensor-barometer, supports data ready and OOR)
    @param int_type Interrupt type
    @n INT1/INT2 supported interrupt types:
    @n - INT1_2_DISABLE:      Disable interrupt
    @n - INT1_2_DATA_READY:   Data ready interrupt
    @n - INT1_2_ANY_MOTION:   Any motion interrupt
    @n - INT1_2_NO_MOTION:    No motion interrupt
    @n - INT1_2_SIG_MOTION:   Significant motion interrupt
    @n - INT1_2_STEP_COUNTER: Step counter interrupt
    @n - INT1_2_FLAT:         Flat interrupt
    @n - INT1_2_ORIENTATION:  Orientation interrupt
    @n - INT1_2_TAP:          Tap interrupt
    @n - INT1_2_TILT:         Tilt interrupt
    @n INT3 supported interrupt types:
    @n - INT3_DISABLE:    Disable interrupt
    @n - INT3_DATA_READY: Data ready interrupt
    @n INT4 supported interrupt types:
    @n - INT4_DISABLE:      Disable interrupt
    @n - INT4_DATA_READY:   Data ready interrupt
    @n - INT4_PRESSURE_OOR: Pressure out-of-range interrupt
    @return True if configuration successful, False otherwise
  '''
  def set_int(self, pin, int_type):

  '''!
    @fn get_int_status
    @brief Read interrupt status (unified API)
    @param pin Interrupt pin
    @return int Interrupt status
    @n Can be bitwise ANDed with corresponding interrupt status constants
    @n INT1/INT2 status constants: INT1_2_INT_STATUS_*
    @n INT3 status constants: INT3_INT_STATUS_*
    @n INT4 status constants: INT4_INT_STATUS_*
    @retval 0 No interrupt or read failed
  '''
  def get_int_status(self, pin):

  '''!
    @fn get_step_count
    @brief Read step counter data
    @return int Cumulative step count (32-bit)
    @retval 0 No step data or read failed
  '''
  def get_step_count(self):

  '''!
    @fn get_tap
    @brief Read tap data
    @return int Tap type
    @n Return values:
    @n - TAP_TYPE_SINGLE: Single tap
    @n - TAP_TYPE_DOUBLE: Double tap
    @n - TAP_TYPE_TRIPLE: Triple tap
    @retval 0 No tap data or read failed
  '''
  def get_tap(self):

  '''!
    @fn get_orientation
    @brief Read orientation data
    @return int Orientation data
    @n High byte: Orientation type
    @n - ORIENT_TYPE_PORTRAIT_UP:     Portrait upright
    @n - ORIENT_TYPE_LANDSCAPE_LEFT:  Landscape left
    @n - ORIENT_TYPE_LANDSCAPE_RIGHT: Landscape right
    @n - ORIENT_TYPE_PORTRAIT_DOWN:   Portrait upside down
    @n Low byte: Face direction
    @n - ORIENT_FACE_UP:   Face forward
    @n - ORIENT_FACE_DOWN: Face backward
    @retval 0 No orientation data or read failed
  '''
  def get_orientation(self):
```

## Compatibility

- RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

- Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |           |            |    √     |         |
| Python3 |     √     |            |          |         |

## History

- Date 2026-01-16
- Version V1.0.0

## Credits

Written by (Martin@dfrobot.com), 2026. (Welcome to our [website](https://www.dfrobot.com/))
