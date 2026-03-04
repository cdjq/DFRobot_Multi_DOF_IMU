# DFRobot_Multi_DOF_IMU
- [English Version](./README.md)

DFRobot_Multi_DOF_IMU 是一款多轴IMU传感器 Raspberry Pi Python 库，支持通过I2C/UART接口读取传感器数据。该库支持6轴（加速度计+陀螺仪）、9轴（+磁力计）和10轴（+气压计）传感器，提供全面的运动检测和姿态感知功能。

该传感器非常适合可穿戴设备、智能手表、健身追踪器、无人机、机器人导航和物联网应用，这些应用对运动感知、姿态检测和能效要求很高。通过可配置的中断引脚（INT1、INT2、INT3、INT4），传感器可以高效地通知主机系统各种运动事件，非常适合电池供电的应用。

**主要特性：**

- 多轴运动感知（6轴/9轴/10轴）
- 支持I2C和UART（Modbus RTU）两种通信方式
- 硬件计步器，支持中断
- 多种运动检测模式（任意运动、无运动、显著运动）
- 手势识别（敲击、倾斜、方向、平面检测）
- 气压检测和海拔高度计算
- 气压超出范围（OOR）中断
- 多种工作模式（睡眠、低功耗、正常、高性能）
- 可配置加速度计和陀螺仪量程
- 多中断引脚，灵活的事件处理

### SEN0692

<p align="center">
  <img src="../../resources/images/SEN0692(1).png" width="45%">
  <img src="../../resources/images/SEN0692(2).png" width="45%">
</p>



### SEN0694

<p align="center">
  <img src="../../resources/images/SEN0694(1).png" width="45%">
  <img src="../../resources/images/SEN0694(2).png" width="45%">
</p>




### SEN0696

<p align="center">
  <img src="../../resources/images/SEN0696(1).png" width="45%">
  <img src="../../resources/images/SEN0696(2).png" width="45%">
</p>



## 产品链接 (https://www.dfrobot.com.cn)

```
SKU: SEN0692/SEN0694/SEN0696
```

## 目录

  * [概述](#概述)
  * [库安装](#库安装)
  * [方法](#方法)
  * [兼容性](#兼容性)
  * [历史](#历史)
  * [创作者](#创作者)

## 概述

本 Python 库为多轴IMU传感器提供全面的接口。它支持：

**基本功能：**

- 通过I2C或UART接口初始化传感器
- 配置传感器工作模式（睡眠、低功耗、正常、高性能）
- 配置加速度计和陀螺仪量程
- 读取6轴传感器数据（加速度计 + 陀螺仪）
- 读取9轴传感器数据（加速度计 + 陀螺仪 + 磁力计）
- 读取10轴传感器数据（加速度计 + 陀螺仪 + 磁力计 + 气压计）
- 气压校准（根据当地海拔高度）
- 气压超出范围（OOR）检测配置

**高级功能：**

- 计步器，支持中断
- 任意运动检测中断
- 无运动检测中断
- 显著运动检测中断
- 平面检测中断
- 方向检测中断（横竖屏、正反面）
- 敲击检测中断（单击/双击/三击）
- 倾斜检测中断
- 气压数据就绪中断
- 气压超出范围中断

## 库安装

1. 下载库至树莓派，使用命令 `git clone https://github.com/DFRobot/DFRobot_Multi_DOF_IMU`
2. 打开 `python/raspberrypi/examples` 文件夹，运行示例程序

## 方法

```python
  '''!
    @fn begin
    @brief 初始化传感器
    @return True 初始化成功, False 初始化失败
  '''
  def begin(self):

  '''!
    @fn reset
    @brief 恢复出厂设置
    @return True 恢复成功, False 恢复失败
  '''
  def reset(self):

  '''!
    @fn set_sensor_mode
    @brief 设置传感器工作模式
    @param mode 传感器工作模式
    @n 可用模式：
    @n - SLEEP_MODE:            睡眠模式（最低功耗，传感器停止工作）
    @n - LOW_POWER_MODE:        低功耗模式（降低采样率，节省功耗）
    @n - NORMAL_MODE:           正常模式（平衡功耗和性能）
    @n - HIGH_PERFORMANCE_MODE: 高性能模式（最高采样率和精度，功耗最高）
    @return True 设置成功, False 设置失败
  '''
  def set_sensor_mode(self, mode):

  '''!
    @fn set_accel_range
    @brief 设置加速度计量程
    @param accel_range 加速度计量程
    @n 可用量程：
    @n - ACCEL_RANGE_2G:  ±2g 量程
    @n - ACCEL_RANGE_4G:  ±4g 量程
    @n - ACCEL_RANGE_8G:  ±8g 量程
    @n - ACCEL_RANGE_16G: ±16g 量程
    @return True 设置成功, False 设置失败
  '''
  def set_accel_range(self, accel_range):

  '''!
    @fn set_gyro_range
    @brief 设置陀螺仪量程
    @param gyro_range 陀螺仪量程
    @n 可用量程：
    @n - GYRO_RANGE_125DPS:  ±125dps 量程
    @n - GYRO_RANGE_250DPS:  ±250dps 量程
    @n - GYRO_RANGE_500DPS:  ±500dps 量程
    @n - GYRO_RANGE_1000DPS: ±1000dps 量程
    @n - GYRO_RANGE_2000DPS: ±2000dps 量程
    @return True 设置成功, False 设置失败
  '''
  def set_gyro_range(self, gyro_range):

  '''!
    @fn get_6dof_data
    @brief 读取6轴IMU数据（物理量）
    @return dict 包含加速度和陀螺仪数据的字典
    @n 返回格式：{'accel': {'x': float, 'y': float, 'z': float}, 'gyro': {'x': float, 'y': float, 'z': float}}
    @n accel: x, y, z 加速度数据（单位：g）
    @n gyro: x, y, z 陀螺仪数据（单位：dps）
    @retval None 读取失败
  '''
  def get_6dof_data(self):

  '''!
    @fn get_9dof_data
    @brief 读取9轴IMU数据（物理量）
    @return dict 包含加速度、陀螺仪和磁力计数据的字典
    @n 返回格式：{'accel': {...}, 'gyro': {...}, 'mag': {'x': float, 'y': float, 'z': float}}
    @n mag: x, y, z 磁力计数据（单位：uT）
    @retval None 读取失败
  '''
  def get_9dof_data(self):

  '''!
    @fn get_10dof_data
    @brief 读取10轴IMU数据（物理量）
    @param as_altitude 是否计算海拔高度，默认为False
    @n True: pressure存储海拔高度（单位：m）
    @n False: pressure存储气压数据（单位：Pa）
    @return dict 包含加速度、陀螺仪、磁力计和气压/海拔数据的字典
    @n 返回格式：{'accel': {...}, 'gyro': {...}, 'mag': {...}, 'pressure': float}
    @retval None 读取失败
  '''
  def get_10dof_data(self, as_altitude=False):

  '''!
    @fn calibrate_altitude
    @brief 根据当地海拔高度校准海拔数据
    @param altitude 当地海拔高度（单位：m）
    @n 例如：540.0 表示海拔540米
    @n 调用此函数后，get_10dof_data 中请求海拔时（calc_altitude 为 True）将使用该校准消除绝对误差
    @return True 校准成功（海拔高度 > 0）, False 校准失败
  '''
  def calibrate_altitude(self, altitude):

  '''!
    @fn set_press_oor
    @brief 配置气压超出范围(OOR)参数
    @param threshold 气压阈值（单位：Pa）
    @param range_val 允许范围（单位：Pa）
    @n 实际允许范围：threshold - range_val ~ threshold + range_val
    @param count_limit 计数限制
    @n 连续N次超出范围后才触发中断，用于滤波避免误触发
    @n 可用值：
    @n - PRESS_OOR_COUNT_LIMIT_1:  连续1次超出范围后触发
    @n - PRESS_OOR_COUNT_LIMIT_3:  连续3次超出范围后触发
    @n - PRESS_OOR_COUNT_LIMIT_7:  连续7次超出范围后触发
    @n - PRESS_OOR_COUNT_LIMIT_15: 连续15次超出范围后触发
    @return True 配置成功, False 配置失败
  '''
  def set_press_oor(self, threshold, range_val, count_limit):

  '''!
    @fn set_int
    @brief 配置中断
    @param pin 中断引脚
    @n 可用引脚：
    @n - IMU_INT_PIN_INT1: INT1引脚（6轴传感器，支持多种中断类型）
    @n - IMU_INT_PIN_INT2: INT2引脚（6轴传感器，支持多种中断类型）
    @n - IMU_INT_PIN_INT3: INT3引脚（9轴传感器-磁力计，仅支持数据就绪中断）
    @n - IMU_INT_PIN_INT4: INT4引脚（10轴传感器-气压计，支持数据就绪和气压OOR中断）
    @param int_type 中断类型
    @n INT1/INT2支持的中断类型：
    @n - INT1_2_DISABLE:      禁用中断
    @n - INT1_2_DATA_READY:   数据就绪中断
    @n - INT1_2_ANY_MOTION:   任意运动中断
    @n - INT1_2_NO_MOTION:    无运动中断
    @n - INT1_2_SIG_MOTION:   显著运动中断
    @n - INT1_2_STEP_COUNTER: 步进计数中断
    @n - INT1_2_FLAT:         平面中断
    @n - INT1_2_ORIENTATION:  方向中断
    @n - INT1_2_TAP:          敲击中断
    @n - INT1_2_TILT:         倾斜中断
    @n INT3支持的中断类型：
    @n - INT3_DISABLE:    禁用中断
    @n - INT3_DATA_READY: 数据就绪中断
    @n INT4支持的中断类型：
    @n - INT4_DISABLE:      禁用中断
    @n - INT4_DATA_READY:   数据就绪中断
    @n - INT4_PRESSURE_OOR: 气压超出范围中断
    @return True 配置成功, False 配置失败
  '''
  def set_int(self, pin, int_type):

  '''!
    @fn get_int_status
    @brief 读取中断状态
    @param pin 中断引脚
    @return int 中断状态
    @n 可与相应的中断状态常量进行按位与操作来判断中断类型
    @n INT1/INT2状态常量：INT1_2_INT_STATUS_*
    @n INT3状态常量：INT3_INT_STATUS_*
    @n INT4状态常量：INT4_INT_STATUS_*
    @retval 0 无中断或读取失败
  '''
  def get_int_status(self, pin):

  '''!
    @fn get_step_count
    @brief 读取步数计数器数据
    @details 读取当前累计步数
    @n 使能步进计数中断后，可随时调用此函数从计步器读取当前真实步数，
    @n 无需等待或依赖 INT1_2_INT_STATUS_STEP_COUNTER 中断触发
    @return int 累计步数（32位）
    @retval 0 无步数数据或读取失败
  '''
  def get_step_count(self):

  '''!
    @fn get_tap
    @brief 读取敲击数据
    @return int 敲击类型
    @n 返回值：
    @n - TAP_TYPE_SINGLE: 单击
    @n - TAP_TYPE_DOUBLE: 双击
    @n - TAP_TYPE_TRIPLE: 三击
    @retval 0 无敲击数据或读取失败
  '''
  def get_tap(self):

  '''!
    @fn get_orientation
    @brief 读取方向数据
    @return int 方向数据
    @n 高字节：方向类型
    @n - ORIENT_TYPE_PORTRAIT_UP:     竖屏正向
    @n - ORIENT_TYPE_LANDSCAPE_LEFT:  横屏向左
    @n - ORIENT_TYPE_LANDSCAPE_RIGHT: 横屏向右
    @n - ORIENT_TYPE_PORTRAIT_DOWN:   竖屏倒置
    @n 低字节：朝向类型
    @n - ORIENT_FACE_UP:   面向前
    @n - ORIENT_FACE_DOWN: 面向后
    @retval 0 无方向数据或读取失败
  '''
  def get_orientation(self):
```

## 兼容性

- RaspberryPi Version

| 板子型号     | 运行良好 | 运行异常 | 未测试 | 备注 |
| ------------ | :------: | :------: | :----: | ---- |
| RaspberryPi2 |          |          |   √    |      |
| RaspberryPi3 |          |          |   √    |      |
| RaspberryPi4 |    √     |          |        |      |

- Python Version

| Python  | 运行良好 | 运行异常 | 未测试 | 备注 |
| ------- | :------: | :------: | :----: | ---- |
| Python2 |          |          |   √    |      |
| Python3 |    √     |          |        |      |

## 历史

- Date 2026-01-16
- Version V1.0.0

## 创作者

Written by (Martin@dfrobot.com), 2026. (Welcome to our [website](https://www.dfrobot.com.cn/))
