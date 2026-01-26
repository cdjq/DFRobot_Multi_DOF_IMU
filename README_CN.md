# DFRobot_Multi_DOF_IMU
- [English Version](./README.md)

DFRobot_Multi_DOF_IMU 是一款多轴IMU传感器库，支持通过I2C/UART接口读取传感器数据。该库支持6轴（加速度计+陀螺仪）、9轴（+磁力计）和10轴（+气压计）传感器，提供全面的运动检测和姿态感知功能。

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
  <img src="./resources/images/SEN0692(1).png" width="45%">
  <img src="./resources/images/SEN0692(2).png" width="45%">
</p>


### SEN0694

<p align="center">
  <img src="./resources/images/SEN0694(1).png" width="45%">
  <img src="./resources/images/SEN0694(2).png" width="48%">
</p>


### SEN0696

<p align="center">
  <img src="./resources/images/SEN0696(1).png" width="45%">
  <img src="./resources/images/SEN0696(2).png" width="45%">
</p>



## 产品链接 (https://www.dfrobot.com)

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

本 Arduino 库为多轴IMU传感器提供全面的接口。它支持：

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

要使用本库，请先下载库文件，将其粘贴到 \Arduino\libraries 目录，然后打开 examples 文件夹并运行其中的示例程序。

## 方法

```C++
  /**
   * @fn begin
   * @brief 初始化UART通信和传感器
   * @return bool
   * @retval true  初始化成功
   * @retval false 初始化失败
   */
  bool begin(void);

  /**
   * @fn setSensorMode
   * @brief 设置传感器工作模式
   * @param mode 传感器工作模式（参见 eSensorMode_t）
   * @n 可用模式：
   * @n - eSleepMode:           睡眠模式（最低功耗，传感器停止工作）
   * @n - eLowPowerMode:        低功耗模式（降低采样率，节省功耗）
   * @n - eNormalMode:          正常模式（平衡功耗和性能）
   * @n - eHighPerformanceMode: 高性能模式（最高采样率和精度，功耗最高）
   * @return bool
   * @retval true  设置成功
   * @retval false 设置失败
   */
  bool setSensorMode(eSensorMode_t mode);

  /**
   * @fn reset
   * @brief 恢复出厂设置
   * @return bool
   * @retval true  恢复出厂设置成功
   * @retval false 恢复出厂设置失败
   */
  bool reset(void);

  /**
   * @fn setAccelRange
   * @brief 设置加速度计量程
   * @param range 加速度计量程（参见 eAccelRange_t）
   * @n 可用量程：
   * @n - eAccelRange2G:  ±2g 量程
   * @n - eAccelRange4G:  ±4g 量程
   * @n - eAccelRange8G:  ±8g 量程
   * @n - eAccelRange16G: ±16g 量程
   * @return bool
   * @retval true  设置成功
   * @retval false 设置失败
   */
  bool setAccelRange(eAccelRange_t range);

  /**
   * @fn setGyroRange
   * @brief 设置陀螺仪量程
   * @param range 陀螺仪量程（参见 eGyroRange_t）
   * @n 可用量程：
   * @n - eGyroRange125DPS:  ±125dps 量程
   * @n - eGyroRange250DPS:  ±250dps 量程
   * @n - eGyroRange500DPS:  ±500dps 量程
   * @n - eGyroRange1000DPS: ±1000dps 量程
   * @n - eGyroRange2000DPS: ±2000dps 量程
   * @return bool
   * @retval true  设置成功
   * @retval false 设置失败
   */
  bool setGyroRange(eGyroRange_t range);

  /**
   * @fn get6dofData
   * @brief 读取6轴IMU数据（物理量）
   * @param accel 指向sSensorData_t结构的指针，用于存储加速度数据（单位：g）
   * @param gyro 指向sSensorData_t结构的指针，用于存储陀螺仪数据（单位：dps）
   * @return bool
   * @retval true  读取成功
   * @retval false 读取失败
   */
  bool get6dofData(sSensorData_t *accel, sSensorData_t *gyro);

  /**
   * @fn get9dofData
   * @brief 读取9轴IMU数据（物理量）
   * @param accel 指向sSensorData_t结构的指针，用于存储加速度数据（单位：g）
   * @param gyro 指向sSensorData_t结构的指针，用于存储陀螺仪数据（单位：dps）
   * @param mag 指向sSensorData_t结构的指针，用于存储磁力计数据（单位：uT）
   * @return bool
   * @retval true  读取成功
   * @retval false 读取失败
   */
  bool get9dofData(sSensorData_t *accel, sSensorData_t *gyro, sSensorData_t *mag);

  /**
   * @fn get10dofData
   * @brief 读取10轴IMU数据（物理量）
   * @param accel 指向sSensorData_t结构的指针，用于存储加速度数据（单位：g）
   * @param gyro 指向sSensorData_t结构的指针，用于存储陀螺仪数据（单位：dps）
   * @param mag 指向sSensorData_t结构的指针，用于存储磁力计数据（单位：uT）
   * @param pressure 指向float类型的指针，用于存储气压或海拔高度数据
   * @param calcAltitude 是否计算海拔高度，默认为false
   * @n true: pressure存储海拔高度（单位：m）
   * @n false: pressure存储气压数据（单位：Pa）
   * @return bool
   * @retval true  读取成功
   * @retval false 读取失败
   */
  bool get10dofData(sSensorData_t *accel, sSensorData_t *gyro, sSensorData_t *mag, float *pressure, bool calcAltitude = false);

  /**
   * @fn calibratePress
   * @brief 根据当地海拔高度校准气压数据
   * @param altitude 当地海拔高度（单位：m）
   * @n 例如：540.0 表示海拔540米
   * @n 调用此函数后，get10dofData中的气压数据将被校准，消除绝对误差
   * @return bool
   * @retval true  校准成功（海拔高度 > 0）
   * @retval false 校准失败（海拔高度 <= 0）
   */
  bool calibratePress(float altitude);

  /**
   * @fn setPressOOR
   * @brief 配置气压超出范围(OOR)参数
   * @param threshold 气压阈值（单位：Pa）
   * @param range 允许范围（单位：Pa）
   * @n 实际允许范围：threshold - range ~ threshold + range
   * @param countLimit 计数限制（参见 ePressOORCountLimit_t）
   * @n 连续N次超出范围后才触发中断，用于滤波避免误触发
   * @n 可用值：
   * @n - ePressOORCountLimit1:  连续1次超出范围后触发
   * @n - ePressOORCountLimit3:  连续3次超出范围后触发
   * @n - ePressOORCountLimit7:  连续7次超出范围后触发
   * @n - ePressOORCountLimit15: 连续15次超出范围后触发
   * @return bool
   * @retval true  配置成功
   * @retval false 配置失败
   */
  bool setPressOOR(uint32_t threshold, uint8_t range, ePressOORCountLimit_t countLimit);

  /**
   * @fn setInt
   * @brief 配置中断（统一API）
   * @param pin 中断引脚（参见 eImuIntPin_t）
   * @n 可用引脚：
   * @n - eImuIntPin1: INT1引脚（6轴传感器，支持多种中断类型）
   * @n - eImuIntPin2: INT2引脚（6轴传感器，支持多种中断类型）
   * @n - eImuIntPin3: INT3引脚（9轴传感器-磁力计，仅支持数据就绪中断）
   * @n - eImuIntPin4: INT4引脚（10轴传感器-气压计，支持数据就绪和气压OOR中断）
   * @param intType 中断类型（uint8_t）
   * @n INT1/INT2支持的中断类型（eInt1_2Type_t）：
   * @n - eInt1_2Disable (0x00): 禁用中断
   * @n - eInt1_2DataReady (0x01): 数据就绪中断
   * @n - eInt1_2AnyMotion (0x02): 任意运动中断
   * @n - eInt1_2NoMotion (0x03): 无运动中断
   * @n - eInt1_2SigMotion (0x04): 显著运动中断
   * @n - eInt1_2StepCounter (0x05): 步进计数中断
   * @n - eInt1_2Flat (0x06): 平面中断
   * @n - eInt1_2Orientation (0x07): 方向中断
   * @n - eInt1_2Tap (0x08): 敲击中断
   * @n - eInt1_2Tilt (0x09): 倾斜中断
   * @n INT3支持的中断类型（eInt3Type_t）：
   * @n - eInt3Disable (0x00): 禁用中断
   * @n - eInt3DataReady (0x01): 数据就绪中断
   * @n INT4支持的中断类型（eInt4Type_t）：
   * @n - eInt4Disable (0x00): 禁用中断
   * @n - eInt4DataReady (0x01): 数据就绪中断
   * @n - eInt4PressureOOR (0x02): 气压超出范围中断
   * @return bool
   * @retval true  配置成功
   * @retval false 配置失败
   */
  bool setInt(eImuIntPin_t pin, uint8_t intType);

  /**
   * @fn getIntStatus
   * @brief 读取中断状态（统一API）
   * @param pin 中断引脚（参见 eImuIntPin_t）
   * @return uint16_t 中断状态
   * @n 可与相应的中断状态宏进行按位与操作来判断中断类型
   * @retval 0 无中断或读取失败
   */
  uint16_t getIntStatus(eImuIntPin_t pin);

  /**
   * @fn getStepCount
   * @brief 读取步数计数器数据
   * @details 读取当前累计步数
   * @n 检测到步进中断后，调用此函数读取累计步数
   * @return uint32_t 累计步数（32位）
   * @retval 0 无步数数据或读取失败
   */
  uint32_t getStepCount(void);

  /**
   * @fn getTap
   * @brief 读取敲击数据
   * @details 当检测到敲击中断后，调用此函数读取具体的敲击类型
   * @return uint16_t 敲击数据
   * @n 返回值：
   * @n - TAP_TYPE_SINGLE (0x0001): 单击
   * @n - TAP_TYPE_DOUBLE (0x0002): 双击
   * @n - TAP_TYPE_TRIPLE (0x0003): 三击
   * @retval 0 无敲击数据或读取失败
   */
  uint16_t getTap(void);

  /**
   * @fn getOrientation
   * @brief 读取方向数据
   * @details 当检测到方向中断后，调用此函数读取具体的方向和面部朝向
   * @return uint16_t 方向数据
   * @n 高字节：方向类型
   * @n - ORIENT_TYPE_PORTRAIT_UP (0x01): 竖屏正向
   * @n - ORIENT_TYPE_LANDSCAPE_LEFT (0x02): 横屏向左
   * @n - ORIENT_TYPE_LANDSCAPE_RIGHT (0x03): 横屏向右
   * @n - ORIENT_TYPE_PORTRAIT_DOWN (0x04): 竖屏倒置
   * @n 低字节：朝向类型
   * @n - ORIENT_FACE_UP (0x00): 面向前
   * @n - ORIENT_FACE_DOWN (0x01): 面向后
   * @retval 0 无方向数据或读取失败
   */
  uint16_t getOrientation(void);
```



## 兼容性

| MCU                | 运行良好 | 运行异常 | 未测试 | 备注 |
| ------------------ | :-----: | :------: | :----: | ---- |
| Arduino uno        |    √    |          |        |      |
| ESP32-P4         |    √     |          |        |      |
| FireBeetle esp32   |    √     |          |        |      |
| FireBeetle esp8266 |    √     |          |        |      |
| FireBeetle m0      |    √     |          |        |      |
| Leonardo           |    √     |          |        |      |
| Microbit           |    √     |          |        |      |
| Arduino MEGA2560 | √ | | | |

## 历史

- Date 2026-01-16
- Version V1.0.0

## 创作者

Written by(Martin@dfrobot.com), 2026. (Welcome to our [website](https://www.dfrobot.com/))
