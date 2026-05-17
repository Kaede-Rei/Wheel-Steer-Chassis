# IMU

- `imu.h / imu.c`
  提供统一入口单例和通用接口：`init / update / get_acc / get_gyro / get_angle`
- `BMI088/bmi088.h / BMI088/bmi088.c`
  提供 BMI088 的具体实现，并注册为一个 `ImuInterface` 实例

## 目录结构

```text
src/device/imu
├── imu.h
├── imu.c
├── bmi088.h
└── bmi088.c
```

## 统一接口

```c
#include "imu/imu.h"
#include "imu/bmi088.h"
```

### 1. 装配实例

```c
imu_set_instance(&bmi088_instance);
imu_init();
```

### 2. 注册中断 / DMA 回调

```c
exti_register_callback(ACC_INT_Pin, bmi088_exti_callback);
exti_register_callback(GYRO_INT_Pin, bmi088_exti_callback);
spi_register_txrx_complete_callback(&hspi2, bmi088_spi_txrx_cplt_callback);
spi_register_error_callback(&hspi2, bmi088_spi_error_callback);
```

### 3. 主循环更新并读取

```c
imu_update();

ImuAcc acc = imu_get_acc();
ImuGyro gyro = imu_get_gyro();
ImuAngle angle = imu_get_angle();
float temp = bmi088_get_temp();
```

## 接口说明

- `imu_init()`
  初始化当前绑定的 IMU 实例
- `imu_update()`
  驱动当前实例推进一次采样 / DMA 状态机并刷新缓存
- `imu_get_acc()`
  返回最近一次缓存的三轴加速度 `ImuAcc`
- `imu_get_gyro()`
  返回最近一次缓存的三轴角速度 `ImuGyro`
- `imu_get_angle()`
  返回最近一次缓存的姿态角 `ImuAngle`

## BMI088 说明

- `bmi088.c` 已经自包含了 BMI088 寄存器定义、SPI 读写、DMA 轮询和回调逻辑
- 当前默认使用 `SPI2`，由 `bmi088.c` 内部的 `BMI088_USING_SPI_UNIT` 绑定到 `hspi2`
- `roll / pitch` 由加速度解算并做简单互补融合
- `yaw` 由陀螺仪角速度积分得到

## 备注

- 当前温度读取仍然是阻塞读取，因为频率要求通常不高
- 如果后续要接入新的 IMU，只需要新增一个新的具体 `xxx_instance`，并实现同样的 `ImuInterface`
