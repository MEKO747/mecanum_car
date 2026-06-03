# Mecanum Car — ROS2 麦克纳姆轮智能小车

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi-red?logo=raspberrypi)](https://www.raspberrypi.com/)

基于 ROS2 的麦克纳姆轮智能小车控制系统，运行在树莓派上，通过 PCA9685 驱动四路电机。支持红外避障、摄像头辅助避障和键盘遥控三种模式。

## 硬件架构

```
┌─────────────────────────────────────────────────┐
│                  Raspberry Pi                    │
│                                                  │
│  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │
│  │ IR Sensor│  │ USB      │  │ PCA9685 (I2C) │  │
│  │ GPIO 12  │  │ Camera   │  │ 0x40          │  │
│  │ GPIO 16  │  │          │  │               │  │
│  └────┬─────┘  └────┬─────┘  └───────┬───────┘  │
│       │             │                │           │
│       ▼             ▼                ▼           │
│  /ir/status    图像分析          PWM 输出          │
│  (Int32)       (OpenCV)     ┌───┬───┬───┬───┐    │
│                             │ M0│ M1│ M2│ M3│    │
│                             └───┴───┴───┴───┘    │
│                             FL  FR  RL  RR       │
└─────────────────────────────────────────────────┘
```

| 硬件 | 型号/规格 | 用途 |
|------|-----------|------|
| 主控 | Raspberry Pi (带 I2C + GPIO) | 运行 ROS2 节点 |
| 电机驱动 | PCA9685 (I2C 0x40) | 16通道 PWM 驱动 |
| 底盘 | 麦克纳姆轮 4WD | 全向移动 |
| 红外传感器 ×2 | GPIO 12 (右), GPIO 16 (左) | 障碍物检测 (active-low) |
| USB 摄像头 | 任意 UVC 摄像头 | 视觉避障分析 |
| 舵机云台 ×2 | PCA9685 ch9 (tilt), ch10 (pan) | 摄像头指向控制 |

### PCA9685 通道分配

| 通道 | 功能 |
|------|------|
| ch0 | Motor 0 (前左) PWM |
| ch1-ch2 | Motor 0 IN2/IN1 |
| ch3-ch4 | Motor 1 (前右) IN1/IN2 |
| ch5 | Motor 1 PWM |
| ch6 | Motor 2 (后左) PWM |
| ch7-ch8 | Motor 2 IN2/IN1 |
| ch9 | Tilt 舵机 (上下) |
| ch10 | Pan 舵机 (左右) |
| ch11 | Motor 3 (后右) PWM |
| GPIO 24-25 | Motor 3 IN1/IN2 |

## 节点架构

```
                  ┌─────────────────┐
                  │  mecanum_keyboard│  (键盘遥控)
                  │  /cmd_vel (Twist)│
                  └────────┬────────┘
                           │
  ┌───────────┐     ┌──────▼──────┐     ┌───────────────┐
  │ ir_sensor │     │car_controller│     │camera_avoidance│
  │ GPIO 12,16│────▶│ 麦克纳姆运动学 │◀────│ 摄像头+红外决策 │
  │/ir/status │     │ PCA9685 驱动 │     │ /cmd_vel 发布  │
  └─────┬─────┘     └──────┬──────┘     └───────┬───────┘
        │                  │                     │
        ▼                  ▼                     ▼
  ir_avoidance        Motor 0-3            /cmd_vel
  (纯红外状态机)       (PWM 输出)          (覆盖手动/自动)
```

### 节点说明

| 节点 | 可执行文件 | 功能 |
|------|-----------|------|
| `ir_sensor` | `ir_sensor` | 读取 GPIO 红外传感器，发布 `/ir/status` (Int32, bit0=左, bit1=右) |
| `car_controller` | `car_controller` | 订阅 `/cmd_vel`，麦克纳姆轮运动学解算，驱动 PCA9685 电机 |
| `ir_avoidance` | `ir_avoidance` | 订阅 `/ir/status`，非阻塞状态机避障，发布 `/cmd_vel` |
| `camera_avoidance` | `camera_avoidance` | 红外触发→拍照分析左右空间→横向平移绕行，发布 `/cmd_vel` |
| `mecanum_keyboard` | `mecanum_keyboard` | 终端键盘遥控，发布 `/cmd_vel` |
| (工具) | `servo_calibrate` | 云台舵机 PWM 校准工具 (standalone) |

### 状态机 (IR 避障)

```
         ┌──────────┐
         │ STOPPED  │◀──────── 看门狗超时 / 初始状态
         └────┬─────┘
              │ 传感器数据到达
              ▼
    ┌──────────────┐    有障碍    ┌──────────┐
    │  FORWARD     │──────────▶│ BACK_UP  │
    │  前进        │            │  后退    │
    └──────┬───────┘            └────┬─────┘
           │                         │ 后退计时到
           │                         ▼
           │               ┌──────────────────┐
           │               │ TURN_LEFT/RIGHT  │
           │               │ 或 TURN_AWAY     │
           │               │ 转向避开          │
           │               └────────┬─────────┘
           │                        │ 传感器清除
           └────────────────────────┘
```

### 状态机 (摄像头避障)

```
STOPPED → FORWARD → (红外触发) → BACK_UP → CAPTURE (异步拍照分析)
                                                    │
                                            ┌───────┼───────┐
                                            ▼       ▼       ▼
                                     AVOID_LEFT AVOID_RIGHT AVOID_DEFAULT
                                       (左平移)   (右平移)   (默认方向)
                                            │       │       │
                                            └───────┴───────┘
                                                    │ 超时重试 (max N次)
                                                    ▼
                                               ROTATE (旋转脱困)
                                                    │
                                                    ▼
                                               FORWARD
```

## 依赖

```bash
# ROS2 (Humble 或更高)
sudo apt install ros-humble-ros-base

# Python 依赖
sudo apt install python3-opencv python3-numpy python3-smbus2 python3-lgpio
```

## 安装

```bash
# 1. 克隆仓库
cd ~/ros2_car_ws/src
git clone https://github.com/MEKO747/mecanum_car.git

# 2. 编译
cd ~/ros2_car_ws
colcon build --packages-select mecanum_car

# 3. 加载环境
source install/setup.bash
```

## 使用方法

### 键盘遥控

终端键盘直接控制小车移动：

```bash
ros2 launch mecanum_car keyboard.launch.py
```

```
    u    i    p       前进/后退: i / ,
    |    |    |       左移/右移: j / l
  j ── + ── l        对角移动: u p m .
    |    |    |       左自旋: y  右自旋: o
    m    ,    .       停止:     k
    y         o       加速: q  减速: e
```

### 红外避障

双红外传感器自动避障（无需摄像头）：

```bash
ros2 launch mecanum_car ir_avoidance.launch.py
```

可调参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `forward_speed` | 0.3 | 前进速度 (m/s) |
| `turn_speed` | 0.4 | 转向速度 (rad/s) |
| `backup_speed` | 0.3 | 后退速度 (m/s) |
| `backup_duration` | 0.4 | 后退持续时间 (s) |
| `turn_duration` | 0.5 | 转向持续时间 (s) |

### 摄像头辅助避障

红外触发后使用摄像头分析左右空闲空间，智能选择绕行方向：

```bash
ros2 launch mecanum_car camera_avoidance.launch.py
```

可调参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `forward_speed` | 0.25 | 前进速度 (m/s) |
| `strafe_speed` | 0.3 | 横向平移速度 (m/s) |
| `backup_speed` | 0.2 | 后退速度 (m/s) |
| `decision_threshold` | 0.01 | 左右边缘密度差阈值 |
| `default_direction` | `right` | 无法判断时的默认绕行方向 |
| `max_retries` | 3 | 避障失败最大重试次数 |
| `canny_low` / `canny_high` | 50 / 150 | Canny 边缘检测阈值 |

### 舵机校准

```bash
# 校准 pan 舵机 (左右)
python3 ros2_car_ws/src/mecanum_car/mecanum_car/servo_calibrate.py --channel 10

# 校准 tilt 舵机 (上下)
python3 ros2_car_ws/src/mecanum_car/mecanum_car/servo_calibrate.py --channel 9
```

舵机会自动来回扫描，观察摄像头指向，按 `c`/`l`/`r` 标记对应位置。

### 仅驱动底盘

只启动电机驱动节点，配合自定义上层控制：

```bash
ros2 launch mecanum_car car.launch.py
```

## 参数参考

### car_controller

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `pca9685_address` | int/hex | `0x40` | PCA9685 I2C 地址 |
| `max_linear_speed` | float | `1.0` | 最大线速度 (m/s) |
| `max_angular_speed` | float | `2.0` | 最大角速度 (rad/s) |
| `wheel_base_x` | float | `0.15` | 半轮距 (m) |
| `wheel_base_y` | float | `0.12` | 半轴距 (m) |
| `timeout` | float | `1.0` | 无指令自动停止时间 (s) |
| `deadband` | float | `0.01` | 速度死区阈值 |

### ir_sensor

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `debounce_samples` | int | `3` | 去抖采样次数 |

## Topic 接口

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅/发布 | 速度指令，各节点发布/订阅 |
| `/ir/status` | `std_msgs/Int32` | 发布 | bit0=左传感器触发, bit1=右传感器触发 |

## 项目结构

```
ros2_car_ws/src/mecanum_car/
├── launch/
│   ├── car.launch.py                  # 底盘驱动启动
│   ├── ir_avoidance.launch.py         # 红外避障启动
│   ├── camera_avoidance.launch.py     # 摄像头避障启动
│   └── keyboard.launch.py             # 键盘遥控启动
├── mecanum_car/
│   ├── __init__.py
│   ├── motor_driver.py                # PCA9685 底层驱动 + MotorDriver
│   ├── car_controller.py              # 麦克纳姆轮运动学 + /cmd_vel 订阅
│   ├── ir_sensor.py                   # GPIO 红外传感器读取
│   ├── ir_avoidance.py                # 纯红外避障状态机
│   ├── camera_avoidance.py            # 摄像头辅助避障状态机
│   ├── mecanum_keyboard.py            # 终端键盘遥控
│   └── servo_calibrate.py             # 舵机 PWM 校准工具
├── resource/
│   └── mecanum_car                    # ament 资源标记
├── package.xml
├── setup.py
└── setup.cfg
```

## License

MIT License — 详见 [package.xml](ros2_car_ws/src/mecanum_car/package.xml)
