# Mecanum Car — 具身智能小车

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-yellow)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Platform: Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red)](https://www.raspberrypi.com/)

自然语言控制的麦克纳姆轮智能小车。你说 "找绿色瓶子"，它自己旋转搜索、视觉识别、追踪靠近、停下来报告。

---

## 架构

```
[笔记本 — 上位机]                    [树莓派 4B — 下位机]
                                    
 agent_node (LLM + 状态机)           camera_node (USB 摄像头)
     │ 自然语言 → Mimo VLM              │ /camera/image_raw
     │ 工具调用 → 运动控制               │
     │                                  │
 vision_node (帧缓存)                  car_controller (麦克纳姆轮运动学)
     │                                  │ /cmd_vel → PCA9685 PWM
     │                                  │
 cmd_vel_mux (优先级仲裁)               ir_sensor (红外安全层)
     │                                  │
     └── /cmd_vel ──── WiFi/DDS ────▶
```

**关键设计**:
- **图像用 HTTP 传** (RPi camera_http_server → 笔记本直接取 JPEG，绕过 DDS 大包问题)
- **指令用 DDS 传** (/cmd_vel 走 ROS2 DDS，小包低延迟)
- **所有 AI 跑在笔记本** (Mimo VLM 看图和决策，树莓派只做相机采集和电机控制)

## 仓库

| 仓库 | 运行位置 | 说明 |
|------|---------|------|
| [mecanum_car](https://github.com/MEKO747/mecanum_car) | 树莓派 | 电机驱动、相机采集、红外传感器 |
| [embodied_agent_robot](https://github.com/MEKO747/embodied_agent_robot) | 笔记本 | LLM Agent、视觉伺服、工具调用 |

## 硬件

| 组件 | 型号 |
|------|------|
| 上位机 | Ubuntu 24.04 笔记本, NVIDIA RTX 4060, 14GB RAM |
| 下位机 | Raspberry Pi 4B (2GB), ROS2 Jazzy |
| 底盘 | 4WD 麦克纳姆轮 |
| 电机驱动 | PCA9685 (I2C 0x40) |
| 摄像头 | USB UVC 摄像头 (640×480@30fps) |
| 红外传感器 | GPIO 12 (右), GPIO 16 (左) |
| 网络 | WiFi 同子网 |

## 安装

### 树莓派

```bash
# 依赖
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport

# 克隆并构建
mkdir -p ~/ros2_car_ws/src
cd ~/ros2_car_ws/src
git clone https://github.com/MEKO747/mecanum_car.git
cd ~/ros2_car_ws
colcon build --packages-select mecanum_car
```

### 笔记本

```bash
# ROS2 依赖
sudo apt install ros-jazzy-rclpy ros-jazzy-cv-bridge ros-jazzy-image-transport

# 克隆并构建
mkdir -p ~/embodied_agent_ws/src
cd ~/embodied_agent_ws/src
git clone https://github.com/MEKO747/embodied_agent_robot.git embodied_agent
cd ~/embodied_agent_ws
colcon build --packages-select embodied_agent
```

## 配置

### 1. Mimo API Key

```bash
export MIMO_API_KEY=sk-your-key-here
```

### 2. 网络

两机必须在同一 WiFi 子网。设置 ROS_IP：

```bash
# 笔记本 (假设 IP 10.246.251.103)
export ROS_IP=10.246.251.103
export ROS_DOMAIN_ID=42

# 树莓派 (假设 IP 10.246.251.19)
export ROS_IP=10.246.251.19
export ROS_DOMAIN_ID=42
```

### 3. 相机 HTTP 服务器 (树莓派)

```bash
# 在 RPi 上运行, 让笔记本通过 HTTP 直接取 JPEG 帧
python3 camera_http_server.py &
```

## 运行

### 启动下位机 (树莓派)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_car_ws/install/setup.bash
export ROS_IP=10.246.251.19
export ROS_DOMAIN_ID=42
ros2 launch mecanum_car full_robot.launch.py
```

### 启动上位机 (笔记本)

```bash
source /opt/ros/jazzy/setup.bash
source ~/embodied_agent_ws/install/setup.bash
export ROS_IP=10.246.251.103
export ROS_DOMAIN_ID=42
export MIMO_API_KEY=sk-your-key-here
ros2 launch embodied_agent agent_system.launch.py
```

### 自然语言控制

```bash
python3 ~/embodied_agent_ws/src/embodied_agent/embodied_agent/agent_cli.py
```

```
🗣  > 前进
🗣  > turn left
🗣  > 找绿色瓶子
🗣  > find a red cup
🗣  > stop
```

## 工作流程 ("找绿色瓶子")

```
用户: "找绿色瓶子"
  ↓
agent_node → Mimo VLM 解析 → TOOL_CALL: search_object(bottle, green)
  ↓
SEARCHING 状态: 分段旋转，每段停顿拍一帧
  ↓
Mimo 看图: "有。左。中" (found, left side, medium size)
  ↓
FOLLOWING 状态: 旋转对齐 → 前进靠近 → 再检查 → 再对齐 → ...
  ↓
Mimo: "有。中。大" (centered, close)
  ↓
停止 → 报告: "Found green bottle"
```

## 工具

| 工具 | 参数 | 触发示例 |
|------|------|---------|
| `move_forward` | distance (0.1-2.0m) | "前进" / "go forward" |
| `move_backward` | distance | "后退" |
| `rotate_left` | degrees (1-360) | "左转" / "turn left" |
| `rotate_right` | degrees | "右转" |
| `stop_robot` | — | "停" / "stop" |
| `search_object` | object_name, color | "找绿色瓶子" / "find a red cup" |
| `follow_object` | object_name | "跟着那个杯子" |

支持语言: 中文 / English / Français / 日本語 / 任何 Mimo 理解的语言。

## 节点

### 笔记本侧

| 节点 | 说明 |
|------|------|
| `agent_node` | LLM + 状态机 + 工具调用 + 视觉伺服 (20Hz) |
| `vision_node` | 轻量帧缓存 + 调试图像发布 |
| `cmd_vel_mux` | /cmd_vel 优先级仲裁 (teleop > agent > avoid) |
| `agent_cli` | 自然语言交互终端 |

### 树莓派侧

| 节点 | 说明 |
|------|------|
| `camera_node` | USB 相机 → /camera/image_raw |
| `car_controller` | /cmd_vel → 麦克纳姆轮运动学 → PCA9685 |
| `ir_sensor` | GPIO 红外传感器 → /ir/status |
| `camera_http_server` | HTTP JPEG 服务器 (8080 端口, 给 Mimo 用) |

## 参数

关键参数在 `config/agent_params.yaml`:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `llm_model` | mimo-v2.5 | LLM 模型 |
| `llm_base_url` | https://api.xiaomimimo.com/v1 | API 端点 |
| `search_angular_speed` | 0.4 | 搜索旋转速度 rad/s |
| `search_full_scan_timeout` | 45.0 | 搜索总超时 |
| `servo_max_turn_speed` | 0.4 | 跟随最大转向速度 |
| `servo_max_approach_speed` | 0.25 | 跟随最大前进速度 |
| `rotate_duration_per_90deg` | 2.4 | 开环旋转 90° 耗时 (s) — 需实测校准 |

## License

MIT
