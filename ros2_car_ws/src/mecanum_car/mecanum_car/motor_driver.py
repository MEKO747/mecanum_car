"""
PCA9685 电机驱动（匹配原版 LOBOROBOT.py 硬件配置）

PCA9685 通道分配:
  Motor 0 (前左): PWM=ch0,  IN1=ch2,  IN2=ch1
  Motor 1 (前右): PWM=ch5,  IN1=ch3,  IN2=ch4
  Motor 2 (后左): PWM=ch6,  IN1=ch8,  IN2=ch7
  Motor 3 (后右): PWM=ch11, IN1=GPIO24, IN2=GPIO25

方向逻辑:
  Motor 0: forward={IN1=0, IN2=1}, backward={IN1=1, IN2=0}
  Motor 1: forward={IN1=1, IN2=0}, backward={IN1=0, IN2=1}  (反相)
  Motor 2: forward={IN1=1, IN2=0}, backward={IN1=0, IN2=1}  (反相)
  Motor 3: forward={IN1=0, IN2=1}, backward={IN1=1, IN2=0}  (GPIO)
"""

import smbus2
import lgpio
import time
import math


# PCA9685 寄存器
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALLLED_ON_L = 0xFA
ALLLED_OFF_L = 0xFC

LED_CHANNEL_OFFSET = 4


class PCA9685:
    """PCA9685 16通道 12-bit PWM 控制器"""

    def __init__(self, address=0x40, bus=1, frequency=50):
        self.address = address
        self.bus = smbus2.SMBus(bus)
        self._init_device(frequency)

    def _init_device(self, freq):
        self._write_byte(MODE1, 0x00)
        self.set_frequency(freq)

    def _write_byte(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def _read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_frequency(self, freq):
        """设置 PWM 频率"""
        freq = max(24.0, min(1526.0, freq))
        prescale = int(round(25000000.0 / (4096.0 * freq)) - 1)
        old_mode = self._read_byte(MODE1)
        self._write_byte(MODE1, (old_mode & 0x7F) | 0x10)
        self._write_byte(PRESCALE, prescale)
        self._write_byte(MODE1, old_mode)
        time.sleep(0.005)
        self._write_byte(MODE1, old_mode | 0x80)

    def set_pwm(self, channel, on, off):
        """设置单个通道 PWM"""
        reg = LED0_ON_L + LED_CHANNEL_OFFSET * channel
        self._write_byte(reg, on & 0xFF)
        self._write_byte(reg + 1, on >> 8)
        self._write_byte(reg + 2, off & 0xFF)
        self._write_byte(reg + 3, off >> 8)

    def set_duty(self, channel, pulse):
        """
        设置占空比
        pulse: 0-100（百分比，匹配原版 setDutycycle）
        """
        pulse = max(0, min(100, pulse))
        value = min(4095, int(pulse * (4096 / 100)))
        self.set_pwm(channel, 0, value)

    def set_level(self, channel, value):
        """设置通道输出 HIGH(1) 或 LOW(0)"""
        if value == 1:
            self.set_pwm(channel, 0, 4095)
        else:
            self.set_pwm(channel, 0, 0)

    def stop_all(self):
        # 临时开启 ALLCALL 以保证 ALLLED 寄存器生效
        old_mode = self._read_byte(MODE1)
        self._write_byte(MODE1, old_mode | 0x01)
        self._write_byte(ALLLED_ON_L, 0)
        self._write_byte(ALLLED_OFF_L, 0)
        self._write_byte(MODE1, old_mode)


class MotorDriver:
    """四路麦克纳姆轮电机驱动（匹配原版 LOBOROBOT 硬件）"""

    # 原版硬件配置
    # motor_0 = 前左 (Front-Left),  motor_1 = 前右 (Front-Right)
    # motor_2 = 后左 (Rear-Left),    motor_3 = 后右 (Rear-Right)
    #
    # 每电机: (pwm_ch, in1_ch, in2_ch, forward_in1_level, forward_in2_level)
    # motor_3 的 in1/in2 是 GPIO 引脚，其他都是 PCA9685 通道
    MOTOR_CONFIG = [
        # pwm, in1, in2,  fwd_in1, fwd_in2
        (0,   2,   1,   0, 1),   # Motor 0: 前左
        (5,   3,   4,   1, 0),   # Motor 1: 前右（反相）
        (6,   8,   7,   1, 0),   # Motor 2: 后左（反相）
        (11,  25,  24,  0, 1),   # Motor 3: 后右（GPIO 25/24，对齐原版 DIN1=25, DIN2=24）
    ]

    def __init__(self, address=0x40):
        self.pca = PCA9685(address=address, frequency=50)
        self.gpio = lgpio.gpiochip_open(0)

        self._motors = []
        for i, (pwm_ch, in1, in2, fwd_in1, fwd_in2) in enumerate(self.MOTOR_CONFIG):
            is_gpio = (i == 3)  # 只有 motor_3 用 GPIO
            self._motors.append({
                'pwm_ch': pwm_ch,
                'in1': in1,
                'in2': in2,
                'fwd_in1': fwd_in1,
                'fwd_in2': fwd_in2,
                'is_gpio': is_gpio,
            })
            if is_gpio:
                lgpio.gpio_claim_output(self.gpio, in1, 0)
                lgpio.gpio_claim_output(self.gpio, in2, 0)

        # 清除上一次进程退出后 PCA9685 可能残留的 PWM/方向输出。
        self.stop_all()

    def set_motor(self, index, speed):
        """
        设置单个电机
        index: 0-3
        speed: -1.0 ~ 1.0（正=前进，负=后退，0=停止）
        """
        speed = max(-1.0, min(1.0, speed))
        m = self._motors[index]
        duty = abs(speed) * 100.0  # 转换为 0-100

        if speed > 0:
            # 前进
            in1_val = m['fwd_in1']
            in2_val = m['fwd_in2']
        elif speed < 0:
            # 后退
            in1_val = m['fwd_in2']
            in2_val = m['fwd_in1']
        else:
            # 停止
            self.pca.set_duty(m['pwm_ch'], 0)
            if m['is_gpio']:
                lgpio.gpio_write(self.gpio, m['in1'], 0)
                lgpio.gpio_write(self.gpio, m['in2'], 0)
            else:
                self.pca.set_level(m['in1'], 0)
                self.pca.set_level(m['in2'], 0)
            return

        # 设置方向
        if m['is_gpio']:
            lgpio.gpio_write(self.gpio, m['in1'], in1_val)
            lgpio.gpio_write(self.gpio, m['in2'], in2_val)
        else:
            self.pca.set_level(m['in1'], in1_val)
            self.pca.set_level(m['in2'], in2_val)

        # 设置速度
        self.pca.set_duty(m['pwm_ch'], duty)

    def set_speeds(self, speeds):
        """设置四个电机速度 [fl, fr, rl, rr] 每个 -1.0 ~ 1.0"""
        for i, speed in enumerate(speeds):
            self.set_motor(i, speed)

    def stop_all(self):
        for i in range(4):
            self.set_motor(i, 0)

    def cleanup(self):
        self.stop_all()
        # 不调用 pca.stop_all() — 它会通过 ALLLED 寄存器清零所有通道，
        # 连舵机（ch9/ch10）一起干掉，导致摄像头甩到极限位置
        lgpio.gpiochip_close(self.gpio)
