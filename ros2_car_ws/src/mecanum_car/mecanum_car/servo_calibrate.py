"""
舵机 PWM 校准工具（自动扫描版，双通道）
在树莓派上运行，舵机自动来回扫描，用户观察摄像头指向标记位置

用法:
  python3 servo_calibrate.py --channel 10     # 校准底座 pan 舵机（左右转）
  python3 servo_calibrate.py --channel 9      # 校准倾斜 tilt 舵机（上下转）
"""

import argparse
import smbus2
import time
import sys
import threading

PCA_ADDR = 0x40

# ---- 通道 10 = pan（底座/左右） ----
PAN_CENTER_LABEL = 'center'   # 摄像头正前方
PAN_LEFT_LABEL   = 'left'     # 摄像头向左 ~30-45°
PAN_RIGHT_LABEL  = 'right'    # 摄像头向右 ~30-45°

# ---- 通道 9  = tilt（倾斜/上下） ----
TILT_CENTER_LABEL = 'center'  # 摄像头略下倾，看清地面

# 扫描范围
SWEEP_MIN = 100
SWEEP_MAX = 800
SWEEP_STEP = 12       # 每步 PWM 增量（~59µs，超过舵机死区）
SWEEP_DELAY = 0.25    # 每步停留秒数

marks = {}
current_pwm = 0
running = True
bus = None
SERVO_CH = 10
MODE = 'pan'          # 'pan' 或 'tilt'


def set_servo(count: int):
    """设置舵机 PWM 脉冲宽度"""
    global current_pwm
    reg = 0x06 + 4 * SERVO_CH
    bus.write_byte_data(PCA_ADDR, reg, 0)
    bus.write_byte_data(PCA_ADDR, reg + 1, 0)
    bus.write_byte_data(PCA_ADDR, reg + 2, count & 0xFF)
    bus.write_byte_data(PCA_ADDR, reg + 3, count >> 8)
    current_pwm = count


def init_pca9685(freq=50):
    """初始化 PCA9685 频率"""
    MODE1 = 0x00
    PRESCALE = 0xFE
    old_mode = bus.read_byte_data(PCA_ADDR, MODE1)
    bus.write_byte_data(PCA_ADDR, MODE1, (old_mode & 0x7F) | 0x10)  # enter SLEEP
    prescale = int(round(25000000.0 / (4096.0 * freq)) - 1)
    bus.write_byte_data(PCA_ADDR, PRESCALE, prescale)
    bus.write_byte_data(PCA_ADDR, MODE1, old_mode & 0xEF)  # wake up: clear SLEEP
    time.sleep(0.005)
    bus.write_byte_data(PCA_ADDR, MODE1, (old_mode & 0xEF) | 0x80)  # RESTART


def sweep_loop():
    """后台线程：舵机自动来回扫描"""
    while running:
        for pwm in range(SWEEP_MIN, SWEEP_MAX + 1, SWEEP_STEP):
            if not running:
                return
            set_servo(pwm)
            time.sleep(SWEEP_DELAY)
        for pwm in range(SWEEP_MAX, SWEEP_MIN - 1, -SWEEP_STEP):
            if not running:
                return
            set_servo(pwm)
            time.sleep(SWEEP_DELAY)


def print_status():
    """打印当前状态"""
    if MODE == 'pan':
        status = (
            f"\r  PWM: {current_pwm:4d}  |  "
            f"正前={marks.get('center', '?')}  "
            f"左偏={marks.get('left', '?')}  "
            f"右偏={marks.get('right', '?')}  "
            f"|  观察摄像头,按 c/l/r 标记  "
        )
    else:
        status = (
            f"\r  PWM: {current_pwm:4d}  |  "
            f"中心={marks.get('center', '?')}  "
            f"|  观察摄像头,按 c 标记中心  "
        )
    sys.stdout.write(status)
    sys.stdout.flush()


def main():
    global bus, running, SERVO_CH, MODE

    parser = argparse.ArgumentParser(description='舵机 PWM 校准工具')
    parser.add_argument('--channel', type=int, default=10,
                        help='PCA9685 通道号: 10=底座pan, 9=倾斜tilt (默认 10)')
    args = parser.parse_args()

    SERVO_CH = args.channel
    MODE = 'pan' if SERVO_CH == 10 else 'tilt'

    bus = smbus2.SMBus(1)
    init_pca9685(freq=50)

    print("=" * 60)
    print(f"  舵机 PWM 校准工具 - 自动扫描模式")
    print("=" * 60)
    print()
    print(f"  舵机通道: {SERVO_CH} ({MODE})")
    print(f"  驱动芯片: PCA9685 @ 0x{PCA_ADDR:02X}")
    print(f"  扫描范围: PWM {SWEEP_MIN} ~ {SWEEP_MAX}, 步进 {SWEEP_STEP}")
    print()
    print("  舵机正在自动来回扫描，请观察摄像头指向。")
    print()

    if MODE == 'pan':
        print("  通道 10 = 底座 pan 舵机（左右转）")
        print("  标记命令:")
        print("    c  →  标记为「正前方」")
        print("    l  →  标记为「左偏」")
        print("    r  →  标记为「右偏」")
    else:
        print("  通道 9 = 倾斜 tilt 舵机（上下转）")
        print("  标记命令:")
        print("    c  →  标记为「中心」(摄像头略下倾、看清地面)")
    print("    q  →  停止扫描，进入手动精调")
    print()

    # 启动后台扫描线程
    sweep_thread = threading.Thread(target=sweep_loop, daemon=True)
    sweep_thread.start()

    # 主线程处理用户输入
    try:
        while running:
            print_status()
            try:
                cmd = input().strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue

            if cmd == 'q':
                running = False
                break
            elif cmd == 'c':
                marks['center'] = current_pwm
                print(f"\n  ✓ 中心标记: PWM = {current_pwm}")
            elif cmd == 'l':
                marks['left'] = current_pwm
                print(f"\n  ✓ 左偏标记: PWM = {current_pwm}")
            elif cmd == 'r':
                marks['right'] = current_pwm
                print(f"\n  ✓ 右偏标记: PWM = {current_pwm}")
            else:
                print(f"\n  未知命令: {cmd}")

    finally:
        running = False
        sweep_thread.join(timeout=1.0)

    # ── 手动精调模式 ──
    print()
    print("=" * 60)
    print("  扫描已停止。当前标记值:")
    if MODE == 'pan':
        print(f"    正前方 (center): {marks.get('center', '未标记')}")
        print(f"    左偏 (left):     {marks.get('left', '未标记')}")
        print(f"    右偏 (right):    {marks.get('right', '未标记')}")
    else:
        print(f"    中心 (center):   {marks.get('center', '未标记')}")
    print("=" * 60)
    print()
    print("  现在可以手动微调。")
    print("  命令:")
    print("    <数字>+Enter  →  直接设置 PWM 值（如 500）")
    print("    w/s  →  粗调 +25/-25")
    print("    a/d  →  细调 +10/-10")
    if MODE == 'pan':
        print("    c/l/r  →  用当前 PWM 覆盖标记")
    else:
        print("    c  →  用当前 PWM 覆盖中心标记")
    print("    done   →  完成，打印最终结果")
    print()

    pwm = current_pwm
    try:
        set_servo(pwm)
        print(f"  当前 PWM: {pwm}")
    except Exception:
        pass

    while True:
        try:
            cmd_str = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd_str:
            continue

        if cmd_str in ('done', 'q'):
            break
        elif cmd_str == 'c':
            marks['center'] = pwm
            print(f"  ✓ 中心更新: PWM = {pwm}")
        elif cmd_str == 'l':
            marks['left'] = pwm
            print(f"  ✓ 左偏更新: PWM = {pwm}")
        elif cmd_str == 'r':
            marks['right'] = pwm
            print(f"  ✓ 右偏更新: PWM = {pwm}")
        elif cmd_str == 'w':
            pwm = min(4095, pwm + 25)
            set_servo(pwm)
            print(f"  PWM = {pwm}")
        elif cmd_str == 's':
            pwm = max(0, pwm - 25)
            set_servo(pwm)
            print(f"  PWM = {pwm}")
        elif cmd_str == 'a':
            pwm = max(0, pwm - 10)
            set_servo(pwm)
            print(f"  PWM = {pwm}")
        elif cmd_str == 'd':
            pwm = min(4095, pwm + 10)
            set_servo(pwm)
            print(f"  PWM = {pwm}")
        else:
            try:
                val = int(cmd_str)
                if 0 <= val <= 4095:
                    pwm = val
                    set_servo(pwm)
                    print(f"  PWM = {pwm}")
                else:
                    print("  值需在 0-4095 之间")
            except ValueError:
                print("  未知命令")

    # 释放舵机
    try:
        set_servo(0)
    except Exception:
        pass
    bus.close()

    # ── 打印结果 ──
    print()
    print("=" * 60)
    print(f"  通道 {SERVO_CH} ({MODE}) 校准完成！")
    print()
    print("  在 launch 文件或节点参数中添加以下值:")
    print()

    if MODE == 'pan':
        if marks.get('center'):
            print(f"    servo_pan_center: {marks['center']}")
        if marks.get('left'):
            print(f"    servo_pan_left:   {marks['left']}")
        if marks.get('right'):
            print(f"    servo_pan_right:  {marks['right']}")
    else:
        if marks.get('center'):
            print(f"    servo_tilt_center: {marks['center']}")

    # 提示另一个通道
    other_ch = 9 if SERVO_CH == 10 else 10
    other_mode = 'tilt' if MODE == 'pan' else 'pan'
    print()
    print(f"  别忘了校准另一个舵机: python3 servo_calibrate.py --channel {other_ch}")
    print("=" * 60)


if __name__ == '__main__':
    main()
