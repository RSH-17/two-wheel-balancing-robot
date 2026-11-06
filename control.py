#!/usr/bin/env python3
"""
두 바퀴 밸런싱 로봇 제어 코드
라즈베리파이 + Cytron MDD3A + JGB37-520 (12V, 엔코더 없음) + MPU6050

배선
  GPIO18 -> MDD3A M1A,  GPIO19 -> MDD3A M1B   (모터 1)
  GPIO12 -> MDD3A M2A,  GPIO13 -> MDD3A M2B   (모터 2)
  MPU6050: VCC 3.3V, GND, SDA = GPIO2, SCL = GPIO3

사용 순서
  python3 balance.py motor    # 1단계: 바퀴 회전 방향 확인
  python3 balance.py axis     # 2단계: 센서 축과 각도 부호 확인
  python3 balance.py run      # 3단계: 밸런싱 시작
  python3 balance.py run --kp 0.06 --kd 0.003 --target 1.5   # 값 바꿔가며 튜닝

필요한 패키지
  pip install smbus2 --break-system-packages   (gpiozero는 기본 설치됨)
"""

import argparse
import math
import time

from gpiozero import PWMOutputDevice
from smbus2 import SMBus

# ===================== 설정 =====================

# 모터 핀
PIN_M1A, PIN_M1B = 18, 19
PIN_M2A, PIN_M2B = 12, 13
PWM_FREQ = 1000          # PWM 주파수 (Hz)

# 바퀴 방향 반전 (motor 모드로 확인 후 수정)
# 두 모터가 서로 마주보게 달려 있으면 보통 한쪽을 반전해야 둘 다 앞으로 굴러갑니다.
INVERT_M1 = False
INVERT_M2 = True

# 센서 축 설정 (axis 모드로 확인 후 수정)
# 기본값: MPU6050을 수평으로 눕히고, 기판의 X축 화살표가 로봇 앞쪽을 향하게 붙인 경우
# 축 번호: 0 = X, 1 = Y, 2 = Z
ACC_FWD_AXIS = 0         # 앞뒤 방향 가속도 축
ACC_FWD_SIGN = -1        # 앞으로 기울일 때 가속도 각도가 +가 되도록 하는 부호
ACC_UP_AXIS = 2          # 위쪽을 향하는 가속도 축
GYRO_AXIS = 1            # 앞뒤로 기울일 때 회전하는 자이로 축
GYRO_SIGN = 1            # 앞으로 기울일 때 자이로 값이 +가 되도록 하는 부호

# 제어 설정
LOOP_HZ = 200            # 제어 주기
ALPHA = 0.98             # 상보 필터 비율 (자이로 비중)
FALL_ANGLE = 40.0        # 이 각도를 넘으면 넘어진 것으로 보고 모터 정지
START_ANGLE = 5.0        # 이 각도 안으로 세워지면 밸런싱 시작
MIN_PWM = 0.12           # 모터 데드밴드 보상 (낮은 PWM에서 안 도는 구간)
I_LIMIT = 0.3            # 적분항이 출력에 기여할 수 있는 최대치

# PID 기본값 (출력 범위 -1 ~ 1, 각도 단위는 도)
DEFAULT_KP = 0.05
DEFAULT_KI = 0.0
DEFAULT_KD = 0.002
DEFAULT_TARGET = 0.0     # 실제로 균형이 맞는 각도 (axis 모드로 확인)

# MPU6050
MPU_ADDR = 0x68
ACC_SCALE = 16384.0      # ±2g
GYRO_SCALE = 65.5        # ±500°/s

# =================================================


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class MPU6050:
    def __init__(self, bus_num=1, addr=MPU_ADDR):
        self.bus = SMBus(bus_num)
        self.addr = addr
        w = self.bus.write_byte_data
        w(addr, 0x6B, 0x01)   # 슬립 해제, 자이로 클럭 사용
        time.sleep(0.1)
        w(addr, 0x19, 4)      # 샘플레이트 1kHz / (1 + 4) = 200Hz
        w(addr, 0x1A, 3)      # 내부 저역통과 필터 약 44Hz
        w(addr, 0x1B, 0x08)   # 자이로 ±500°/s
        w(addr, 0x1C, 0x00)   # 가속도 ±2g
        self.gyro_offset = 0.0

    def read_raw(self):
        # 가속도(6) + 온도(2) + 자이로(6) = 14바이트를 한 번에 읽음
        d = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
        vals = []
        for i in range(0, 14, 2):
            v = (d[i] << 8) | d[i + 1]
            vals.append(v - 65536 if v > 32767 else v)
        acc = [v / ACC_SCALE for v in vals[0:3]]
        gyro = [v / GYRO_SCALE for v in vals[4:7]]
        return acc, gyro

    def calibrate(self, samples=500):
        total = 0.0
        for _ in range(samples):
            _, gyro = self.read_raw()
            total += gyro[GYRO_AXIS]
            time.sleep(0.002)
        self.gyro_offset = total / samples

    def read_pitch(self):
        """가속도로 계산한 각도(도)와 자이로 각속도(도/초)를 반환"""
        acc, gyro = self.read_raw()
        acc_angle = math.degrees(
            math.atan2(ACC_FWD_SIGN * acc[ACC_FWD_AXIS], acc[ACC_UP_AXIS])
        )
        rate = GYRO_SIGN * (gyro[GYRO_AXIS] - self.gyro_offset)
        return acc_angle, rate

    def close(self):
        self.bus.close()


class Motor:
    """MDD3A 한 채널 (듀얼 PWM 방식)"""

    def __init__(self, pin_a, pin_b, invert=False):
        self.a = PWMOutputDevice(pin_a, frequency=PWM_FREQ)
        self.b = PWMOutputDevice(pin_b, frequency=PWM_FREQ)
        self.invert = invert

    def set(self, speed):
        speed = clamp(speed)
        if self.invert:
            speed = -speed
        if speed > 0:
            self.b.value = 0
            self.a.value = speed
        elif speed < 0:
            self.a.value = 0
            self.b.value = -speed
        else:
            self.a.value = 0
            self.b.value = 0

    def close(self):
        self.set(0)
        self.a.close()
        self.b.close()


def test_motors():
    """1단계: 각 모터가 '앞으로' 도는지 확인"""
    m1 = Motor(PIN_M1A, PIN_M1B, INVERT_M1)
    m2 = Motor(PIN_M2A, PIN_M2B, INVERT_M2)
    try:
        print("로봇을 들어서 바퀴가 바닥에 닿지 않게 해주세요.")
        time.sleep(2)
        print("모터 1 앞으로 (1.5초)")
        m1.set(0.4)
        time.sleep(1.5)
        m1.set(0)
        time.sleep(0.5)
        print("모터 2 앞으로 (1.5초)")
        m2.set(0.4)
        time.sleep(1.5)
        m2.set(0)
        time.sleep(0.5)
        print("두 모터 함께 앞으로 (1.5초)")
        m1.set(0.4)
        m2.set(0.4)
        time.sleep(1.5)
        print("\n두 바퀴 모두 로봇 앞쪽으로 굴러가는 방향이어야 합니다.")
        print("반대로 도는 바퀴가 있으면 코드 위쪽의 INVERT_M1 / INVERT_M2 값을 바꾸세요.")
    finally:
        m1.close()
        m2.close()


def check_axis():
    """2단계: 앞으로 기울였을 때 값들이 모두 +가 되는지 확인"""
    mpu = MPU6050()
    print("로봇을 똑바로 세운 채 가만히 두세요. 자이로 보정 중...")
    mpu.calibrate()
    angle, _ = mpu.read_pitch()
    last = time.perf_counter()
    print("로봇을 앞으로 천천히 기울여 보세요. 세 값이 모두 +로 커져야 합니다.")
    print("Ctrl+C로 종료\n")
    try:
        while True:
            now = time.perf_counter()
            dt = now - last
            last = now
            acc_angle, rate = mpu.read_pitch()
            angle = ALPHA * (angle + rate * dt) + (1 - ALPHA) * acc_angle
            print(f"가속도 각도 {acc_angle:7.2f}°   필터 각도 {angle:7.2f}°   자이로 {rate:7.1f}°/s")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n균형이 맞는 지점의 '필터 각도'를 --target 값으로 쓰면 됩니다.")
    finally:
        mpu.close()


def run_balance(kp, ki, kd, target):
    """3단계: 밸런싱"""
    mpu = MPU6050()
    m1 = Motor(PIN_M1A, PIN_M1B, INVERT_M1)
    m2 = Motor(PIN_M2A, PIN_M2B, INVERT_M2)

    print(f"Kp={kp}  Ki={ki}  Kd={kd}  target={target}")
    print("로봇을 똑바로 세운 채 가만히 두세요. 자이로 보정 중...")
    mpu.calibrate()

    angle, _ = mpu.read_pitch()
    integral = 0.0
    armed = False
    period = 1.0 / LOOP_HZ
    last = time.perf_counter()
    next_t = last
    last_print = last
    print(f"준비 완료. 로봇을 ±{START_ANGLE}° 안으로 세우면 시작합니다. (Ctrl+C로 종료)")

    try:
        while True:
            now = time.perf_counter()
            dt = now - last
            last = now
            if dt <= 0 or dt > 0.1:
                dt = period

            acc_angle, rate = mpu.read_pitch()
            angle = ALPHA * (angle + rate * dt) + (1 - ALPHA) * acc_angle
            error = angle - target

            if not armed:
                out = 0.0
                if abs(error) < START_ANGLE:
                    armed = True
                    integral = 0.0
                    print("밸런싱 시작")
            elif abs(error) > FALL_ANGLE:
                armed = False
                out = 0.0
                print("넘어짐 감지: 모터 정지. 다시 세우면 재시작합니다.")
            else:
                if ki > 0:
                    integral = clamp(integral + error * dt, -I_LIMIT / ki, I_LIMIT / ki)
                out = kp * error + ki * integral + kd * rate
                if abs(out) > 0.01:
                    out += MIN_PWM if out > 0 else -MIN_PWM
                out = clamp(out)

            m1.set(out)
            m2.set(out)

            if now - last_print > 0.2:
                print(f"각도 {angle:6.2f}°  출력 {out:5.2f}  주기 {1 / dt:4.0f}Hz")
                last_print = now

            # 일정한 주기 유지
            next_t += period
            remaining = next_t - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        m1.close()
        m2.close()
        mpu.close()


def main():
    p = argparse.ArgumentParser(description="두 바퀴 밸런싱 로봇")
    p.add_argument("mode", choices=["motor", "axis", "run"],
                   help="motor: 바퀴 방향 확인 / axis: 센서 확인 / run: 밸런싱")
    p.add_argument("--kp", type=float, default=DEFAULT_KP)
    p.add_argument("--ki", type=float, default=DEFAULT_KI)
    p.add_argument("--kd", type=float, default=DEFAULT_KD)
    p.add_argument("--target", type=float, default=DEFAULT_TARGET)
    args = p.parse_args()

    if args.mode == "motor":
        test_motors()
    elif args.mode == "axis":
        check_axis()
    else:
        run_balance(args.kp, args.ki, args.kd, args.target)


if __name__ == "__main__":
    main()