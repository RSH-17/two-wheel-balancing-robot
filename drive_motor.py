#!/usr/bin/env python3
"""
모터 구동 테스트 (자이로/밸런싱 없음)
라즈베리파이 + Cytron MDD3A + JGB37-520

배선
  GPIO18 -> MDD3A M1A,  GPIO19 -> MDD3A M1B   (모터 1)
  GPIO12 -> MDD3A M2A,  GPIO13 -> MDD3A M2B   (모터 2)

사용법
  python3 drive_motor.py                      # 기본 테스트 시퀀스 (양쪽 모터 번갈아 전진/후진)
  python3 drive_motor.py --speed 0.5 --duration 2   # 속도/시간 지정
  python3 drive_motor.py --motor 1 --speed -0.3      # 모터 1만 후진
"""

import argparse
import time

from gpiozero import PWMOutputDevice

# ===================== 설정 =====================

PIN_M1A, PIN_M1B = 18, 19
PIN_M2A, PIN_M2B = 12, 13
PWM_FREQ = 1000          # PWM 주파수 (Hz)

# 바퀴 방향 반전 (필요하면 조정)
INVERT_M1 = False
INVERT_M2 = True

# =================================================


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


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


def run_default_sequence(speed, duration):
    """기본 테스트: 모터 1, 모터 2, 두 모터 함께 순서대로 구동"""
    m1 = Motor(PIN_M1A, PIN_M1B, INVERT_M1)
    m2 = Motor(PIN_M2A, PIN_M2B, INVERT_M2)
    try:
        print("로봇을 들어서 바퀴가 바닥에 닿지 않게 해주세요.")
        time.sleep(2)

        print(f"모터 1 구동 (speed={speed}, {duration}초)")
        m1.set(speed)
        time.sleep(duration)
        m1.set(0)
        time.sleep(0.5)

        print(f"모터 2 구동 (speed={speed}, {duration}초)")
        m2.set(speed)
        time.sleep(duration)
        m2.set(0)
        time.sleep(0.5)

        print(f"두 모터 함께 구동 (speed={speed}, {duration}초)")
        m1.set(speed)
        m2.set(speed)
        time.sleep(duration)
    finally:
        m1.close()
        m2.close()
        print("정지")


def run_single_motor(motor_num, speed, duration):
    """지정한 모터 하나만 구동"""
    if motor_num == 1:
        m = Motor(PIN_M1A, PIN_M1B, INVERT_M1)
    else:
        m = Motor(PIN_M2A, PIN_M2B, INVERT_M2)
    try:
        print(f"모터 {motor_num} 구동 (speed={speed}, {duration}초)")
        m.set(speed)
        time.sleep(duration)
    finally:
        m.close()
        print("정지")


def main():
    p = argparse.ArgumentParser(description="모터 구동 테스트")
    p.add_argument("--speed", type=float, default=0.4, help="속도 (-1.0 ~ 1.0, 기본 0.4)")
    p.add_argument("--duration", type=float, default=1.5, help="구동 시간(초), 기본 1.5")
    p.add_argument("--motor", type=int, choices=[1, 2], default=None,
                   help="지정하면 해당 모터만 구동 (생략하면 기본 시퀀스 실행)")
    args = p.parse_args()

    try:
        if args.motor is not None:
            run_single_motor(args.motor, args.speed, args.duration)
        else:
            run_default_sequence(args.speed, args.duration)
    except KeyboardInterrupt:
        print("\n중단됨")


if __name__ == "__main__":
    main()
