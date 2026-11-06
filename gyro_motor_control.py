#!/usr/bin/env python3
"""
자이로센서 값으로 모터 제어 (PID 없음, naive)
자이로 각속도 값을 그대로 게인만 곱해서 두 모터에 동일하게 출력합니다.

라즈베리파이 + Cytron MDD3A + JGB37-520 + MPU6050

배선
  GPIO18 -> MDD3A M1A,  GPIO19 -> MDD3A M1B   (모터 1)
  GPIO12 -> MDD3A M2A,  GPIO13 -> MDD3A M2B   (모터 2)
  MPU6050: VCC 3.3V, GND, SDA = GPIO2, SCL = GPIO3

사용법
  python3 gyro_motor_control.py                  # 기본 게인 사용
  python3 gyro_motor_control.py --gain 0.02       # 게인 조절
"""

import argparse
import time

from gpiozero import PWMOutputDevice
from smbus2 import SMBus

# ===================== 설정 =====================

PIN_M1A, PIN_M1B = 18, 19
PIN_M2A, PIN_M2B = 12, 13
PWM_FREQ = 1000

INVERT_M1 = False
INVERT_M2 = True

GYRO_AXIS = 1            # 제어에 사용할 자이로 축 (0=X, 1=Y, 2=Z)
GYRO_SIGN = 1            # 부호 (방향이 반대면 -1로 바꾸기)

MIN_PWM = 0.12           # 모터 데드밴드 보상
DEFAULT_GAIN = 0.02      # 자이로 값(deg/s) -> 모터 출력 게인

MPU_ADDR = 0x68
GYRO_SCALE = 65.5        # ±500°/s

# =================================================


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class MPU6050:
    def __init__(self, bus_num=1, addr=MPU_ADDR):
        self.bus = SMBus(bus_num)
        self.addr = addr
        w = self.bus.write_byte_data
        w(addr, 0x6B, 0x01)   # 슬립 해제
        time.sleep(0.1)
        w(addr, 0x19, 4)      # 샘플레이트 200Hz
        w(addr, 0x1A, 3)      # 저역통과 필터
        w(addr, 0x1B, 0x08)   # 자이로 ±500°/s
        w(addr, 0x1C, 0x00)   # 가속도 ±2g
        self.gyro_offset = 0.0

    def read_gyro(self):
        d = self.bus.read_i2c_block_data(self.addr, 0x43, 6)
        vals = []
        for i in range(0, 6, 2):
            v = (d[i] << 8) | d[i + 1]
            vals.append(v - 65536 if v > 32767 else v)
        return [v / GYRO_SCALE for v in vals]

    def calibrate(self, samples=300):
        total = 0.0
        for _ in range(samples):
            gyro = self.read_gyro()
            total += gyro[GYRO_AXIS]
            time.sleep(0.002)
        self.gyro_offset = total / samples

    def read_rate(self):
        gyro = self.read_gyro()
        return GYRO_SIGN * (gyro[GYRO_AXIS] - self.gyro_offset)

    def close(self):
        self.bus.close()


class Motor:
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


def run(gain):
    mpu = MPU6050()
    m1 = Motor(PIN_M1A, PIN_M1B, INVERT_M1)
    m2 = Motor(PIN_M2A, PIN_M2B, INVERT_M2)

    print("자이로 보정 중... 로봇을 가만히 두세요.")
    mpu.calibrate()
    print(f"준비 완료. gain={gain}  (Ctrl+C로 종료)")

    try:
        while True:
            rate = mpu.read_rate()
            out = gain * rate
            if abs(out) > 0.01:
                out += MIN_PWM if out > 0 else -MIN_PWM
            out = clamp(out)

            m1.set(out)
            m2.set(out)

            print(f"gyro {rate:7.1f} deg/s   out {out:5.2f}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        m1.close()
        m2.close()
        mpu.close()


def main():
    p = argparse.ArgumentParser(description="자이로 값으로 모터 제어 (naive)")
    p.add_argument("--gain", type=float, default=DEFAULT_GAIN)
    args = p.parse_args()
    run(args.gain)


if __name__ == "__main__":
    main()
