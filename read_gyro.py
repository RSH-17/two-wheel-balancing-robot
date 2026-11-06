#!/usr/bin/env python3
"""
MPU6050 자이로센서 값 출력 (모터 제어 없음)
라즈베리파이 + MPU6050 (VCC 3.3V, GND, SDA = GPIO2, SCL = GPIO3)

사용법
  python3 read_gyro.py

필요한 패키지
  pip install smbus2 --break-system-packages
"""

import time

from smbus2 import SMBus

MPU_ADDR = 0x68
ACC_SCALE = 16384.0      # ±2g
GYRO_SCALE = 65.5        # ±500°/s


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

    def read_raw(self):
        # 가속도(6) + 온도(2) + 자이로(6) = 14바이트를 한 번에 읽음
        d = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
        vals = []
        for i in range(0, 14, 2):
            v = (d[i] << 8) | d[i + 1]
            vals.append(v - 65536 if v > 32767 else v)
        acc = [v / ACC_SCALE for v in vals[0:3]]
        temp = vals[3] / 340.0 + 36.53
        gyro = [v / GYRO_SCALE for v in vals[4:7]]
        return acc, temp, gyro

    def close(self):
        self.bus.close()


def main():
    mpu = MPU6050()
    print("자이로 값 출력 (단위: deg/s). Ctrl+C로 종료\n")
    try:
        while True:
            acc, temp, gyro = mpu.read_raw()
            gx, gy, gz = gyro
            ax, ay, az = acc
            print(
                f"gyro  x={gx:7.2f}  y={gy:7.2f}  z={gz:7.2f}  deg/s   |   "
                f"acc  x={ax:6.3f}  y={ay:6.3f}  z={az:6.3f}  g   |   temp={temp:5.1f}C"
            )
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        mpu.close()


if __name__ == "__main__":
    main()
