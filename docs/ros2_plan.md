# ROS2 전환 설계 메모

> 작성: 2026-11-06 · 상태: **설계만 완료, 구현 안 됨**
>
> 현재 환경
> - 로컬 PC: Ubuntu 24.04, ROS2 **Jazzy** 설치됨
> - 라즈베리파이4: **Raspberry Pi OS 13 (trixie, Debian 13 기반)** 확인됨. ROS2 **미설치**
> - 현재 코드는 `control.py` 단일 프로세스 (200Hz 루프, gpiozero PWM + smbus2 I2C)

---

## 1. 가장 중요한 설계 원칙

**200Hz 밸런싱 PID 루프를 ROS2 토픽으로 쪼개지 말 것.**

IMU 노드 → 제어 노드 → 모터 노드로 나누는 게 "ROS답게" 보이지만,
매 주기마다 DDS를 두 번 왕복하면 지터가 수 ms 단위로 들어온다.
밸런싱은 그 지터에 바로 넘어진다.

대신 **이중 루프**로 나눈다.

```
 [내부 루프 · 200Hz · 단일 노드 안의 전용 스레드]
   MPU6050 읽기 → 상보필터 → PID → PWM 출력
   (토픽 안 거침. 지금 control.py 의 run_balance() 그대로)
                    ↑ 공유 변수 (lock)
 [외부 루프 · 20~50Hz · ROS2 토픽/파라미터]
   /cmd_vel 수신 → 목표 기울기 bias + 조향량
   /imu/data, /balance_state 발행 → PC 에서 rqt_plot
   ros2 param set 으로 Kp/Ki/Kd 실시간 튜닝
```

ROS2 는 **밸런싱을 하는 주체가 아니라, 밸런싱 노드에 명령을 넣고 상태를 꺼내 보는 껍데기** 역할.

---

## 2. 패키지 구조

```
~/ros2_ws/src/
├── balance_bot/                      # ament_python
│   ├── balance_bot/
│   │   ├── mpu6050.py                # 기존 MPU6050 클래스 그대로 재사용
│   │   ├── motor.py                  # 기존 Motor 클래스 그대로 재사용
│   │   └── balance_controller.py     # 노드 (내부 루프 스레드 + 콜백)
│   ├── launch/balance.launch.py
│   ├── config/balance_params.yaml    # kp/ki/kd/target/invert/축 설정
│   ├── setup.py, package.xml
└── balance_bot_interfaces/           # ament_cmake
    └── msg/BalanceState.msg          # angle, rate, output, armed, loop_hz
```

기존 `MPU6050`, `Motor` 클래스는 ROS 의존성이 전혀 없으므로 **그대로 모듈로 옮기면 된다.**
새로 쓸 건 노드 래퍼뿐.

### 사용할 메시지 타입

| 용도 | 타입 |
|---|---|
| 속도 명령 수신 | `geometry_msgs/Twist` (`/cmd_vel`) |
| IMU 발행 | `sensor_msgs/Imu` (`/imu/data_raw`) |
| 밸런싱 상태 발행 | 커스텀 `BalanceState.msg` (`/balance_state`) |
| 비상 정지 | `std_srvs/Trigger` |

---

## 3. 핵심 코드 모양

### cmd_vel → 기울기 바이어스 + 차동 조향

밸런싱 로봇은 앞으로 기울여서 전진한다. 그래서 `linear.x` 는 목표 각도를 흔드는 값이 된다.

```python
def on_cmd_vel(self, msg):              # 콜백, 느린 주기
    with self.lock:
        self.bias  = clamp(self.k_vel   * msg.linear.x,  -self.max_bias, self.max_bias)
        self.steer = clamp(self.k_steer * msg.angular.z, -self.max_steer, self.max_steer)
        self.last_cmd = time.monotonic()
```

```python
# 내부 루프 안 (기존 run_balance 루프에서 두 줄만 바뀜)
with self.lock:
    if time.monotonic() - self.last_cmd > 0.5:   # 워치독: 명령 끊기면 제자리
        self.bias = self.steer = 0.0
    bias, steer = self.bias, self.steer

error = angle - (self.target_angle + bias)
out = self.kp * error + self.ki * integral + self.kd * rate
m1.set(clamp(out + steer))               # ← 조향은 여기서만 갈라짐
m2.set(clamp(out - steer))
```

### PID 실시간 튜닝 (ROS2 로 옮기는 가장 큰 실익)

지금은 `--kp` 바꿔서 매번 프로세스를 재시작해야 한다.

```python
self.declare_parameter('kp', 0.05)   # ... ki, kd, target_angle
self.add_on_set_parameters_callback(self.on_set_params)
```

```bash
# 로봇이 흔들리는 중에 바로
ros2 param set /balance_controller kp 0.062
```

### 루프 스레드

`create_timer` 는 200Hz 에서 지터가 있으니 쓰지 말 것.

```python
threading.Thread(target=self.control_loop, daemon=True).start()
rclpy.spin(node)        # 콜백은 메인 스레드에서
```

루프가 대부분 `sleep` 상태라 GIL 경합은 사실상 없다.
발행은 루프 안에서 매번 하지 말고 20Hz 로 솎아낸다.

---

## 4. 미리 알아야 할 한계: 엔코더가 없다

`/cmd_vel` 을 받아도 **속도 피드백이 없어서 지정 속도를 유지할 수 없다.**
`linear.x` 는 기울기 바이어스로만 쓰이므로 사실상 가속도 명령이다.
명령을 계속 주면 계속 가속하고, 0 을 줘도 관성으로 밀려 제자리로 못 돌아온다.

- `max_bias` 를 작게 (±2~3° 정도) 잡고 레이트 리미터를 걸 것
- `nav2`, `odom`, SLAM 은 엔코더 없이 **불가능**.
  거기까지 가려면 JGB37-520 엔코더 버전으로 교체가 선행 조건
- 지금 할 수 있는 건 **teleop 수준의 수동 조종 + 텔레메트리 시각화**까지

---

## 5. ROS2 설치 경로

**확인됨: Pi 는 Raspberry Pi OS 13 (trixie, Debian 13 기반), arm64.**
ROS2 공식 apt 저장소는 Ubuntu 전용이라 Raspberry Pi OS 에는 Jazzy/Humble 패키지가 없다.
선택지는 아래 세 가지.

| 방법 | 내용 | 평가 |
|---|---|---|
| **Docker (추천)** | `docker run --device /dev/i2c-1 --device /dev/gpiochip0 ... ros:jazzy-ros-base` | OS 그대로 두고 격리됨. GPIO/I2C 는 `--device` 로 컨테이너에 노출. PC 와 동일 Jazzy 라 통신 걱정 없음 ✅ |
| Ubuntu Server 24.04 arm64 로 SD 카드 재설치 | RPi Imager 에서 OS 를 Ubuntu Server 24.04 arm64 로 교체 | `apt` 로 Jazzy 바로 설치, 가장 깔끔하지만 **재설치라 기존 설정 날아감** |
| 소스 빌드 / RoboStack(conda) | Debian trixie 위에서 `rosdep` 으로 직접 빌드 | 가능은 하나 의존성 맞추기 번거로움, 비권장 |

Docker 쪽이 **지금 설정을 안 건드리고 바로 시도해볼 수 있어 1순위**.
단, GPIO 하드웨어 PWM 접근이 컨테이너 안에서 막히는 경우가 있어 실제로 모터가 돌아가는지는 붙여보고 확인 필요.

PC 와 Pi 는 같은 `ROS_DOMAIN_ID` 만 맞추면 같은 LAN 에서 자동으로 서로 보인다.
그러면 **제어는 Pi 에서, `rqt_plot` 으로 각도 그래프 보고 `teleop_twist_keyboard` 로 조종은 PC 에서** 하는 구성이 된다.
지금 `print` 로 찍어 보는 각도/출력을 그래프로 보게 되는 것만으로도 PID 튜닝이 훨씬 수월해진다.

---

## 6. 별건이지만 관련된 메모

GPIO18/19/12/13 은 하드웨어 PWM 핀인데 `gpiozero` 의 lgpio 백엔드는 소프트웨어 PWM 을 쓴다
(작업 폴더에 `.lgd-nfy0` FIFO 가 생긴 이유).
ROS2 와 별개 사안이지만 200Hz 제어에서는 PWM 지터가 성능에 영향을 주므로,
밸런싱이 잘 안 잡히면 하드웨어 PWM 전환도 후보로 둘 것.

---

## 7. 다음에 할 일

- [x] Pi 의 OS 확인 — **Raspberry Pi OS 13 (trixie)** 확인됨
- [ ] Pi 에 Docker 설치 후 `ros:jazzy-ros-base` 로 ROS2 컨테이너 기동, GPIO/I2C 장치 노출 확인
- [ ] `balance_bot` / `balance_bot_interfaces` 패키지 생성
- [ ] `MPU6050`, `Motor` 클래스를 모듈로 분리
- [ ] `balance_controller` 노드 작성 (내부 루프 스레드 + cmd_vel 콜백 + 파라미터 콜백)
- [ ] launch 파일 + params yaml
- [ ] PC 에서 `teleop_twist_keyboard` / `rqt_plot` 연결 확인
