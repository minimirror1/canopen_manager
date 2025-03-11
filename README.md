# CANopen Manager

CANopen 프로토콜을 관리하는 ROS 패키지입니다.

## 설치 방법

```bash
# ROS 작업 공간으로 이동
cd ~/catkin_ws

# 소스 코드 복제
git clone https://github.com/your_username/canopen_manager.git src/canopen_manager

# 의존성 설치
sudo apt-get install python-pip
pip install canopen

# 빌드
catkin_make

# 환경 설정
source devel/setup.bash
```

## 사용 방법

### CAN 인터페이스 설정

이 패키지는 CAN 인터페이스를 자동으로 설정하려고 시도합니다. 하지만 CAN 인터페이스 설정에는 root 권한이 필요합니다. 다음 두 가지 방법 중 하나를 선택할 수 있습니다:

1. 노드를 실행하기 전에 수동으로 CAN 인터페이스 설정 (CANable 사용):
   ```bash
   sudo slcand -o -s8 -t hw -S 1000000 /dev/ttyACM0 can0
   sudo ip link set can0 up
   sudo ip link set can0 txqueuelen 1000
   ```

2. 노드를 root 권한으로 실행:
   ```bash
   sudo roslaunch canopen_manager canopen_manager.launch
   ```

### 노드 실행

```bash
# launch 파일을 사용하여 노드 실행
roslaunch canopen_manager canopen_manager.launch
```

### 파라미터

launch 파일에서 다음 파라미터를 설정할 수 있습니다:

- `can_interface`: CAN 인터페이스 이름 (기본값: "can0")
- `node_id`: CANopen 노드 ID (기본값: 1)
- `heartbeat_interval`: 하트비트 간격 (ms) (기본값: 1000)

### 토픽

#### 발행 토픽

- `/canopen/status` (std_msgs/String): CANopen 장치의 상태 정보
- `/canopen/position` (std_msgs/Float64): 현재 모터 위치 (라디안)

#### 구독 토픽

- `/canopen/command` (std_msgs/String): CANopen 장치에 보낼 명령
- `/canopen/set_position` (std_msgs/Float64): 모터 위치 설정 (라디안)

### 명령 보내기

```bash
# 일반 명령 보내기
rosrun canopen_manager send_command.py command reset
rosrun canopen_manager send_command.py command init

# 특정 모터에 위치 명령 보내기 (라디안)
rosrun canopen_manager send_command.py position 1.57 j_l1
```

### 모터 관리 기능

이 패키지는 `canopen_motor` 모듈을 통해 다양한 제조사의 CANopen 모터를 관리할 수 있습니다.

#### 모터 설정 방법

모터 설정은 `canopen_info_json/canopen_info.json` 파일에서 관리됩니다. 이 파일의 형식은 다음과 같습니다:

```json
{
    "motors": [
        {
            "name": "j_l1",
            "vendor_type": "VendorZeroErr", 
            "node_id": 11,            
            "zero_offset": 84303,
            "operation_mode": "PROFILE_POSITION",
            "profile_velocity": 1.0,
            "profile_acceleration": 1.0,
            "profile_deceleration": 1.0
        }
    ]
}
```

각 모터에 대해 다음 파라미터를 설정할 수 있습니다:
- `name`: 모터 이름 (필수)
- `vendor_type`: 제조사 타입 (필수, 예: "VendorZeroErr")
- `node_id`: CAN 노드 ID (필수)
- `operation_mode`: 동작 모드 (필수, 예: "PROFILE_POSITION")
- `zero_offset`: 영점 오프셋 (선택 사항)
- `profile_velocity`: 프로파일 속도 (rad/s) (선택 사항)
- `profile_acceleration`: 프로파일 가속도 (rad/s²) (선택 사항)
- `profile_deceleration`: 프로파일 감속도 (rad/s²) (선택 사항)

#### 지원하는 모터 제조사

- VendorZeroErr: ZeroErr 모터 드라이버
- VendorElmo: Elmo 모터 드라이버

## 라이선스

BSD 라이선스 