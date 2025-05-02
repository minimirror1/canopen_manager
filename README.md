# CANopen Manager

CANopen 프로토콜을 사용하여 모터를 제어하는 ROS1/ROS2 공용 패키지입니다.

## 폴더 구조

```
canopen_shared_repo/
├── common_canopen/         # 공통 CANopen 로직 구현 (ROS 의존 없음)
│   ├── canopen_motor/      # 모터 관련 로직
│   │   ├── motor_manager/  # 모터 관리 클래스
│   │   ├── motor_vendors/  # 제조사별 모터 구현
│   │   └── config/         # 설정 파일
│   ├── canopen_info_json/  # CANopen 설정 JSON 파일
│   ├── include/            # 헤더 파일(필요시)
│   └── src/                # 소스 파일(필요시)
├── ros1_interface/         # ROS1 인터페이스
│   ├── src/                # ROS1 노드 구현
│   │   ├── canopen_node.py
│   │   ├── send_command.py
│   │   └── motor_setting_node.py
│   ├── launch/             # ROS1 launch 파일
│   └── msg/                # ROS1 메시지 정의(필요시)
├── ros2_interface/         # ROS2 인터페이스
│   ├── src/                # ROS2 노드 구현
│   │   ├── canopen_node.py
│   │   └── send_command.py
│   ├── launch/             # ROS2 launch 파일
│   └── msg/                # ROS2 메시지 정의(필요시)
```

## 사용 방법

### ROS1에서 사용

```bash
# ROS1 워크스페이스에서 빌드
cd ~/catkin_ws/src
git clone [REPO_URL]
cd ..
catkin_make

# 노드 실행
roslaunch ros1_interface canopen_manager.launch

# 모터에 위치 명령 전송
rosrun ros1_interface send_command.py position 1.57 [모터이름]

# 명령 전송
rosrun ros1_interface send_command.py command [명령]
```

### ROS2에서 사용

```bash
# ROS2 워크스페이스에서 빌드
cd ~/ros2_ws/src
git clone [REPO_URL]
cd ..
colcon build --packages-select common_canopen ros2_interface

# 노드 실행
ros2 launch ros2_interface canopen_manager.launch.py

# 모터에 위치 명령 전송
ros2 run ros2_interface send_command.py position 1.57 [모터이름]

# 명령 전송
ros2 run ros2_interface send_command.py command [명령]
```

## 의존성

- Python 3.6 이상
- python-canopen-pip (pip3 install canopen)
- ROS1 또는 ROS2
- PySide6 (GUI를 위한 의존성, 선택적)

## 라이선스

BSD 라이선스

## 저자

[저자 이름] <email@example.com> 