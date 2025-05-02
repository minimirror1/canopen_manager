#!/bin/bash

# 이 스크립트는 ros2_interface를 빌드합니다

# 현재 디렉토리 위치 저장
SCRIPT_DIR=$(pwd)

# ROS2 환경 설정 (ROS2 설치 경로에 따라 수정 필요)
if [ -f /opt/ros/foxy/setup.bash ]; then
  source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# 상위 디렉토리(워크스페이스 루트)로 이동
cd ..

# ros2_interface 패키지만 빌드
colcon build --packages-select ros2_interface

# 이전 디렉토리로 복귀
cd $SCRIPT_DIR

echo "빌드 완료!"
