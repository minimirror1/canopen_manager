#!/bin/bash

# 이 스크립트는 ros1_interface를 빌드합니다

# 현재 디렉토리 위치 저장
SCRIPT_DIR=$(pwd)

# ROS1 환경 설정 (ROS1 설치 경로에 따라 수정 필요)
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
  source /opt/ros/melodic/setup.bash
fi

# 상위 디렉토리(워크스페이스 루트)로 이동
cd ..

# ros1_interface 패키지 빌드
catkin_make --pkg ros1_interface

# 이전 디렉토리로 복귀
cd $SCRIPT_DIR

echo "빌드 완료!" 