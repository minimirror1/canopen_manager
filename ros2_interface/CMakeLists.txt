cmake_minimum_required(VERSION 3.8)
project(ros2_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/canopen_node.py
  src/canopen_node_new.py
  src/send_command.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 추가: common_canopen 디렉토리 복사
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_canopen
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*"
)

# 모든 EDS 파일 설치를 위한 설정
file(GLOB EDS_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../common_canopen/canopen_motor/config/*.eds")
install(FILES
  ${EDS_FILES}
  DESTINATION share/${PROJECT_NAME}/common_canopen/config
)

ament_package() 