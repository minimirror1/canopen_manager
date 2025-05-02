#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import os
import json
import sys
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState

def load_motors_info():
    """JSON 파일에서 모터 정보를 로드합니다."""
    try:
        # 설치된 config 디렉토리에서 파일 찾기
        ros_share_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', '..', 'share', 'ros2_interface', 'config')
        json_path = os.path.join(ros_share_path, 'canopen_info.json')
        
        # 파일이 존재하지 않으면 개발 환경에서 찾기
        if not os.path.exists(json_path):
            common_canopen_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'common_canopen')
            json_path = os.path.join(common_canopen_path, 'canopen_info_json', 'canopen_info.json')
            
        with open(json_path, 'r') as f:
            data = json.load(f)
            return data.get('motors', [])
    except Exception as e:
        print(f"모터 정보 로드 실패: {str(e)}")
        return []

def get_motor_by_name(motors_info, name):
    """이름으로 모터 정보를 찾습니다."""
    for motor in motors_info:
        if motor.get('name') == name:
            return motor
    return None

class CommandSender(Node):
    def __init__(self, command_type, args):
        super().__init__('canopen_command_sender')
        
        # 모터 정보 로드
        self.motors_info = load_motors_info()
        
        if command_type == "command":
            if len(args) < 1:
                self.get_logger().error("명령을 지정해야 합니다.")
                return
                
            # 발행자 설정
            self.pub = self.create_publisher(String, 'canopen/command', 10)
            
            # 명령 메시지 생성
            command = ' '.join(args)
            msg = String()
            msg.data = command
            
            # 메시지 발행
            self.pub.publish(msg)
            self.get_logger().info(f"명령을 보냈습니다: {command}")
            
        elif command_type == "position":
            if len(args) < 1:
                self.get_logger().error("위치 값을 지정해야 합니다.")
                return
                
            try:
                position = float(args[0])
            except ValueError:
                self.get_logger().error("위치 값은 숫자여야 합니다.")
                return
            
            # 모터 이름 확인
            motor_name = args[1] if len(args) > 1 else None
            
            if motor_name:
                # 특정 모터에 명령 전송
                motor_info = get_motor_by_name(self.motors_info, motor_name)
                if not motor_info:
                    self.get_logger().error(f"모터 '{motor_name}'를 찾을 수 없습니다.")
                    return
                    
                # 발행자 설정
                self.pub = self.create_publisher(
                    Float64, 
                    f'canopen/single_motor/{motor_name}/position', 
                    10
                )
                
                # 위치 메시지 생성
                msg = Float64()
                msg.data = position
                
                # 메시지 발행
                self.pub.publish(msg)
                self.get_logger().info(f"모터 '{motor_name}'에 위치 명령을 보냈습니다: {position:.2f}")
            else:
                self.get_logger().error("모터 이름을 지정해야 합니다.")
                # 사용 가능한 모터 목록 출력
                if self.motors_info:
                    self.get_logger().error("사용 가능한 모터 목록:")
                    for motor in self.motors_info:
                        self.get_logger().error(f"  - {motor.get('name')} (노드ID: {motor.get('node_id')})")
            
        else:
            self.get_logger().error(f"알 수 없는 명령 유형: {command_type}")
            self.get_logger().error("사용 가능한 명령 유형: command, position")

def print_usage():
    print("사용법: ros2 run ros2_interface send_command.py <명령> [값] [모터이름]")
    print("  명령: command, position")
    print("  예시: ros2 run ros2_interface send_command.py command reset")
    print("  예시: ros2 run ros2_interface send_command.py position 1.57 j_l1")
    
    # 사용 가능한 모터 목록 출력
    motors_info = load_motors_info()
    if motors_info:
        print("사용 가능한 모터 목록:")
        for motor in motors_info:
            print(f"  - {motor.get('name')} (노드ID: {motor.get('node_id')})")

def main(args=None):
    if len(sys.argv) < 2:
        print_usage()
        return
        
    rclpy.init(args=args)
    
    command_type = sys.argv[1]
    command_args = sys.argv[2:]
    
    node = CommandSender(command_type, command_args)
    
    try:
        # 명령이 전송될 시간을 줍니다
        rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 