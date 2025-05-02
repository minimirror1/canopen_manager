#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import json
from std_msgs.msg import String, Float64
import sys
import rospkg

def load_motors_info():
    """JSON 파일에서 모터 정보를 로드합니다."""
    try:
        # common_canopen 패키지 경로 사용
        rospack = rospkg.RosPack()
        common_canopen_path = rospack.get_path('common_canopen')
        json_path = os.path.join(common_canopen_path, 'canopen_info_json', 'canopen_info.json')
        
        with open(json_path, 'r') as f:
            data = json.load(f)
            return data.get('motors', [])
    except Exception as e:
        rospy.logerr("모터 정보 로드 실패: %s", str(e))
        return []

def get_motor_by_name(motors_info, name):
    """이름으로 모터 정보를 찾습니다."""
    for motor in motors_info:
        if motor.get('name') == name:
            return motor
    return None

def send_command():
    # 노드 초기화
    rospy.init_node('canopen_command_sender', anonymous=True)
    
    # 모터 정보 로드
    motors_info = load_motors_info()
    
    # 명령행 인수 확인
    if len(sys.argv) < 2:
        rospy.logerr("사용법: rosrun canopen_manager send_command.py <명령> [값] [모터이름]")
        rospy.logerr("  명령: command, position")
        rospy.logerr("  예시: rosrun canopen_manager send_command.py command reset")
        rospy.logerr("  예시: rosrun canopen_manager send_command.py position 1.57 j_l1")
        
        # 사용 가능한 모터 목록 출력
        if motors_info:
            rospy.logerr("사용 가능한 모터 목록:")
            for motor in motors_info:
                rospy.logerr("  - %s (노드ID: %d)", motor.get('name'), motor.get('node_id'))
        return
    
    command_type = sys.argv[1]
    
    if command_type == "command":
        if len(sys.argv) < 3:
            rospy.logerr("명령을 지정해야 합니다.")
            return
            
        # 발행자 설정
        pub = rospy.Publisher('canopen/command', String, queue_size=10)
        
        # 명령 메시지 생성
        command = ' '.join(sys.argv[2:])
        msg = String()
        msg.data = command
        
        # 발행자가 연결될 때까지 잠시 대기
        rospy.sleep(1)
        
        # 메시지 발행
        pub.publish(msg)
        rospy.loginfo("명령을 보냈습니다: %s", command)
        
    elif command_type == "position":
        if len(sys.argv) < 3:
            rospy.logerr("위치 값을 지정해야 합니다.")
            return
            
        try:
            position = float(sys.argv[2])
        except ValueError:
            rospy.logerr("위치 값은 숫자여야 합니다.")
            return
        
        # 모터 이름 확인
        motor_name = sys.argv[3] if len(sys.argv) > 3 else None
        
        if motor_name:
            # 특정 모터에 명령 전송
            motor_info = get_motor_by_name(motors_info, motor_name)
            if not motor_info:
                rospy.logerr("모터 '%s'를 찾을 수 없습니다.", motor_name)
                return
                
            # 발행자 설정
            pub = rospy.Publisher(f'canopen/{motor_name}/set_position', Float64, queue_size=10)
            
            # 위치 메시지 생성
            msg = Float64()
            msg.data = position
            
            # 발행자가 연결될 때까지 잠시 대기
            rospy.sleep(1)
            
            # 메시지 발행
            pub.publish(msg)
            rospy.loginfo("모터 '%s'에 위치 명령을 보냈습니다: %.2f", motor_name, position)
        else:
            rospy.logerr("모터 이름을 지정해야 합니다.")
            # 사용 가능한 모터 목록 출력
            if motors_info:
                rospy.logerr("사용 가능한 모터 목록:")
                for motor in motors_info:
                    rospy.logerr("  - %s (노드ID: %d)", motor.get('name'), motor.get('node_id'))
        
    else:
        rospy.logerr("알 수 없는 명령 유형: %s", command_type)
        rospy.logerr("사용 가능한 명령 유형: command, position")

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass 