#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import json
import subprocess
import time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState

# 상대 경로를 사용하여 모듈 임포트
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from canopen_motor.motor_manager.motor_controller import MotorController
from canopen_motor.motor_manager.motor_factory import MotorFactory

def run_command(command):
    """
    명령어를 실행하고 결과를 반환합니다.
    """
    try:
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return True, result.stdout
    except subprocess.CalledProcessError as e:
        return False, e.stderr

def is_can_interface_up(interface):
    """
    CAN 인터페이스가 활성화되어 있는지 확인합니다.
    """
    success, output = run_command(f"ip link show {interface}")
    if not success:
        return False
    
    return "UP" in output and "state UP" in output

def setup_can_interface(interface, bitrate=1000000, txqueuelen=1000):
    """
    CAN 인터페이스를 설정합니다.
    """
    # 권한 확인
    is_root = os.geteuid() == 0
    if not is_root:
        rospy.logwarn("CAN 인터페이스 설정에는 root 권한이 필요합니다.")
        rospy.logwarn("다음 명령을 수동으로 실행하거나, sudo로 노드를 실행하세요:")
        rospy.logwarn("sudo slcand -o -s8 /dev/ttyACM0 can0")
        rospy.logwarn(f"sudo ip link set {interface} up")
        rospy.logwarn(f"sudo ip link set {interface} txqueuelen {txqueuelen}")
        
        # 인터페이스가 이미 활성화되어 있는지 확인
        if is_can_interface_up(interface):
            rospy.loginfo("CAN 인터페이스 %s가 이미 활성화되어 있습니다.", interface)
            return True
        else:
            rospy.logwarn("CAN 인터페이스 %s가 활성화되어 있지 않습니다.", interface)
            return False
    
    # 인터페이스가 이미 활성화되어 있는지 확인
    if is_can_interface_up(interface):
        rospy.loginfo("CAN 인터페이스 %s가 이미 활성화되어 있습니다.", interface)
        return True
    
    # CANable2 설정 (slcan 사용)
    success, output = run_command("slcand -o -s8 /dev/ttyACM0 can0")
    if not success:
        rospy.logwarn("slcand 실행 실패: %s", output)
        rospy.loginfo("이미 실행 중이거나 장치에 문제가 있을 수 있습니다.")
    else:
        rospy.loginfo("slcand 실행 성공")
        # slcand가 인터페이스를 설정할 시간을 줍니다
        time.sleep(1)
    
    # 인터페이스 설정
    commands = [
        f"ip link set {interface} up",
        f"ip link set {interface} txqueuelen {txqueuelen}"
    ]
    
    for cmd in commands:
        success, output = run_command(cmd)
        if not success:
            rospy.logerr("CAN 인터페이스 설정 실패: %s", output)
            return False
    
    # 설정 확인
    success, output = run_command("ip link show")
    if success:
        rospy.loginfo("CAN 인터페이스 설정 결과:\n%s", output)
    
    return is_can_interface_up(interface)

class CANopenManager:
    def __init__(self):
        SYNC_INTERVAL = 0.01
        
        rospy.init_node('canopen_manager', anonymous=False)
        
        # 파라미터 가져오기
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.heartbeat_interval = rospy.get_param('~heartbeat_interval', 1000)  # ms
        self.can_bitrate = rospy.get_param('~can_bitrate', 1000000)  # 1Mbps
        self.can_txqueuelen = rospy.get_param('~can_txqueuelen', 1000)
        
        # CAN 인터페이스 설정
        if setup_can_interface(self.can_interface, self.can_bitrate, self.can_txqueuelen):
            rospy.loginfo("CAN 인터페이스 %s 설정 완료", self.can_interface)
        else:
            rospy.logwarn("CAN 인터페이스 %s 설정 실패, 계속 진행합니다.", self.can_interface)
        
        # JSON 파일에서 모터 정보 가져오기
        self.motors_info = self.load_motors_info()
        
        # 발행자 설정
        self.joint_state_pub = rospy.Publisher('canopen/joint_states', JointState, queue_size=10)
        
        # 구독자 설정
        rospy.Subscriber('canopen/multiple_joints', JointState, self.multiple_joints_callback)
        
        # 모터 컨트롤러 초기화
        try:
            self.motor_controller = MotorController(channel=self.can_interface)
            
            # JSON에서 가져온 모터 정보로 모터 초기화
            for motor_info in self.motors_info:
                motor = MotorFactory.create_motor(motor_info)
                self.motor_controller.add_motor(motor)
                
                # 모터별 위치 명령 구독자 설정
                motor_name = motor_info['name']
                node_id = motor_info['node_id']
                
                # 위치 명령 구독자 설정
                rospy.Subscriber(
                    f'canopen/single_motor/{motor_name}/position',
                    Float64,
                    lambda msg, id=node_id: self.single_motor_position_callback(msg, id)
                )
            
            # 모터 초기화
            self.motor_controller.all_motors_init_start(interval=SYNC_INTERVAL)
            
            rospy.loginfo("모터 초기화 완료")
        except Exception as e:
            rospy.logerr("모터 초기화 실패: %s", str(e))
            self.motor_controller = None
        
        self.rate = rospy.Rate(10)  # 10Hz
        rospy.loginfo("CANopen Manager 노드가 시작되었습니다.")
        rospy.loginfo("CAN 인터페이스: %s, 하트비트 간격: %d ms", 
                     self.can_interface, self.heartbeat_interval)
        
        # 모터 정보 출력
        for motor_info in self.motors_info:
            rospy.loginfo("모터 등록: 이름=%s, 노드ID=%d, 제조사=%s", 
                         motor_info['name'], motor_info['node_id'], motor_info['vendor_type'])
    
    def load_motors_info(self):
        """JSON 파일에서 모터 정보를 로드합니다."""
        try:
            # 패키지 경로 가져오기
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            json_path = os.path.join(package_path, 'canopen_info_json', 'canopen_info.json')
            
            with open(json_path, 'r') as f:
                data = json.load(f)
                return data.get('motors', [])
        except Exception as e:
            rospy.logerr("모터 정보 로드 실패: %s", str(e))
            return []
    
    def multiple_joints_callback(self, msg):
        """
        여러 관절의 위치를 동시에 제어하는 콜백 함수
        """
        if not self.motor_controller:
            rospy.logwarn("모터 컨트롤러가 초기화되지 않았습니다.")
            return

        try:
            # JointState 메시지의 각 관절에 대해 위치 명령을 처리
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position):
                    # 모터 정보에서 해당 관절의 node_id 찾기
                    motor_info = next((m for m in self.motors_info if m['name'] == joint_name), None)
                    if motor_info:
                        node_id = motor_info['node_id']
                        position = msg.position[i]
                        self.motor_controller.set_position(node_id, position)
                        # rospy.loginfo("위치 명령 전송: 관절=%s, 노드ID=%d, 위치=%.2f", 
                        #             joint_name, node_id, position)
                    else:
                        rospy.logwarn("알 수 없는 관절 이름: %s", joint_name)
        except Exception as e:
            rospy.logerr("위치 명령 처리 중 오류 발생: %s", str(e))
        
    def single_motor_position_callback(self, msg, node_id):
        """
        단일 모터의 위치를 제어하는 콜백 함수
        """
        if self.motor_controller:
            try:
                self.motor_controller.set_position(node_id, msg.data)
                #rospy.loginfo("위치 명령 전송: 노드ID=%d, 위치=%.2f", node_id, msg.data)
            except Exception as e:
                rospy.logerr("위치 명령 전송 실패: %s", str(e))
        
    def publish_status(self):
        """
        CANopen 상태를 JointState 메시지로 발행합니다
        """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = "base_link"  # 프레임 ID 설정
        
        if self.motor_controller:
            try:
                positions = self.motor_controller.get_positions()
                velocities = self.motor_controller.get_velocities()
                torques = self.motor_controller.get_torques()  # 토크 정보 가져오기
                
                # JointState 메시지 생성
                joint_names = []
                joint_positions = []
                joint_velocities = []
                joint_efforts = []  # 토크 정보를 저장할 리스트
                
                # 각 모터의 위치, 속도, 토크 정보 저장
                for motor_info in self.motors_info:
                    node_id = motor_info['node_id']
                    motor_name = motor_info['name']
                    
                    if node_id in positions:
                        position = positions[node_id]
                        velocity = velocities.get(node_id, 0.0)  # 속도 정보가 없으면 0으로 설정
                        torque = torques.get(node_id, 0.0)  # 토크 정보가 없으면 0으로 설정
                        
                        joint_names.append(motor_name)
                        joint_positions.append(position)
                        joint_velocities.append(velocity)
                        joint_efforts.append(torque)  # 토크 정보 추가
                
                # JointState 메시지 채우기
                joint_state_msg.name = joint_names
                joint_state_msg.position = joint_positions
                joint_state_msg.velocity = joint_velocities
                joint_state_msg.effort = joint_efforts  # 토크 정보 설정
                
                # 상태가 정상인 경우 로그 출력
                if joint_names:
                    rospy.logdebug("CANopen 장치 상태: 정상")
                
            except Exception as e:
                rospy.logerr("CANopen 장치 상태: 오류 (%s)", str(e))
        else:
            rospy.logdebug("CANopen 장치 상태: 초기화되지 않음")
        
        # JointState 메시지 발행
        if joint_state_msg.name:  # 이름 배열이 비어있지 않은 경우에만 발행
            self.joint_state_pub.publish(joint_state_msg)
        
    def run(self):
        """
        메인 실행 루프
        """
        while not rospy.is_shutdown():
            self.publish_status()
            self.rate.sleep()
            
    def shutdown(self):
        """
        종료 처리
        """
        if self.motor_controller:
            self.motor_controller.disconnect()
            rospy.loginfo("모터 컨트롤러 연결 해제")

if __name__ == '__main__':
    try:
        canopen_manager = CANopenManager()
        rospy.on_shutdown(canopen_manager.shutdown)
        canopen_manager.run()
    except rospy.ROSInterruptException:
        pass 