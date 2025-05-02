#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import json
import subprocess
import time
import canopen
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

# common_canopen 패키지의 모듈 임포트
import rospkg
rospack = rospkg.RosPack()
common_canopen_path = rospack.get_path('common_canopen')
sys.path.append(common_canopen_path)

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
    success, output = run_command("slcand -o -s8 /dev/canable can0")
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
        
        # 모터 상태 캐싱 변수
        self.motors_status_ok = True  # 모든 모터가 정상 상태인지 여부
        self.motors_status_cache = {}  # 모터 상태 캐시
        self.last_status_check_time = rospy.Time.now()
        self.status_check_interval = rospy.Duration(0.5)  # 0.5초마다 상태 확인
        
        # CAN 인터페이스 설정
        if setup_can_interface(self.can_interface, self.can_bitrate, self.can_txqueuelen):
            rospy.loginfo("CAN 인터페이스 %s 설정 완료", self.can_interface)
        else:
            rospy.logwarn("CAN 인터페이스 %s 설정 실패, 계속 진행합니다.", self.can_interface)
        
        # JSON 파일에서 모터 정보 가져오기
        self.motors_info = self.load_motors_info()
        
        # 발행자 설정
        self.joint_state_pub = rospy.Publisher('canopen/joint_states', JointState, queue_size=10)
        
        # 서비스 등록
        self.motor_status_service = rospy.Service('canopen/check_motors', Trigger, self.handle_motor_status_check)
        
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
            # common_canopen 패키지 경로 사용
            json_path = os.path.join(common_canopen_path, 'canopen_info_json', 'canopen_info.json')
            
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
            
        # 캐싱된 모터 상태를 확인
        if not self.motors_status_ok:
            # 문제가 있는 모터 정보 찾아서 로그 출력
            for node_id, status in self.motors_status_cache.items():
                if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                    motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
                    motor_name = motor_info['name'] if motor_info else f"unknown_motor_{node_id}"
                    rospy.logerr(f"모터 {motor_name}(ID:{node_id})에 문제가 발생하여 모든 위치 명령을 무시합니다. 상태워드: 0x{status.get('statusword', 0):04X}")
                    break
            return  # 명령 전달 중단

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
                        #                 joint_name, node_id, position)
                    else:
                        rospy.logwarn("알 수 없는 관절 이름: %s", joint_name)
        except Exception as e:
            rospy.logerr("위치 명령 처리 중 오류 발생: %s", str(e))
        
    def single_motor_position_callback(self, msg, node_id):
        """
        단일 모터의 위치를 제어하는 콜백 함수
        """
        if not self.motor_controller:
            rospy.logwarn("모터 컨트롤러가 초기화되지 않았습니다.")
            return
            
        # 캐싱된 모터 상태를 확인
        if not self.motors_status_ok:
            # 현재 명령을 받는 모터 이름 찾기
            target_motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
            target_motor_name = target_motor_info['name'] if target_motor_info else f"unknown_motor_{node_id}"
            
            # 문제가 있는 모터 정보 찾아서 로그 출력
            for motor_id, status in self.motors_status_cache.items():
                if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                    motor_info = next((m for m in self.motors_info if m['node_id'] == motor_id), None)
                    motor_name = motor_info['name'] if motor_info else f"unknown_motor_{motor_id}"
                    rospy.logerr(f"모터 {motor_name}(ID:{motor_id})에 문제가 발생하여 {target_motor_name}(ID:{node_id})에 대한 위치 명령을 무시합니다.")
                    break
            return  # 명령 전달 중단
        
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
                # 모터 상태 체크
                all_motors_ok = True
                if hasattr(self.motor_controller, 'get_all_motors_status'):
                    all_statuses = self.motor_controller.get_all_motors_status()
                    
                    # 각 모터 상태 확인
                    for node_id, status in all_statuses.items():
                        # 문제가 있는 모터 찾기
                        if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                            all_motors_ok = False
                            
                            # 모터 이름 찾기
                            motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
                            motor_name = motor_info['name'] if motor_info else f"unknown_motor_{node_id}"
                            
                            # 문제 원인 파악
                            error_reasons = []
                            if status.get('error', False):
                                error_reasons.append(f"고장 발생(statusword: 0x{status.get('statusword', 0):04X})")
                            if status.get('disabled', False):
                                error_reasons.append("비활성화됨")
                            if not status.get('active', True):
                                error_reasons.append("동작 중지")
                                
                            # 에러 로그 출력
                            if error_reasons:
                                rospy.logerr(f"모터 {motor_name}(ID:{node_id}) 문제 발생: {', '.join(error_reasons)}")
                
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
                if joint_names and all_motors_ok:
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
            current_time = rospy.Time.now()
            
            # 주기적으로 모터 상태 확인 (0.5초마다)
            if current_time - self.last_status_check_time >= self.status_check_interval:
                self.last_status_check_time = current_time
                
                # 모터 상태 확인 및 캐싱
                if self.motor_controller and hasattr(self.motor_controller, 'get_all_motors_status'):
                    try:
                        all_statuses = self.motor_controller.get_all_motors_status()
                        self.motors_status_cache = all_statuses
                        
                        # 모터 상태 확인
                        self.motors_status_ok = True
                        for node_id, status in all_statuses.items():
                            if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                                self.motors_status_ok = False
                                
                                # 문제있는 모터 정보 로깅
                                motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
                                motor_name = motor_info['name'] if motor_info else f"unknown_motor_{node_id}"
                                rospy.logerr(f"모터 {motor_name}(ID:{node_id})에 문제가 발생했습니다. 모든 명령이 무시됩니다. 상태워드: 0x{status.get('statusword', 0):04X}")
                                break
                        
                        # 모터 상태 확인 및 문제 발생 시 모두 비활성화 (disable_all_motors 호출 제거)
                        # 메시지 처리가 이미 차단되므로 추가 비활성화는 필요 없음
                    except Exception as e:
                        rospy.logerr("모터 상태 확인 중 오류 발생: %s", str(e))
                        self.motors_status_ok = False  # 에러 발생 시 안전하게 False로 설정
            
            # 정기적인 상태 발행
            self.publish_status()
            self.rate.sleep()
            
    def shutdown(self):
        """
        종료 처리
        """
        if self.motor_controller:
            self.motor_controller.disconnect()
            rospy.loginfo("모터 컨트롤러 연결 해제")

    def get_motor_status(self, req):
        """
        CANopen 장치의 상태를 반환하는 서비스 함수
        """
        response = TriggerResponse()
        
        if not self.motor_controller:
            response.success = False
            response.message = "모터 컨트롤러가 초기화되지 않았습니다."
            return response
            
        try:
            # 모든 모터의 상태 정보 가져오기
            motor_statuses = {}
            if hasattr(self.motor_controller, 'get_all_motors_status'):
                all_statuses = self.motor_controller.get_all_motors_status()
                
                # 각 모터 상태 처리
                for node_id, status in all_statuses.items():
                    # 모터 이름 찾기
                    motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
                    motor_name = motor_info['name'] if motor_info else f"unknown_motor_{node_id}"
                    
                    # 상태 정보 가공
                    motor_statuses[motor_name] = {
                        'node_id': node_id,
                        'position': status.get('position', 0),
                        'velocity': status.get('velocity', 0),
                        'torque': status.get('torque', 0),
                        'error': status.get('error', False),
                        'disabled': status.get('disabled', False),
                        'active': status.get('active', False),
                        'warning': status.get('warning', False),
                        'statusword': f"0x{status.get('statusword', 0):04X}"
                    }
            
            # 모터 에러 여부 확인
            has_error = any(status.get('error', False) for status in motor_statuses.values())
            has_disabled = any(status.get('disabled', False) for status in motor_statuses.values())
            all_active = all(status.get('active', False) for status in motor_statuses.values())
            
            # 응답 메시지 생성
            status_summary = "정상" if all_active and not has_error and not has_disabled else "문제 발생"
            details = []
            
            if has_error:
                details.append("하나 이상의 모터에 고장이 발생했습니다.")
            if has_disabled:
                details.append("하나 이상의 모터가 비활성화되었습니다.")
            if not all_active:
                details.append("하나 이상의 모터가 동작 상태가 아닙니다.")
                
            response.success = all_active and not has_error and not has_disabled
            response.message = f"모터 상태: {status_summary}" + (f" - {'; '.join(details)}" if details else "")
            
            # 상세 상태 정보도 메시지에 추가
            status_details = []
            for name, status in motor_statuses.items():
                status_text = f"{name}(ID:{status['node_id']}): "
                if status['error']:
                    status_text += "고장 발생! "
                elif status['disabled']:
                    status_text += "비활성화됨! "
                elif not status['active']:
                    status_text += "동작 중지! "
                else:
                    status_text += "정상 동작중 "
                
                status_text += f"[위치:{status['position']:.2f}, 속도:{status['velocity']:.2f}, 토크:{status['torque']:.2f}]"
                status_details.append(status_text)
            
            response.message += "\n" + "\n".join(status_details)
            
            return response
        except Exception as e:
            response.success = False
            response.message = f"모터 상태 확인 중 오류 발생: {str(e)}"
            return response

    def handle_motor_status_check(self, req):
        """
        CANopen 장치의 상태를 반환하는 서비스 함수
        """
        return self.get_motor_status(req)

if __name__ == '__main__':
    try:
        canopen_manager = CANopenManager()
        rospy.on_shutdown(canopen_manager.shutdown)
        canopen_manager.run()
    except rospy.ROSInterruptException:
        pass 