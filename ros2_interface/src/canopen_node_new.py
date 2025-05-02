#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import sys
import os
import json
import subprocess
import time
import canopen
import traceback
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# common_canopen 패키지 경로를 시스템 경로에 추가
# 여러 경로를 시도하여 모듈을 찾음
def find_common_canopen():
    # 실행 파일의 경로 기준으로 share 디렉토리 찾기
    exec_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 가능한 경로들
    possible_paths = [
        # 설치된 패키지에서 찾기
        os.path.join(os.path.dirname(os.path.dirname(exec_dir)), 'share', 'ros2_interface', 'common_canopen'),
        # 상위 디렉토리에서 직접 찾기
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(exec_dir))), 'common_canopen'),
        # 현재 작업 디렉토리 기준
        os.path.join(os.getcwd(), 'common_canopen')
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            print(f"common_canopen 경로 발견: {path}")
            return path
    
    return None

common_canopen_path = find_common_canopen()

if common_canopen_path:
    # 경로를 시스템 경로에 추가
    sys.path.append(common_canopen_path)
    sys.path.append(os.path.dirname(common_canopen_path))  # 상위 디렉토리도 추가
    
    # canopen_motor 경로도 추가
    canopen_motor_path = os.path.join(common_canopen_path, 'canopen_motor')
    if os.path.exists(canopen_motor_path):
        sys.path.append(canopen_motor_path)
    
    print(f"추가된 경로: {common_canopen_path}")
    print(f"현재 위치: {os.path.abspath(__file__)}")
    print(f"모듈 검색 경로: {sys.path}")
    
    try:
        # 디렉토리 내용 확인
        print(f"common_canopen 디렉토리 내용: {os.listdir(common_canopen_path)}")
        
        # 직접 임포트 시도
        from canopen_motor.motor_manager.motor_controller import MotorController
        from canopen_motor.motor_manager.motor_factory import MotorFactory
        print("성공적으로 모듈을 임포트했습니다!")
    except Exception as e:
        print(f"직접 임포트 실패: {type(e).__name__}: {e}")
        traceback.print_exc()
        
        # 대체 임포트 시도
        try:
            import common_canopen.canopen_motor.motor_manager.motor_controller as module1
            import common_canopen.canopen_motor.motor_manager.motor_factory as module2
            MotorController = module1.MotorController
            MotorFactory = module2.MotorFactory
            print("대체 방법으로 모듈 임포트 성공!")
        except Exception as e2:
            print(f"대체 임포트도 실패: {type(e2).__name__}: {e2}")
            traceback.print_exc()
            sys.exit(1)
else:
    print("오류: common_canopen 모듈을 찾을 수 없습니다.")
    sys.exit(1)

def run_command(command):
    """명령어를 실행하고 결과를 반환합니다."""
    try:
        result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return True, result.stdout
    except subprocess.CalledProcessError as e:
        return False, e.stderr

def is_can_interface_up(interface):
    """CAN 인터페이스가 활성화되어 있는지 확인합니다."""
    success, output = run_command(f"ip link show {interface}")
    if not success:
        return False
    
    return "UP" in output and "state UP" in output

def setup_can_interface(interface, bitrate=1000000, txqueuelen=1000, logger=None):
    """CAN 인터페이스를 설정합니다."""
    # 권한 확인
    is_root = os.geteuid() == 0
    if not is_root:
        if logger:
            logger.warn("CAN 인터페이스 설정에는 root 권한이 필요합니다.")
            logger.warn("다음 명령을 수동으로 실행하거나, sudo로 노드를 실행하세요:")
            logger.warn("sudo slcand -o -s8 /dev/ttyACM0 can0")
            logger.warn(f"sudo ip link set {interface} up")
            logger.warn(f"sudo ip link set {interface} txqueuelen {txqueuelen}")
        
        # 인터페이스가 이미 활성화되어 있는지 확인
        if is_can_interface_up(interface):
            if logger:
                logger.info(f"CAN 인터페이스 {interface}가 이미 활성화되어 있습니다.")
            return True
        else:
            if logger:
                logger.warn(f"CAN 인터페이스 {interface}가 활성화되어 있지 않습니다.")
            return False
    
    # 인터페이스가 이미 활성화되어 있는지 확인
    if is_can_interface_up(interface):
        if logger:
            logger.info(f"CAN 인터페이스 {interface}가 이미 활성화되어 있습니다.")
        return True
    
    # CANable2 설정 (slcan 사용)
    success, output = run_command("slcand -o -s8 /dev/canable can0")
    if not success:
        if logger:
            logger.warn(f"slcand 실행 실패: {output}")
            logger.info("이미 실행 중이거나 장치에 문제가 있을 수 있습니다.")
    else:
        if logger:
            logger.info("slcand 실행 성공")
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
            if logger:
                logger.error(f"CAN 인터페이스 설정 실패: {output}")
            return False
    
    # 설정 확인
    success, output = run_command("ip link show")
    if success and logger:
        logger.info(f"CAN 인터페이스 설정 결과:\n{output}")
    
    return is_can_interface_up(interface)

class CANopenManager(Node):
    def __init__(self):
        super().__init__('canopen_manager')
        
        SYNC_INTERVAL = 0.01
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('heartbeat_interval', 1000)  # ms
        self.declare_parameter('can_bitrate', 1000000)  # 1Mbps
        self.declare_parameter('can_txqueuelen', 1000)
        
        # 파라미터 가져오기
        self.can_interface = self.get_parameter('can_interface').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.can_txqueuelen = self.get_parameter('can_txqueuelen').value
        
        # 모터 상태 캐싱 변수
        self.motors_status_ok = True  # 모든 모터가 정상 상태인지 여부
        self.motors_status_cache = {}  # 모터 상태 캐시
        self.last_status_check_time = self.get_clock().now()
        self.status_check_interval = rclpy.duration.Duration(seconds=0.5)  # 0.5초마다 상태 확인
        
        # CAN 인터페이스 설정
        if setup_can_interface(self.can_interface, self.can_bitrate, self.can_txqueuelen, logger=self.get_logger()):
            self.get_logger().info(f"CAN 인터페이스 {self.can_interface} 설정 완료")
        else:
            self.get_logger().warn(f"CAN 인터페이스 {self.can_interface} 설정 실패, 계속 진행합니다.")
        
        # JSON 파일에서 모터 정보 가져오기
        self.motors_info = self.load_motors_info()
        
        # 발행자 설정
        self.joint_state_pub = self.create_publisher(JointState, 'canopen/joint_states', 10)
        
        # 서비스 등록
        self.motor_status_service = self.create_service(Trigger, 'canopen/check_motors', self.handle_motor_status_check)
        
        # 구독자 설정
        self.multiple_joints_sub = self.create_subscription(
            JointState, 
            'canopen/multiple_joints', 
            self.multiple_joints_callback, 
            10
        )
        
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
                self.create_subscription(
                    Float64,
                    f'canopen/single_motor/{motor_name}/position',
                    lambda msg, id=node_id: self.single_motor_position_callback(msg, id),
                    10
                )
            
            # 모터 초기화
            self.motor_controller.all_motors_init_start(interval=SYNC_INTERVAL)
            
            self.get_logger().info("모터 초기화 완료")
        except Exception as e:
            self.get_logger().error(f"모터 초기화 실패: {str(e)}")
            self.get_logger().error(f"예외 타입: {type(e).__name__}")
            self.get_logger().error(f"스택 트레이스: {traceback.format_exc()}")
            self.motor_controller = None
        
        # 타이머 설정 (상태 발행용)
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10Hz
        
        self.get_logger().info("CANopen Manager 노드가 시작되었습니다.")
        self.get_logger().info(f"CAN 인터페이스: {self.can_interface}, 하트비트 간격: {self.heartbeat_interval} ms")
        
        # 모터 정보 출력
        for motor_info in self.motors_info:
            self.get_logger().info(f"모터 등록: 이름={motor_info['name']}, 노드ID={motor_info['node_id']}, 제조사={motor_info['vendor_type']}")
    
    def load_motors_info(self):
        """JSON 파일에서 모터 정보를 로드합니다."""
        try:
            # 설치된 config 디렉토리에서 파일 찾기
            ros_share_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', '..', 'share', 'ros2_interface', 'config')
            json_path = os.path.join(ros_share_path, 'canopen_info.json')
            
            # 파일이 존재하지 않으면 개발 환경에서 찾기
            if not os.path.exists(json_path):
                json_path = os.path.join(common_canopen_path, 'canopen_info_json', 'canopen_info.json')
            
            with open(json_path, 'r') as f:
                data = json.load(f)
                return data.get('motors', [])
        except Exception as e:
            self.get_logger().error(f"모터 정보 로드 실패: {str(e)}")
            return []
    
    def multiple_joints_callback(self, msg):
        """여러 관절의 위치를 동시에 제어하는 콜백 함수"""
        if not self.motor_controller:
            self.get_logger().warn("모터 컨트롤러가 초기화되지 않았습니다.")
            return
            
        # 캐싱된 모터 상태를 확인
        if not self.motors_status_ok:
            # 문제가 있는 모터 정보 찾아서 로그 출력
            for node_id, status in self.motors_status_cache.items():
                if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                    motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
                    motor_name = motor_info['name'] if motor_info else f"unknown_motor_{node_id}"
                    self.get_logger().error(f"모터 {motor_name}(ID:{node_id})에 문제가 발생하여 모든 위치 명령을 무시합니다. 상태워드: 0x{status.get('statusword', 0):04X}")
                    break
            return  # 명령 전달 중단
        
        # 관절 목록 순회
        for i, joint_name in enumerate(msg.name):
            # 해당 모터 찾기
            motor_info = next((m for m in self.motors_info if m['name'] == joint_name), None)
            if motor_info:
                node_id = motor_info['node_id']
                position = msg.position[i]
                
                # 위치 명령 전송
                try:
                    self.motor_controller.set_target_position(node_id, position)
                except Exception as e:
                    self.get_logger().error(f"모터 {joint_name}(ID:{node_id})에 위치 명령 전송 실패: {str(e)}")
    
    def single_motor_position_callback(self, msg, node_id):
        """단일 모터에 위치 명령을 전송하는 콜백 함수"""
        if not self.motor_controller:
            self.get_logger().warn("모터 컨트롤러가 초기화되지 않았습니다.")
            return
            
        # 캐싱된 모터 상태를 확인
        motor_status = self.motors_status_cache.get(node_id, {})
        if motor_status.get('error', False) or motor_status.get('disabled', False) or not motor_status.get('active', True):
            motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
            motor_name = motor_info['name'] if motor_info else f"unknown_motor_{node_id}"
            self.get_logger().error(f"모터 {motor_name}(ID:{node_id})에 문제가 발생하여 위치 명령을 무시합니다. 상태워드: 0x{motor_status.get('statusword', 0):04X}")
            return
        
        # 위치 명령 전송
        try:
            self.motor_controller.set_target_position(node_id, msg.data)
            motor_info = next((m for m in self.motors_info if m['node_id'] == node_id), None)
            if motor_info:
                self.get_logger().info(f"모터 {motor_info['name']}(ID:{node_id})에 위치 명령 전송: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"모터 ID:{node_id}에 위치 명령 전송 실패: {str(e)}")
    
    def publish_status(self):
        """모터 상태를 발행하는 함수"""
        if not self.motor_controller:
            return
            
        try:
            # 모터 상태 확인
            current_time = self.get_clock().now()
            if current_time - self.last_status_check_time > self.status_check_interval:
                self.last_status_check_time = current_time
                
                # 모든 모터의 상태 확인 및 캐싱
                all_motors_ok = True
                for motor_info in self.motors_info:
                    node_id = motor_info['node_id']
                    
                    try:
                        # 모터 상태 읽기
                        status = self.motor_controller.get_motor_status(node_id)
                        
                        # 상태 캐싱
                        self.motors_status_cache[node_id] = status
                        
                        # 문제가 있는 모터가 있으면 플래그 설정
                        if status.get('error', False) or status.get('disabled', False) or not status.get('active', True):
                            all_motors_ok = False
                    except Exception as e:
                        self.get_logger().error(f"모터 ID:{node_id} 상태 읽기 실패: {str(e)}")
                        all_motors_ok = False
                
                # 모든 모터 상태 업데이트
                self.motors_status_ok = all_motors_ok
            
            # JointState 메시지 생성
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            
            try:
                # 모든 모터의 위치, 속도, 토크 정보 가져오기
                positions = self.motor_controller.get_positions()
                velocities = self.motor_controller.get_velocities()
                torques = self.motor_controller.get_torques()  # 토크 정보 가져오기
                
                # 각 모터의 위치, 속도, 토크 정보 저장
                for motor_info in self.motors_info:
                    node_id = motor_info['node_id']
                    motor_name = motor_info['name']
                    
                    if node_id in positions:
                        position = positions[node_id]
                        velocity = velocities.get(node_id, 0.0)  # 속도 정보가 없으면 0으로 설정
                        torque = torques.get(node_id, 0.0)  # 토크 정보가 없으면 0으로 설정
                        
                        # 메시지에 추가
                        joint_state.name.append(motor_name)
                        joint_state.position.append(position)
                        joint_state.velocity.append(velocity)
                        joint_state.effort.append(torque)  # 토크 정보 추가

                        #elf.get_logger().warning(f"모터 {motor_name}(ID:{node_id}) 위치: {position}, 속도: {velocity}, 토크: {torque}")
                    else:
                        self.get_logger().warning(f"모터 ID:{node_id} 정보를 찾을 수 없음")
            except Exception as e:
                self.get_logger().warning(f"모터 정보 읽기 실패: {str(e)}")
            
            # 메시지 발행
            if joint_state.name:
                self.joint_state_pub.publish(joint_state)
                self.get_logger().warning(f"토픽 발행 - joint_states: {len(joint_state.name)}개 모터 정보")
        except Exception as e:
            self.get_logger().error(f"상태 발행 중 오류 발생: {str(e)}")
    
    def handle_motor_status_check(self, request, response):
        """모터 상태 확인 서비스 핸들러"""
        if not self.motor_controller:
            response.success = False
            response.message = "모터 컨트롤러가 초기화되지 않았습니다."
            return response
        
        try:
            # 모든 모터의 상태 확인
            all_motors_ok = True
            status_msg = []
            
            for motor_info in self.motors_info:
                node_id = motor_info['node_id']
                motor_name = motor_info['name']
                
                try:
                    # 모터 상태 읽기
                    status = self.motor_controller.get_motor_status(node_id)
                    
                    # 상태 문자열 생성
                    status_str = f"모터 {motor_name}(ID:{node_id}): "
                    
                    if status.get('error', False):
                        status_str += "오류 발생"
                        all_motors_ok = False
                    elif status.get('disabled', False):
                        status_str += "비활성화됨"
                        all_motors_ok = False
                    elif not status.get('active', True):
                        status_str += "미응답"
                        all_motors_ok = False
                    else:
                        status_str += "정상"
                    
                    status_str += f" (상태워드: 0x{status.get('statusword', 0):04X})"
                    status_msg.append(status_str)
                except Exception as e:
                    status_str = f"모터 {motor_name}(ID:{node_id}): 통신 오류 - {str(e)}"
                    status_msg.append(status_str)
                    all_motors_ok = False
            
            # 응답 생성
            response.success = all_motors_ok
            response.message = "\n".join(status_msg)
            return response
        except Exception as e:
            response.success = False
            response.message = f"상태 확인 중 오류 발생: {str(e)}"
            return response

def main(args=None):
    rclpy.init(args=args)
    node = CANopenManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 모터 컨트롤러 종료
        if hasattr(node, 'motor_controller') and node.motor_controller:
            try:
                # 모든 모터 비활성화
                node.motor_controller.all_motors_disable()
                node.get_logger().info("모든 모터가 비활성화되었습니다.")
            except Exception as e:
                node.get_logger().error(f"모터 비활성화 중 오류 발생: {str(e)}")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 