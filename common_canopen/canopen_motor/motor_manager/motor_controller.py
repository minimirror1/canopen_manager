import canopen
import time
from .abstract_motor import AbstractMotor

class MotorController:
    """
    하나의 CAN Bus 상에서 여러 모터(Node)를 관리하는 컨트롤러.
    예시: USB-CAN 장치와 연결하고, 제조사별 Motor 객체 등록/호출 등.
    """
    def __init__(self, channel='can0', bustype='socketcan', bitrate=1000000, interface=None):
        """
        :param channel: 예) 'can0', 'pcan0', 'usb0' 또는 'COM3' 등
        :param bustype: canopen 또는 python-can에서 사용하는 bustype 설정
        :param bitrate: CAN Bus 속도
        :param interface: slcan 등의 인터페이스 타입. 설정 시 bustype 대신 사용됨
        """
        self.network = canopen.Network()
        try:
            if interface is None:
                self.network.connect(channel=channel, bustype=bustype, bitrate=bitrate)
            else:
                self.network.connect(interface=interface, channel=channel, bitrate=bitrate)
            print(f"CAN bus connected successfully: channel={channel}, bustype={bustype}, bitrate={bitrate}")
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}")
            raise
        # 등록된 모터 리스트/딕셔너리
        self.motors = {}
        self.name_to_id = {}  # 이름으로 모터 ID를 찾기 위한 매핑 추가

    def add_motor(self, motor: AbstractMotor):
        """MotorController가 관리할 모터를 추가한다."""
        node = self.network.add_node(motor.node_id, motor.eds_path)
        motor.node = node
        motor.network = self.network
        # SDO 타임아웃 값 설정
        node.sdo.RESPONSE_TIMEOUT = 2.0  # 2초로 변경
        # 재시도 횟수 설정
        node.sdo.MAX_RETRIES = 3
        # 요청 전 대기 시간 설정
        # node.sdo.PAUSE_BEFORE_SEND = 5
        self.motors[motor.node_id] = motor
        self.name_to_id[motor.name] = motor.node_id  # 이름과 ID 매핑 저장

    def all_motors_init_start(self, interval=0.01):
        print("Starting motor initialization sequence...")
        
        # 리셋
        self.reset_all()
        time.sleep(0.5)  # 리셋 후 잠시 대기
        
        # 모터 전체 초기화
        self.init_all()
        time.sleep(0.5)  # 초기화 후 잠시 대기
        
        # PDO 매핑 (활성화)
        self.pdo_mapping_all()
        time.sleep(0.5)  # PDO 매핑 후 잠시 대기

        # Switch On
        self.set_switchOn_all()
        time.sleep(0.5)  # Switch On 후 잠시 대기

        # PDO 콜백 등록
        self.pdo_callback_register_all()
        time.sleep(0.5)  # 콜백 등록 후 잠시 대기
    
        # 동기화 시작
        self.sync_start(interval)
        print("Motor initialization sequence completed")

    def init_all(self):
        """등록된 모든 모터를 초기화"""
        for node_id, motor in self.motors.items():
            motor.init()

    def reset_all(self):
        
        self.network.nmt.send_command(0x02)  # Stop
        time.sleep(0.5)
        self.network.nmt.send_command(0x82)  # Reset communication
        time.sleep(1)  # 재설정 후 충분한 대기 시간
        #self.network.nmt.send_command(0x80)  # Reset node
        print("Reset all motors")
    
        """등록된 모든 모터를 리셋"""
        for node_id, motor in self.motors.items():
            motor.reset()            


        # NMT 상태 확인
        try:
            self.network.nmt.send_command(0x01)  # Start
            print("NMT state transition to Operational successful")
        except Exception as e:
            print(f"Failed to transition NMT state: {e}")

    def pdo_mapping_all(self):
        """등록된 모든 모터에 대해 PDO 매핑 설정"""
        for node_id, motor in self.motors.items():
            motor.pdo_mapping()

        # Start remote node
        try:
            self.network.nmt.send_command(0x01)  # NMT 시작 명령 전송
            print('원격 노드 시작 명령을 전송했습니다.')
        except canopen.SdoCommunicationError as e:
            print(f'원격 노드 시작 중 오류 발생: {str(e)}')

    def set_switchOn_all(self):
        for node_id, motor in self.motors.items():
            motor.set_switchOn()

    def pdo_callback_register_all(self):
        for node_id, motor in self.motors.items():
            motor.pdo_callback_register()

    def sync_start(self, interval=0.01):
        print(f"Starting sync with interval: {interval}")
        for node_id, motor in self.motors.items():
            motor.set_dt(interval)

        try:
            self.network.sync.start(interval)
            print(f"Sync started successfully with interval: {interval}")
        except Exception as e:
            print(f"Failed to start sync: {e}")

    def sync_stop(self):
        self.network.sync.stop()        

    def set_position_all(self, value):
        """등록된 모든 모터에 동일 위치를 세팅"""
        for node_id, motor in self.motors.items():
            motor.set_position(value)            
    
    def set_position(self, node_id, value):
        if node_id in self.motors:
            self.motors[node_id].set_position(value)
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def get_positions(self):
        """등록된 모든 모터의 위치를 dict로 반환"""        
        positions = {}
        for node_id, motor in self.motors.items():
            positions[node_id] = motor.get_position()
        return positions
    
    def get_velocities(self):
        """등록된 모든 모터의 속도를 dict로 반환"""        
        velocities = {}
        for node_id, motor in self.motors.items():
            velocities[node_id] = motor.get_velocity()
        return velocities
    
    def set_torque_all(self, value):
        for node_id, motor in self.motors.items():
            motor.set_torque(value)

    def set_torque(self, node_id, value):
        if node_id in self.motors:
            self.motors[node_id].set_torque(value)
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def get_torque_all(self):
        for node_id, motor in self.motors.items():
            motor.get_torque()

    def get_torque(self, node_id):
        if node_id in self.motors:
            return self.motors[node_id].get_torque()
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def get_velocity(self, node_id):
        if node_id in self.motors:
            return self.motors[node_id].get_velocity()
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def get_acceleration(self, node_id):
        if node_id in self.motors:
            return self.motors[node_id].get_acceleration()
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def disconnect(self):
        """네트워크 해제"""
        self.network.sync.stop()
        self.network.disconnect()

    def log_start_all(self):
        """모든 모터의 로그 기록 시작"""
        for node_id, motor in self.motors.items():
            motor.log_start()

    def log_stop_all(self):
        """모든 모터의 로그 기록 종료"""
        for node_id, motor in self.motors.items():
            motor.log_stop()

    def log_start(self, node_id):
        """특정 모터의 로그 기록 시작"""
        if node_id in self.motors:
            self.motors[node_id].log_start()
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def log_stop(self, node_id):
        """특정 모터의 로그 기록 종료"""
        if node_id in self.motors:
            self.motors[node_id].log_stop()
        else:
            print(f"Node {node_id} not found in motors dictionary.")

    def set_position_by_name(self, name, value):
        """이름으로 모터를 찾아 위치를 설정"""
        if name in self.name_to_id:
            motor_id = self.name_to_id[name]
            self.set_position(motor_id, value)
        else:
            print(f"Motor with name {name} not found")

    def get_torques(self):
        """등록된 모든 모터의 토크를 dict로 반환"""        
        torques = {}
        for node_id, motor in self.motors.items():
            torques[node_id] = motor.get_torque()
        return torques
        
    def check_all_motors_status(self):
        """등록된 모든 모터의 상태를 확인하고 문제가 있으면 처리"""
        has_error = False
        error_motor_id = None
        error_reason = None
        
        for node_id, motor in self.motors.items():
            if hasattr(motor, 'get_status'):
                status = motor.get_status()
                
                # 에러 또는 비활성화 상태 확인
                if status.get('error', False):
                    has_error = True
                    error_motor_id = node_id
                    error_reason = f"Fault detected (statusword: 0x{status.get('statusword', 0):04X})"
                    break
                    
                if status.get('disabled', False):
                    has_error = True
                    error_motor_id = node_id
                    error_reason = f"Motor disabled (statusword: 0x{status.get('statusword', 0):04X})"
                    break
                    
                if not status.get('active', True):
                    has_error = True
                    error_motor_id = node_id
                    error_reason = f"Operation not enabled (statusword: 0x{status.get('statusword', 0):04X})"
                    break
        
        # 문제가 있으면 모든 모터 비활성화
        if has_error and error_motor_id is not None:
            self.disable_all_motors(error_motor_id, error_reason)
            return False
            
        return True
    
    def disable_all_motors(self, error_motor_id, error_reason):
        """문제가 발생한 경우 모든 모터를 비활성화"""
        print(f"ERROR: 모터 ID {error_motor_id}에서 문제 발생: {error_reason}")
        print("안전을 위해 모든 모터를 비활성화합니다.")
        
        try:
            # 모든 모터에 빠른 정지(Quick Stop) 명령 보내기
            for node_id, motor in self.motors.items():
                try:
                    if hasattr(motor, 'node') and hasattr(motor.node, 'sdo'):
                        # 컨트롤워드에 Quick Stop 명령 설정 (0x0002)
                        motor.node.sdo[0x6040].raw = 0x0002
                except Exception as e:
                    print(f"모터 {node_id} 비활성화 중 오류 발생: {str(e)}")
            
            # Sync 멈추기
            self.sync_stop()
            
            # NMT 명령으로 모든 노드 정지
            self.network.nmt.send_command(0x02)  # Stop all nodes
            
            print("모든 모터가 비활성화되었습니다.")
        except Exception as e:
            print(f"모터 비활성화 중 오류 발생: {str(e)}")
            
    def get_all_motors_status(self):
        """모든 모터의 상태를 반환"""
        statuses = {}
        for node_id, motor in self.motors.items():
            if hasattr(motor, 'get_status'):
                statuses[node_id] = motor.get_status()
        return statuses

    def get_motor_status(self, node_id):
        if node_id in self.motors:
            if hasattr(self.motors[node_id], 'get_motor_status'):
                return self.motors[node_id].get_motor_status()
            else:
                # 가능한 기본 상태 값 반환
                return {'active': True, 'error': False, 'disabled': False, 'statusword': 0}
        else:
            print(f"Node {node_id} not found in motors dictionary.")
            return None