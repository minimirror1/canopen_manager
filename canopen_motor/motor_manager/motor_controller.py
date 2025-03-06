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
        if interface is None:
            self.network.connect(channel=channel, bustype=bustype, bitrate=bitrate)
        else:
            self.network.connect(interface=interface, channel=channel, bitrate=bitrate)
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
            # 리셋
        self.reset_all()
        #time.sleep(3) 
        
        # 모터 전체 초기화
        self.init_all()
        #time.sleep(3) 
        
        # PDO 매핑
        self.pdo_mapping_all()

        # Switch On
        self.set_switchOn_all()

        # PDO 콜백 등록
        self.pdo_callback_register_all()
    
        # 동기화 시작
        self.sync_start(interval)

    def init_all(self):
        """등록된 모든 모터를 초기화"""
        for node_id, motor in self.motors.items():
            motor.init()

    def reset_all(self):
        """등록된 모든 모터를 리셋"""
        for node_id, motor in self.motors.items():
            motor.reset()            

        self.network.nmt.send_command(0x02)  # Stop
        time.sleep(0.5)
        self.network.nmt.send_command(0x82)  # Reset
        time.sleep(1)  # 재설정 후 충분한 대기 시간

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
        for node_id, motor in self.motors.items():
            motor.set_dt(interval)

        self.network.sync.start(interval) # 10ms

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