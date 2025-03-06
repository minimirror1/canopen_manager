from abc import ABC, abstractmethod

class AbstractMotor(ABC):
    """모든 모터가 공통으로 가져야 할 인터페이스 정의(추상 클래스)."""
       
    # 모드 매핑 추가
    OPERATION_MODES = {
        'PROFILE_POSITION': 0x01,
        'PROFILE_VELOCITY': 0x03,
        'PROFILE_TORQUE': 0x04,
        'HOMING': 0x06,
        'INTERPOLATED_POSITION': 0x07,
        'CYCLIC_SYNC_POSITION': 0x08,
        'CYCLIC_SYNC_VELOCITY': 0x09,
        'CYCLIC_SYNC_TORQUE': 0x0A
    }

    def __init__(self, node_id, eds_path, zero_offset=0, operation_mode='PROFILE_POSITION',
                 profile_velocity=1.0, profile_acceleration=1.0, profile_deceleration=1.0, name=None, count_per_revolution=1000):  # rad/s, rad/s², rad/s², name 파라미터 추가
        self.operation_mode = operation_mode
        self.node_id = node_id
        self.eds_path = eds_path
        self.name = name or f"joint_{node_id}"  # name이 없으면 기본값으로 joint_노드ID 사용
        self.node = None  # canopen에서 로드되는 노드 객체 (초기에는 None)
        self.network = None  # network 객체 추가
        # 목표 위치와 현재 위치를 저장하는 변수 추가
        self.zero_offset = zero_offset

        self.target_position = 0    # 목표 위치값 저장용 변수
        self.target_torque = 0      # 목표 토크값 저장용 변수

        self.current_position = 0       # 현재 위치값 저장용 변수
        self.current_torque_sensor = 0  # 현재 토크값 저장용 변수
        self.current_velocity = 0       # 현재 속도값 저장용 변수
        self.current_velocity_old = 0   # 이전 속도값 저장용 변수
        self.current_acceleration = 0   # 현재 가속도값 저장용 변수

        # 프로파일 파라미터 추가 (라디안 단위)
        self.profile_velocity = profile_velocity  # rad/s
        self.profile_acceleration = profile_acceleration  # rad/s²
        self.profile_deceleration = profile_deceleration  # rad/s²
        self.count_per_revolution = count_per_revolution  # 한 바퀴당 펄스 수
    @abstractmethod
    def init(self):
        """모터 초기화 절차"""
        pass

    @abstractmethod
    def reset(self):
        """모터 리셋(에러 클리어 등)"""
        pass

    def set_dt(self, value = 0.01):
        self.dt = value
        pass

    @abstractmethod
    def pdo_mapping(self):
        """PDO 매핑 설정"""
        pass

    @abstractmethod
    def set_switchOn(self):
        """Switch On 명령"""
        pass

    def set_zero_offset(self, value):
        """Zero Offset 변경"""
        self.zero_offset = value
        pass

    def get_zero_offset(self):
        """Zero Offset 조회"""
        return self.zero_offset
    
    @abstractmethod
    def pdo_callback_register(self):
        """PDO 콜백 등록"""
        pass

    @abstractmethod
    def set_position(self, value):
        """모터 위치 명령"""
        pass

    @abstractmethod
    def get_position(self):
        """모터 위치 확인"""
        pass

    @abstractmethod
    def set_torque(self, value):
        """모터 토크 명령"""
        pass

    @abstractmethod
    def get_torque(self):
        """모터 토크 확인"""
        pass

    @abstractmethod
    def set_velocity(self, value):
        """모터 속도 명령"""
        pass

    @abstractmethod
    def get_velocity(self):
        """모터 속도 확인"""
        pass

    @abstractmethod
    def set_acceleration(self, value):
        """모터 가속도 명령"""
        pass

    @abstractmethod
    def get_acceleration(self):
        """모터 가속도 확인"""
        pass

    @abstractmethod
    def log_start(self):
        """로그 기록 시작"""
        pass

    @abstractmethod
    def log_stop(self):
        """로그 기록 종료"""
        pass

