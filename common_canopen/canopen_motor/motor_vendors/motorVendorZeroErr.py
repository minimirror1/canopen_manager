from ..motor_manager.abstract_motor import AbstractMotor
import time
import csv
from datetime import datetime
from math import pi

class MotorVendorZeroErr(AbstractMotor):
    """제조사 A 모터에 대한 구체 구현."""
    PULSE_PER_REVOLUTION = 524288  # 한 바퀴당 펄스 수

    
    
    def __init__(self, node_id, eds_path, zero_offset=0, operation_mode='PROFILE_POSITION',
                 profile_velocity=1.0, profile_acceleration=1.0, profile_deceleration=1.0, name=None):  # name 파라미터 추가
        super().__init__(node_id, eds_path, zero_offset, operation_mode,
                        profile_velocity, profile_acceleration, profile_deceleration,
                        name)  # name을 부모 클래스 생성자에 전달
        
    def init(self, operation_mode=None):
        if operation_mode:
            self.operation_mode = operation_mode.upper()
        
        if self.operation_mode not in self.OPERATION_MODES:
            raise ValueError(f"지원하지 않는 동작 모드입니다: {self.operation_mode}")
            
        print(f"[MotorVendorZeroErr] Init motor node: {self.node_id}")
        
        # 모터 상태 변수 초기화
        self.motor_status = {
            'statusword': 0,
            'ready_to_switch_on': False,
            'switched_on': False,
            'operation_enabled': False,
            'fault': False,
            'voltage_enabled': False,
            'quick_stop': False,
            'switch_on_disabled': False,
            'warning': False
        }
        
        # 모드 설정
        mode_value = self.OPERATION_MODES[self.operation_mode]
        self.node.sdo['Modes of operation'].raw = mode_value
        print(f'[write] Modes of operation: {hex(mode_value)} ({self.operation_mode})')

        self.ModeOfOperationDisplay = self.node.sdo['Modes of operation display'].raw
        print(f'[read] Modes of operation display: {self.ModeOfOperationDisplay}')
        
        # 모드별 초기화
        self._init_mode_specific_parameters()

        self.plusToRad = 2 * pi / self.PULSE_PER_REVOLUTION
        
    def try_auto_reset(self):
        """모터 에러 발생 시 자동 리셋을 시도하는 함수"""
        if self.motor_status.get('fault', False):
            print(f"[MotorVendorZeroErr] Attempting to auto-reset motor {self.node_id} after fault")
            self.reset()
            return True
        return False
    
    def get_motor_status(self):
        """모터 상태 반환"""
        return self.motor_status

    def _convert_rad_to_pulse(self, rad_value):
        """라디안 값을 펄스 카운트로 변환"""
        return int((rad_value * self.PULSE_PER_REVOLUTION) / (2 * pi))

    def _init_mode_specific_parameters(self):
        """모드별 특정 파라미터 초기화"""
        if self.operation_mode == 'PROFILE_POSITION':
            # 라디안 단위를 펄스 단위로 변환
            velocity_pulse = self._convert_rad_to_pulse(self.profile_velocity)
            acceleration_pulse = self._convert_rad_to_pulse(self.profile_acceleration)
            deceleration_pulse = self._convert_rad_to_pulse(self.profile_deceleration)
            
            self.node.sdo['Profile velocity'].raw = velocity_pulse  #0x6081
            self.node.sdo['Profile acceleration'].raw = acceleration_pulse  #0x6083
            self.node.sdo['Profile deceleration'].raw = deceleration_pulse  #0x6084
            print(f'[write] Profile parameters set for Position mode:')
            print(f'  Velocity: {self.profile_velocity} rad/s -> {velocity_pulse} pulse/s')
            print(f'  Acceleration: {self.profile_acceleration} rad/s² -> {acceleration_pulse} pulse/s²')
            print(f'  Deceleration: {self.profile_deceleration} rad/s² -> {deceleration_pulse} pulse/s²')
            
        elif self.operation_mode == 'PROFILE_TORQUE':
            self.node.sdo['Target torque'].raw = 0 #0x6071
            print(f'[write] Profile parameters set for Torque mode')

        else:
            print(f"지원하지 않는 동작 모드입니다: {self.operation_mode}")
            
    def reset(self):
        print(f"[MotorVendorZeroErr] Reset motor node: {self.node_id}")
        self.node.sdo[0x6040].raw = 0x27
        time.sleep(0.1)
        self.node.sdo[0x6040].raw = 0x26    
        time.sleep(0.1)
        self.node.sdo[0x6040].raw = 0x80  # 에러 클리어
        time.sleep(0.1)
        pass

    def log_start(self):
        """로그 시작"""
        self.logging = True
        self.start_time = time.time()
        
        # 현재 시간을 이용한 파일명 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"motor_log_{self.node_id}_{timestamp}.csv"
        
        # CSV 파일 생성 및 헤더 작성
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Time(ms)', 'Position(rad)', 'Torque(Nm)', 'Velocity(rad/s)', 'Acceleration(rad/s^2)'])

    def log_stop(self):
        """로그 종료"""
        if hasattr(self, 'logging') and self.logging:
            self.logging = False
            self.log_file.close()

    def pdo_mapping(self):
        print(f"[MotorVendorZeroErr] PDO mapping for node: {self.node_id}")
        # Read PDO configuration from node
        self.node.tpdo.read()
        self.node.rpdo.read()

        # master <- motor
        # 읽기 : 상태 값, 토크 센서 값
        self.node.tpdo[1].clear()
        self.node.tpdo[1].add_variable('Statusword')
        self.node.tpdo[1].add_variable('Position actual value') #aPosition actual value
        self.node.tpdo[1].cob_id = 0x180 + self.node_id
        self.node.tpdo[1].trans_type = 1
        #self.node.tpdo[1].trans_type = 254  # SYNC마다 전송
        #self.node.tpdo[1].event_timer = 10
        self.node.tpdo[1].enabled = True

        # 읽기 : 속도, 위치
        self.node.tpdo[2].clear()
        self.node.tpdo[2].add_variable('Torque sensor') #0x3B69, mN.m
        self.node.tpdo[2].add_variable('Velocity actual value') #0x606C, plus/s            
        self.node.tpdo[2].cob_id = 0x280 + self.node_id
        self.node.tpdo[2].trans_type = 1  # SYNC마다 전
        #self.node.tpdo[2].trans_type = 254  # SYNC마다 전송
        #self.node.tpdo[2].event_timer = 10
        self.node.tpdo[2].enabled = True

        # motor <- master
        # 쓰기 : 위치 목표값
        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable('Controlword')
        self.node.rpdo[1].add_variable('Target Position')
        self.node.rpdo[1].cob_id = 0x200 + self.node_id
        self.node.rpdo[1].trans_type = 0  # 즉시 적용
        #self.node.rpdo[1].event_timer = 255   # 이벤트 타이머 비활성화
        self.node.rpdo[1].enabled = True

        # 쓰기 : 토크 목표값
        self.node.rpdo[2].clear() 
        self.node.rpdo[2].add_variable('Controlword')
        self.node.rpdo[2].add_variable('Target torque') #0x6071 
        self.node.rpdo[2].cob_id = 0x300 + self.node_id
        self.node.rpdo[2].trans_type = 0  # 즉시 적용
        #self.node.rpdo[1].event_timer = 255   # 이벤트 타이머 비활성화
        self.node.rpdo[2].enabled = True

        self.motor_rated_current = self.node.sdo['Motor rated current'].raw #0x6075 모터 정격 전류 mA
        print(f'[read] Motor rated current: {self.motor_rated_current}')
        
        # Save new configuration (node must be in pre-operational)
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.tpdo.save()
        self.node.rpdo.save()

        # Start remote node
        self.node.nmt.state = 'OPERATIONAL'
        pass

    def set_switchOn(self):
        print(f"[MotorVendorZeroErr] Set switch on, node: {self.node_id}")
        """self.node.rpdo[1]['Controlword'].phys = 0x26
        self.node.rpdo[1].transmit()  # start() 대신 transmit() 사용    
        self.network.sync.transmit()

        self.node.rpdo[1]['Controlword'].phys = 0x27
        self.node.rpdo[1].transmit()
        self.network.sync.transmit()

        self.node.rpdo[1]['Controlword'].phys = 0x2f
        self.node.rpdo[1].transmit()
        self.network.sync.transmit()"""
        
        time.sleep(0.001)
        self.node.rpdo[1]['Controlword'].phys = 0x2f
        self.node.rpdo[1]['Target Position'].phys = self.node.sdo['Position actual value'].raw
        self.node.rpdo[1].transmit()
        time.sleep(0.001)

        self.node.rpdo[1]['Controlword'].phys = 0x3f
        self.node.rpdo[1].transmit()        
        time.sleep(0.001)

        pass

    def pdo_callback_register(self):
        self.network.subscribe(self.node.tpdo[1].cob_id, self.node.tpdo[1].on_message)
        self.node.tpdo[1].add_callback(self.tpdo1_callback)

        self.network.subscribe(self.node.tpdo[2].cob_id, self.node.tpdo[2].on_message)
        self.node.tpdo[2].add_callback(self.tpdo2_callback)

    def set_position(self, value):  # value in radians        
        """모터 위치 명령 (라디안 단위)"""
        #print(f"[MotorVendorZeroErr] Set position to {value} rad, node: {self.node_id}")
        self.node.rpdo[1]['Controlword'].phys = 0x2f
        
        # # 목표 위치를 라디안 단위로 저장
        self.target_position = value
        
        # 라디안 값을 펄스로 변환한 후 zero_offset(펄스)을 더함
        position_pulse = self._convert_rad_to_pulse(value) + self.zero_offset
        self.node.rpdo[1]['Target Position'].phys = position_pulse
        self.node.rpdo[1].transmit()

        # print(f"zero_offset: {self.zero_offset} pulse, target_position_pulse: {position_pulse}")

        self.node.rpdo[1]['Controlword'].phys = 0x3f
        self.node.rpdo[1].transmit()
        
    def get_position(self):        
        # self.current_position = self.node.sdo['Position actual value'].raw
        # print(f"[MotorVendorZeroErr] Get position, node: {self.node_id}, position: {self.current_position}")
        return self.current_position
    
    def set_torque(self, value):        
        self.target_torque = value * 1000 / self.motor_rated_current # mA

        print(f"[MotorVendorZeroErr] Set torque to {self.target_torque}, node: {self.node_id}")
        self.node.rpdo[1]['Controlword'].phys = 0x2f
        self.target_torque = value
        self.node.rpdo[1]['Target torque'].phys = self.target_torque
        self.node.rpdo[1].transmit()

        self.node.rpdo[1]['Controlword'].phys = 0x3f
        self.node.rpdo[1].transmit()       

    def get_torque(self):        
        # CANopen 노드에서 토크 값을 읽어옴
        # self.current_torque_sensor = self.node.sdo['Torque sensor'].raw / 1000  # mN.m을 N.m으로 변환
        return self.current_torque_sensor
    
    def get_velocity(self):
        return self.velocity_actual_value
    
    def get_acceleration(self):
        return self.current_acceleration

    def tpdo1_callback(self, message):
        # Statusword 읽기 (message.data의 첫 2바이트)
        statusword = int.from_bytes(message.data[0:2], byteorder='little')
        
        # 이전 상태와 현재 상태 비교를 위해 이전 상태 저장
        previous_status = self.motor_status.copy() if hasattr(self, 'motor_status') else None
        
        # Statusword 비트 해석
        is_ready_to_switch_on = bool(statusword & (1 << 0))  # Bit 0
        is_switched_on = bool(statusword & (1 << 1))        # Bit 1
        is_operation_enabled = bool(statusword & (1 << 2))  # Bit 2
        is_fault = bool(statusword & (1 << 3))              # Bit 3
        is_voltage_enabled = bool(statusword & (1 << 4))    # Bit 4
        is_quick_stop = bool(statusword & (1 << 5))         # Bit 5
        is_switch_on_disabled = bool(statusword & (1 << 6)) # Bit 6
        is_warning = bool(statusword & (1 << 7))            # Bit 7
        
        # 모터 상태 저장
        self.motor_status = {
            'statusword': statusword,
            'ready_to_switch_on': is_ready_to_switch_on,
            'switched_on': is_switched_on,
            'operation_enabled': is_operation_enabled,
            'fault': is_fault,
            'voltage_enabled': is_voltage_enabled,
            'quick_stop': is_quick_stop,
            'switch_on_disabled': is_switch_on_disabled,
            'warning': is_warning
        }
        
        # 모터 상태 감지 및 표시 (새로운 상태 변화가 있을 때만 출력)
        if previous_status is None or previous_status['fault'] != is_fault:
            if is_fault:
                print(f"[MotorVendorZeroErr] ERROR: Motor {self.node_id} Fault detected! Statusword: 0x{statusword:04X}")
                # 에러 발생 시 자동 리셋 시도
                #self.try_auto_reset()
        
        if previous_status is None or previous_status['switch_on_disabled'] != is_switch_on_disabled:
            if is_switch_on_disabled:
                print(f"[MotorVendorZeroErr] WARNING: Motor {self.node_id} Switch ON disabled! Statusword: 0x{statusword:04X}")
                # 비활성화 상태인 경우 활성화 시도
                #self.set_switchOn()
        
        if previous_status is None or previous_status['operation_enabled'] != is_operation_enabled:
            if not is_operation_enabled:
                print(f"[MotorVendorZeroErr] WARNING: Motor {self.node_id} Operation not enabled! Statusword: 0x{statusword:04X}")
                # 운영 모드가 비활성화된 경우 활성화 시도
                #if not is_fault and not is_quick_stop:  # 에러나 빠른 정지 상태가 아닌 경우에만 시도
                    #self.set_switchOn()
        
        if previous_status is None or previous_status['warning'] != is_warning:
            if is_warning:
                print(f"[MotorVendorZeroErr] WARNING: Motor {self.node_id} Warning bit set! Statusword: 0x{statusword:04X}")

        # 원래 위치 값 읽기 처리
        position = int.from_bytes(message.data[2:5], byteorder='little', signed=True)
        self.current_position = (position - self.zero_offset) * self.plusToRad  # rad로 변환
        #print(f'TPDO1 Position actual value: {self.current_position}')

    def tpdo2_callback(self, message):
        current_torque = int.from_bytes(message.data[0:3], byteorder='little', signed=True)  

        self.current_torque_sensor = current_torque / 1000        
        #print(f'TPDO2 Torque sensor: {self.current_torque_sensor}')


        pulse_velocity = int.from_bytes(message.data[4:7], byteorder='little', signed=True)

        self.velocity_actual_value = pulse_velocity * self.plusToRad  # rad/s로 변환
        #print(f'TPDO2 Velocity actual value: {self.velocity_actual_value} rad/s')
        
        self.current_acceleration = (self.velocity_actual_value - self.current_velocity_old) / self.dt
        self.current_velocity_old = self.velocity_actual_value
        #print(f'TPDO2 Acceleration: {self.current_acceleration} rad/s^2')

        # 로깅이 활성화된 경우 데이터 저장
        if hasattr(self, 'logging') and self.logging:
            current_time = (time.time() - self.start_time) * 1000  # ms 단위로 변환
            self.csv_writer.writerow([
                f"{current_time:.1f}",
                f"{self.current_position:.6f}",
                f"{self.current_torque_sensor:.6f}",
                f"{self.velocity_actual_value:.6f}",
                f"{self.current_acceleration:.6f}"
            ])

    def set_velocity(self, value):
        """모터 속도 명령"""
        print(f"[MotorVendorZeroErr] Set velocity to {value}, node: {self.node_id}")

    def set_acceleration(self, value):
        """모터 가속도 명령"""
        print(f"[MotorVendorZeroErr] Set acceleration to {value}, node: {self.node_id}")

    def get_status(self):
        """모터의 전체 상태 정보를 반환하는 함수"""
        status = {
            'node_id': self.node_id,
            'name': self.name,
            'position': self.current_position if hasattr(self, 'current_position') else 0,
            'torque': self.current_torque_sensor if hasattr(self, 'current_torque_sensor') else 0,
            'velocity': self.velocity_actual_value if hasattr(self, 'velocity_actual_value') else 0
        }
        
        # 모터 상태 정보 추가
        if hasattr(self, 'motor_status'):
            status.update({
                'error': self.motor_status.get('fault', False),
                'disabled': self.motor_status.get('switch_on_disabled', False),
                'active': self.motor_status.get('operation_enabled', False),
                'statusword': self.motor_status.get('statusword', 0),
                'warning': self.motor_status.get('warning', False),
                'ready': self.motor_status.get('ready_to_switch_on', False)
            })
        
        return status

# 다음 개선 사항:
# 1. MotorVendorElmo 클래스에도 동일한 Status word 감시 기능 구현 필요
# 2. 구현 방법:
#    - motor_status 변수 초기화를 init()에 추가
#    - tpdo1_callback에서 Status word 비트 해석 코드 추가
#    - try_auto_reset(), get_motor_status(), get_status() 메소드 추가
#    - 에러 및 비활성화 시 자동 복구 로직 구현
