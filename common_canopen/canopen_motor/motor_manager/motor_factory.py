import os
# ROS1 의존성 제거
# import rospkg
from ..motor_vendors.motorVendorZeroErr import MotorVendorZeroErr
from ..motor_vendors.motorVendorElmo import MotorVendorElmo

# 필요하다면, 제조사 정보를 바탕으로 인스턴스를 생성해주는 Factory 구현 예시
class MotorFactory:
    @staticmethod
    def create_motor(motor_config):
        """모터 객체 생성 팩토리 메서드
        :param motor_config: 모터 설정 딕셔너리
            필수 키:
                - vendor_type: 제조사 타입 (예: "VendorZeroErr", "VendorElmo")
                - node_id: CAN 노드 ID
                - operation_mode: 동작 모드
                - name: 조인트 이름
            선택적 키:
                - zero_offset: 영점 오프셋 (기본값: 0)
                - profile_velocity: 프로파일 속도 (rad/s) (기본값: 1.0)
                - profile_acceleration: 프로파일 가속도 (rad/s²) (기본값: 1.0)
                - profile_deceleration: 프로파일 감속도 (rad/s²) (기본값: 1.0)
                - count_per_revolution: 한 바퀴당 펄스 수 (기본값: 1000)
                
        """
        # 필수 파라미터 검증
        required_params = ['vendor_type', 'node_id', 'operation_mode']
        for param in required_params:
            if param not in motor_config:
                raise ValueError(f"Missing required parameter: {param}")

        # 선택적 파라미터 기본값 설정
        vendor_type = motor_config['vendor_type']
        node_id = motor_config['node_id']
        operation_mode = motor_config['operation_mode']
        name = motor_config.get('name', f"joint_{node_id}")

        zero_offset = motor_config.get('zero_offset', 0)
        profile_velocity = motor_config.get('profile_velocity', 1.0)
        profile_acceleration = motor_config.get('profile_acceleration', 1.0)
        profile_deceleration = motor_config.get('profile_deceleration', 1.0)

        count_per_revolution = motor_config.get('count_per_revolution', 1000)

        print(f"Received vendor_type: '{vendor_type}'")  # 디버깅용 출력 유지

        # ROS1 패키지 경로 가져오기 대신 현재 모듈 경로 기준으로 처리
        # rospack = rospkg.RosPack()
        # package_path = rospack.get_path('canopen_manager')
        
        # 현재 파일의 위치를 기준으로 경로 계산
        current_file_path = os.path.dirname(os.path.abspath(__file__))
        motor_manager_dir = os.path.dirname(current_file_path)
        canopen_motor_dir = os.path.dirname(motor_manager_dir)
        
        if vendor_type == "VendorZeroErr":
            eds_path = os.path.join(canopen_motor_dir, 'config', 'ZeroErr Driver_V1.5.eds')
            # 파일 존재 확인
            if not os.path.exists(eds_path):
                print(f"경고: EDS 파일을 찾을 수 없습니다: {eds_path}")
                # 대체 경로 시도
                possible_paths = [
                    os.path.join(canopen_motor_dir, 'config', 'ZeroErr Driver_V1.5.eds'),
                    os.path.join(os.path.dirname(canopen_motor_dir), 'config', 'ZeroErr Driver_V1.5.eds'),
                    os.path.join('/root/ros_ws/src/canopen_manager/common_canopen/canopen_motor/config', 'ZeroErr Driver_V1.5.eds')
                ]
                
                for path in possible_paths:
                    if os.path.exists(path):
                        eds_path = path
                        print(f"EDS 파일을 찾았습니다: {eds_path}")
                        break
                else:
                    raise FileNotFoundError(f"EDS 파일을 찾을 수 없습니다: ZeroErr Driver_V1.5.eds")
            
            return MotorVendorZeroErr(node_id, 
                                    eds_path, 
                                    zero_offset, 
                                    operation_mode,
                                    profile_velocity, 
                                    profile_acceleration, 
                                    profile_deceleration,
                                    name)
        elif vendor_type == "VendorElmo":
            eds_path = os.path.join(canopen_motor_dir, 'config', 'elmo.dcf')
            # 파일 존재 확인
            if not os.path.exists(eds_path):
                print(f"경고: DCF 파일을 찾을 수 없습니다: {eds_path}")
                # 대체 경로 시도
                possible_paths = [
                    os.path.join(canopen_motor_dir, 'config', 'elmo.dcf'),
                    os.path.join(os.path.dirname(canopen_motor_dir), 'config', 'elmo.dcf'),
                    os.path.join('/root/ros_ws/src/canopen_manager/common_canopen/canopen_motor/config', 'elmo.dcf')
                ]
                
                for path in possible_paths:
                    if os.path.exists(path):
                        eds_path = path
                        print(f"DCF 파일을 찾았습니다: {eds_path}")
                        break
                else:
                    raise FileNotFoundError(f"DCF 파일을 찾을 수 없습니다: elmo.dcf")
            
            return MotorVendorElmo(node_id, 
                                  eds_path, 
                                  zero_offset, 
                                  operation_mode,
                                  profile_velocity, 
                                  profile_acceleration, 
                                  profile_deceleration,
                                  name,
                                  count_per_revolution)
        else:
            raise ValueError(f"Unknown vendor type: {vendor_type}")