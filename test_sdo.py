#!/usr/bin/env python3
import canopen
import time
import sys

# CAN 네트워크 설정
network = canopen.Network()
network.connect(channel='can0', bustype='socketcan', bitrate=1000000)

# 테스트할 노드 ID 목록
node_ids = [1, 2, 3, 4, 5, 6, 7, 8, 10]

print(f"CAN 버스 연결됨: can0, bitrate=1000000")

# EDS 파일 경로
eds_path = '/root/ros_ws/src/canopen_manager/common_canopen/canopen_motor/config/ZeroErr Driver_V1.5.eds'
dcf_path = '/root/ros_ws/src/canopen_manager/common_canopen/canopen_motor/config/elmo.dcf'

# 각 노드에 대해 ping 테스트 실행
for node_id in node_ids:
    try:
        if node_id == 10:  # Elmo 모터는 다른 EDS 파일 사용
            node = network.add_node(node_id, dcf_path)
            print(f"노드 {node_id}(Elmo)에 ping 전송 중...")
        else:
            node = network.add_node(node_id, eds_path)
            print(f"노드 {node_id}(ZeroErr)에 ping 전송 중...")
        
        # NMT 상태 확인 (heartbeat 모니터링)
        node.nmt.start_heartbeat_monitor()
        time.sleep(0.5)  # 응답 대기
        
        if node.nmt.state:
            print(f"  응답 있음! 노드 {node_id} 상태: {node.nmt.state}")
        else:
            print(f"  응답 없음: 노드 {node_id}")
        
        # SDO 통신 시도
        try:
            print(f"  SDO 읽기 시도(장치 유형)...")
            device_type = node.sdo[0x1000].raw
            print(f"  성공! 장치 유형: 0x{device_type:08X}")
        except Exception as e:
            print(f"  SDO 읽기 실패: {type(e).__name__}: {str(e)}")
        
    except Exception as e:
        print(f"노드 {node_id} 통신 오류: {type(e).__name__}: {str(e)}")
    
    print("")  # 줄 바꿈
    
# 네트워크 연결 종료
network.disconnect()
print("CAN 버스 연결 종료") 