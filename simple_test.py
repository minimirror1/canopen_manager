#!/usr/bin/env python3
import canopen
import time
network = canopen.Network()
network.connect(channel="can0", bustype="socketcan", bitrate=1000000)
print("CAN에 연결됨")
for id in [1, 2, 3, 4, 5, 6, 7, 8, 10]:
    print(f"노드 {id} 테스트중...")
    node = network.add_node(id)
    node.nmt.send_command(0x80)  # Reset node
    time.sleep(0.2)
network.disconnect()
