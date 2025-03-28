#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import json
import rospy
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSpinBox, QPushButton, 
                             QComboBox, QMessageBox, QGroupBox)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState

class MotorSettingWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Settings")  # 모터 설정
        self.setGeometry(100, 100, 800, 600)
        
        # 한글 폰트 설정
        self.set_font()
        
        # ROS 노드 초기화
        rospy.init_node('motor_setting_gui', anonymous=True)
        
        # 모터 정보 로드
        self.motors_info = self.load_motors_info()
        
        # 메인 위젯 및 레이아웃 설정
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # 모터 선택 그룹
        motor_group = QGroupBox("Motor Selection")  # 모터 선택
        motor_layout = QHBoxLayout()
        
        self.motor_combo = QComboBox()
        self.motor_combo.addItems([motor['name'] for motor in self.motors_info])
        self.motor_combo.currentIndexChanged.connect(self.on_motor_changed)
        motor_layout.addWidget(QLabel("Motor:"))  # 모터:
        motor_layout.addWidget(self.motor_combo)
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)
        
        # 모터 상태 그룹
        status_group = QGroupBox("Motor Status")  # 모터 상태
        status_layout = QVBoxLayout()
        
        self.position_label = QLabel("Position: 0.0")  # 위치
        self.velocity_label = QLabel("Velocity: 0.0")  # 속도
        self.torque_label = QLabel("Torque: 0.0")  # 토크
        
        status_layout.addWidget(self.position_label)
        status_layout.addWidget(self.velocity_label)
        status_layout.addWidget(self.torque_label)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 모터 제어 그룹
        control_group = QGroupBox("Motor Control")  # 모터 제어
        control_layout = QVBoxLayout()
        
        # 위치 제어
        position_layout = QHBoxLayout()
        position_layout.addWidget(QLabel("Position:"))  # 위치:
        self.position_spin = QSpinBox()
        self.position_spin.setRange(-360, 360)
        position_layout.addWidget(self.position_spin)
        position_layout.addWidget(QLabel("deg"))  # 도
        
        # 속도 제어
        velocity_layout = QHBoxLayout()
        velocity_layout.addWidget(QLabel("Velocity:"))  # 속도:
        self.velocity_spin = QSpinBox()
        self.velocity_spin.setRange(0, 1000)
        velocity_layout.addWidget(self.velocity_spin)
        velocity_layout.addWidget(QLabel("rpm"))
        
        # 토크 제어
        torque_layout = QHBoxLayout()
        torque_layout.addWidget(QLabel("Torque:"))  # 토크:
        self.torque_spin = QSpinBox()
        self.torque_spin.setRange(0, 100)
        torque_layout.addWidget(self.torque_spin)
        torque_layout.addWidget(QLabel("%"))
        
        # 제어 버튼
        self.send_button = QPushButton("Send Command")  # 명령 전송
        self.send_button.clicked.connect(self.send_command)
        
        control_layout.addLayout(position_layout)
        control_layout.addLayout(velocity_layout)
        control_layout.addLayout(torque_layout)
        control_layout.addWidget(self.send_button)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # ROS 구독자 및 발행자 설정
        self.joint_state_sub = rospy.Subscriber('canopen/joint_states', JointState, self.joint_state_callback)
        self.position_pub = None
        
        # 타이머 설정 (GUI 업데이트용)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # 100ms 간격으로 업데이트
        
        # 현재 선택된 모터 정보
        self.current_motor = None
        self.on_motor_changed(0)
        
    def load_motors_info(self):
        """Load motor information from JSON file"""  # JSON 파일에서 모터 정보를 로드합니다
        try:
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            json_path = os.path.join(package_path, 'canopen_info_json', 'canopen_info.json')
            
            with open(json_path, 'r') as f:
                data = json.load(f)
                return data.get('motors', [])
        except Exception as e:
            rospy.logerr("Failed to load motor information: %s", str(e))  # 모터 정보 로드 실패
            return []
    
    def on_motor_changed(self, index):
        """Called when motor selection is changed"""  # 모터 선택이 변경되었을 때 호출되는 함수
        if 0 <= index < len(self.motors_info):
            self.current_motor = self.motors_info[index]
            self.setWindowTitle(f"Motor Settings - {self.current_motor['name']}")  # 모터 설정 - {모터명}
            
            # Update publisher with new motor name
            if self.position_pub:
                self.position_pub.unregister()
            self.position_pub = rospy.Publisher(
                f'canopen/single_motor/{self.current_motor["name"]}/position',
                Float64,
                queue_size=10
            )
            
            rospy.loginfo(f"Selected motor: {self.current_motor['name']} (Node ID: {self.current_motor['node_id']})")  # 선택된 모터: {모터명} (노드 ID: {노드ID})
    
    def joint_state_callback(self, msg):
        """Process JointState message"""  # JointState 메시지를 처리하는 콜백 함수
        if not self.current_motor:
            return
            
        try:
            motor_name = self.current_motor['name']
            if motor_name in msg.name:
                index = msg.name.index(motor_name)
                position = msg.position[index]
                velocity = msg.velocity[index]
                effort = msg.effort[index]
                
                self.position_label.setText(f"Position: {position:.2f}")  # 위치
                self.velocity_label.setText(f"Velocity: {velocity:.2f}")  # 속도
                self.torque_label.setText(f"Torque: {effort:.2f}")  # 토크
        except Exception as e:
            rospy.logerr(f"Error updating status: {str(e)}")  # 상태 업데이트 중 오류 발생
    
    def send_command(self):
        """Send motor control command"""  # 모터 제어 명령을 전송하는 함수
        if not self.current_motor:
            return
            
        try:
            position = self.position_spin.value()
            velocity = self.velocity_spin.value()
            torque = self.torque_spin.value()
            
            # 위치 명령 전송
            self.position_pub.publish(position)
            
            rospy.loginfo(f"Command sent: position={position}, velocity={velocity}, torque={torque}")  # 명령 전송: 위치, 속도, 토크
            QMessageBox.information(self, "Notice", "Command sent successfully")  # 알림: 명령이 전송되었습니다
        except Exception as e:
            rospy.logerr(f"Error sending command: {str(e)}")  # 명령 전송 중 오류 발생
            QMessageBox.warning(self, "Error", f"Failed to send command: {str(e)}")  # 오류: 명령 전송 실패
    
    def update_gui(self):
        """Timer callback for GUI update"""  # GUI 업데이트를 위한 타이머 콜백 함수
        rospy.sleep(0.1)  # ROS 스핀 처리
    
    def set_font(self):
        """Set Korean font"""  # 한글 폰트를 설정합니다
        try:
            # Ubuntu의 기본 한글 폰트
            font = QFont("Ubuntu", 10)
            self.setFont(font)
        except:
            try:
                # DejaVu Sans 폰트
                font = QFont("DejaVu Sans", 10)
                self.setFont(font)
            except:
                # 기본 폰트
                font = QFont()
                font.setPointSize(10)
                self.setFont(font)

def main():
    app = QApplication(sys.argv)
    window = MotorSettingWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 