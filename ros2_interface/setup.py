from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_interface'

# common_canopen 패키지 경로 추가
common_canopen_path = os.path.join(os.path.dirname(__file__), '..', 'common_canopen')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.src', 'common_canopen', 'common_canopen.canopen_motor',
              'common_canopen.canopen_motor.motor_manager', 'common_canopen.canopen_motor.motor_vendors'],
    package_dir={
        'common_canopen': common_canopen_path,
        'common_canopen.canopen_motor': os.path.join(common_canopen_path, 'canopen_motor'),
        'common_canopen.canopen_motor.motor_manager': os.path.join(common_canopen_path, 'canopen_motor', 'motor_manager'),
        'common_canopen.canopen_motor.motor_vendors': os.path.join(common_canopen_path, 'canopen_motor', 'motor_vendors')
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob(os.path.join(common_canopen_path, 'canopen_info_json', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS User',
    maintainer_email='user@example.com',
    description='CANopen ROS2 인터페이스 - ROS2에서 CANopen 라이브러리를 사용하는 인터페이스',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'canopen_node = ros2_interface.src.canopen_node:main',
            'canopen_node_new = ros2_interface.src.canopen_node_new:main',
            'send_command = ros2_interface.src.send_command:main',
        ],
    },
) 