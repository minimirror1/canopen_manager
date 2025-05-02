from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    can_interface = LaunchConfiguration('can_interface')
    heartbeat_interval = LaunchConfiguration('heartbeat_interval')
    can_bitrate = LaunchConfiguration('can_bitrate')
    can_txqueuelen = LaunchConfiguration('can_txqueuelen')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface to use'
        ),
        
        DeclareLaunchArgument(
            'heartbeat_interval',
            default_value='1000',
            description='Heartbeat interval in ms'
        ),
        
        DeclareLaunchArgument(
            'can_bitrate',
            default_value='1000000',
            description='CAN bitrate (1Mbps)'
        ),
        
        DeclareLaunchArgument(
            'can_txqueuelen',
            default_value='1000',
            description='CAN txqueuelen'
        ),
        
        # Nodes
        Node(
            package='ros2_interface',
            executable='canopen_node.py',
            name='canopen_manager',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'heartbeat_interval': heartbeat_interval,
                'can_bitrate': can_bitrate,
                'can_txqueuelen': can_txqueuelen
            }]
        )
    ]) 