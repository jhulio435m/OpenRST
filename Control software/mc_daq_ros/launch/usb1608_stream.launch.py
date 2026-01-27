from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usb1608_stream_freq',
            default_value='500',
            description='Frequency of the usb1608 stream'
        ),
        Node(
            package='mc_daq_ros',
            executable='mc_daq_usb1608_stream',
            name='usb1608_node',
            output='screen',
            parameters=[{'usb1608_stream_freq': LaunchConfiguration('usb1608_stream_freq')}]
        ),
    ])
