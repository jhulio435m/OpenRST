from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usb3104_stream_freq',
            default_value='125',
            description='Frequency of the usb3104 stream'
        ),
        Node(
            package='mc_daq_ros',
            executable='mc_daq_usb3104_stream',
            name='usb3104_node',
            output='screen',
            parameters=[{'usb3104_stream_freq': LaunchConfiguration('usb3104_stream_freq')}]
        ),
    ])
