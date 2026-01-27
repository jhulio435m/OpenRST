from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usbquad08_stream_freq',
            default_value='500',
            description='Frequency of the usbquad08 stream'
        ),
        Node(
            package='mc_daq_ros',
            executable='mc_daq_usbquad08_stream',
            name='usbquad08_node',
            output='screen',
            parameters=[{'usbquad08_stream_freq': LaunchConfiguration('usbquad08_stream_freq')}]
        ),
    ])
