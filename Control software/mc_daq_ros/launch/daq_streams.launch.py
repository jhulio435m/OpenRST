from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mc_daq_ros',
            executable='mc_daq_usb1608_stream',
            name='usb1608_stream',
            output='screen',
            parameters=[{'usb1608_stream_freq': 100}]
        ),
        Node(
            package='mc_daq_ros',
            executable='mc_daq_usb3104_stream',
            name='usb3104_stream',
            output='screen',
            parameters=[{'usb3104_stream_freq': 100}]
        ),
        Node(
            package='mc_daq_ros',
            executable='mc_daq_usbquad08_stream',
            name='usbquad08_stream',
            output='screen',
            parameters=[{'usbquad08_stream_freq': 100}]
        ),
    ])
