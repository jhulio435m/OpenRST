from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('openrst_id', default_value='0'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('node_name', default_value='openrst_control'),
        DeclareLaunchArgument('node_namespace', default_value='openrst'),
        
        # DAQ channels configuration
        DeclareLaunchArgument('m0_current_ai_ch', default_value='0'),
        DeclareLaunchArgument('m0_photosensor_ai_ch', default_value='8'),
        DeclareLaunchArgument('m0_control_ch', default_value='0'),
        DeclareLaunchArgument('m0_encoder_ch', default_value='0'),
        
        DeclareLaunchArgument('m1_current_ai_ch', default_value='2'),
        DeclareLaunchArgument('m1_photosensor_ai_ch', default_value='10'),
        DeclareLaunchArgument('m1_control_ch', default_value='2'),
        DeclareLaunchArgument('m1_encoder_ch', default_value='2'),
        
        DeclareLaunchArgument('m2_current_ai_ch', default_value='1'),
        DeclareLaunchArgument('m2_photosensor_ai_ch', default_value='9'),
        DeclareLaunchArgument('m2_control_ch', default_value='1'),
        DeclareLaunchArgument('m2_encoder_ch', default_value='1'),
        
        DeclareLaunchArgument('m0_engage_sensor_threshold', default_value='0.1'),
        DeclareLaunchArgument('m1_engage_sensor_threshold', default_value='0.1'),
        DeclareLaunchArgument('m2_engage_sensor_threshold', default_value='0.1'),
        
        DeclareLaunchArgument('cyclic_time_usec', default_value='8000'),

        Node(
            package='openrst_control',
            executable='openrst_control_node',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('node_namespace'),
            output='screen',
            parameters=[{
                'openrst_id': LaunchConfiguration('openrst_id'),
                'use_sim': LaunchConfiguration('use_sim'),
                'cyclic_time_usec': LaunchConfiguration('cyclic_time_usec'),
                'm0_current_ai_ch': LaunchConfiguration('m0_current_ai_ch'),
                'm0_photosensor_ai_ch': LaunchConfiguration('m0_photosensor_ai_ch'),
                'm0_control_ch': LaunchConfiguration('m0_control_ch'),
                'm0_encoder_ch': LaunchConfiguration('m0_encoder_ch'),
                'm1_current_ai_ch': LaunchConfiguration('m1_current_ai_ch'),
                'm1_photosensor_ai_ch': LaunchConfiguration('m1_photosensor_ai_ch'),
                'm1_control_ch': LaunchConfiguration('m1_control_ch'),
                'm1_encoder_ch': LaunchConfiguration('m1_encoder_ch'),
                'm2_current_ai_ch': LaunchConfiguration('m2_current_ai_ch'),
                'm2_photosensor_ai_ch': LaunchConfiguration('m2_photosensor_ai_ch'),
                'm2_control_ch': LaunchConfiguration('m2_control_ch'),
                'm2_encoder_ch': LaunchConfiguration('m2_encoder_ch'),
                'm0_engage_sensor_threshold': LaunchConfiguration('m0_engage_sensor_threshold'),
                'm1_engage_sensor_threshold': LaunchConfiguration('m1_engage_sensor_threshold'),
                'm2_engage_sensor_threshold': LaunchConfiguration('m2_engage_sensor_threshold'),
            }]
        ),
    ])
