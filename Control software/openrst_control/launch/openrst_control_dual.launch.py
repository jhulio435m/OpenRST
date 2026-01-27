from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    openrst_control_share = get_package_share_directory('openrst_control')
    # Since we are likely running from the source or after install, 
    # we need to find the launch file.
    # In a real scenario, it would be in the share directory.
    launch_path = os.path.join(openrst_control_share, 'launch', 'openrst_control.launch.py')
    
    return LaunchDescription([
        # Forceps 0
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                'openrst_id': '0',
                'node_name': 'openrst_control_0',
                'm0_current_ai_ch': '0',
                'm0_photosensor_ai_ch': '8',
                'm0_control_ch': '0',
                'm0_encoder_ch': '0',
                'm1_current_ai_ch': '2',
                'm1_photosensor_ai_ch': '10',
                'm1_control_ch': '2',
                'm1_encoder_ch': '2',
                'm2_current_ai_ch': '1',
                'm2_photosensor_ai_ch': '9',
                'm2_control_ch': '1',
                'm2_encoder_ch': '1',
                'm0_engage_sensor_threshold': '0.1',
                'm1_engage_sensor_threshold': '0.1',
                'm2_engage_sensor_threshold': '0.1',
            }.items(),
        ),
        
        # Forceps 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                'openrst_id': '1',
                'node_name': 'openrst_control_1',
                'm0_current_ai_ch': '3',
                'm0_photosensor_ai_ch': '11',
                'm0_control_ch': '3',
                'm0_encoder_ch': '3',
                'm1_current_ai_ch': '5',
                'm1_photosensor_ai_ch': '13',
                'm1_control_ch': '5',
                'm1_encoder_ch': '5',
                'm2_current_ai_ch': '4',
                'm2_photosensor_ai_ch': '12',
                'm2_control_ch': '4',
                'm2_encoder_ch': '4',
                'm0_engage_sensor_threshold': '0.15',
                'm1_engage_sensor_threshold': '0.15',
                'm2_engage_sensor_threshold': '0.15',
            }.items(),
        ),
    ])