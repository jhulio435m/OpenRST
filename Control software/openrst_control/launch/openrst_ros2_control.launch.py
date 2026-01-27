import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    openrst_control_share = get_package_share_directory('openrst_control')
    openrst_description_share = get_package_share_directory('openrst_description')
    
    # Path to URDF
    urdf_file = os.path.join(openrst_description_share, 'urdf', 'openrst.urdf')
    
    # Path to Controllers YAML
    controllers_yaml = os.path.join(openrst_control_share, 'config', 'openrst_controllers.yaml')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # ROS 2 Control Manager
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controllers_yaml],
        output='screen',
    )
    
    # Joint State Broadcaster Spawner
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # Effort Controller Spawner
    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        jsb_spawner,
        effort_spawner,
    ])
