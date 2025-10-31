from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    # Get package directory
    pkg_share = get_package_share_directory('dual_arm_teleoperation')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'dual_ur5.urdf.xacro')
    
    # Path to RViz config
    rviz_config = os.path.join(pkg_share, 'rviz', 'dual_arm.rviz')
    
    # Process xacro
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Keyboard Teleop Node
    keyboard_teleop = Node(
        package='dual_arm_teleoperation',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal
    )
    
    # RViz Node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        keyboard_teleop,
        rviz
    ])
