from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the included launch file
    load_robot_launch = os.path.join(
        get_package_share_directory('urdf_basics'),
        'launch',
        'replacement_robot',
        'load_robot.launch.py'
    )

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('urdf_basics'),
        'config',
        'view_robot.rviz'
    )

    return LaunchDescription([
        # Include the other launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(load_robot_launch)
        ),
 
        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': True}]
        ),
        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
