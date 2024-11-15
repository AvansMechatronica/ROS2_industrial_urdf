from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    # Paths
    package_share_directory = get_package_share_directory('urdf_basics')
    xacro_file = os.path.join(package_share_directory, 'urdf', 'hrwros_assignment1.xacro')
    rviz_config_file = os.path.join(package_share_directory, 'config', 'assignment1.rviz')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file])
            }]
        ),
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': True}],
            #ros_parameters={
            #    'zeros': {
            #        'robot1_shoulder_pan_joint': 0.6207787083493841,
            #        'robot1_shoulder_lift_joint': -1.004681330618082,
            #        'robot1_elbow_joint': 1.6983449885307538,
            #        'robot1_wrist_1_joint': -2.301530778020034,
            #        'robot1_wrist_2_joint': -1.625460038967466,
            #        'robot1_wrist_3_joint': 0.0,
            #        'robot2_shoulder_pan_joint': 1.5707963267950005,
            #        'robot2_shoulder_lift_joint': -1.5525750894041779,
            #        'robot2_elbow_joint': 1.5525750894041783,
            #        'robot2_wrist_1_joint': -1.570796326795,
            #        'robot2_wrist_2_joint': -1.534353852013356,
            #        'robot2_wrist_3_joint': 0.0,
            #    }
            #}
        ),

    ])
