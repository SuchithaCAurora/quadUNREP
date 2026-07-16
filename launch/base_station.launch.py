import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    behavior_selector_launch_file = os.path.join(
        get_package_share_directory('behavior_selector2'),
        'launch',
        'gui.launch.py'
    )

    rviz_node = Node(
            package="rviz2",
            name="rviz2",
            executable="rviz2",
            output="screen",
            arguments= ['-d',
                os.path.join(get_package_share_directory('trajectory_generator_ros2_unrep'),
                        'config', 'default.rviz')
            ]
        )

    return LaunchDescription([
            rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(behavior_selector_launch_file)
        )
    ])