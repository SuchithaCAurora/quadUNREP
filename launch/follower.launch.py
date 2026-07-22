import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare arguments for vehicle type and number
    veh_arg = DeclareLaunchArgument(
        'veh',
        default_value=EnvironmentVariable('VEHTYPE', default_value='SQ'),
        description='Vehicle type'
    )
    num_arg = DeclareLaunchArgument(
        'num',
        default_value=EnvironmentVariable('VEHNUM', default_value='02'),
        description='Vehicle number'
    )

    # Namespace from veh and num arguements
    namespace = [LaunchConfiguration('veh'), LaunchConfiguration('num')]

    # Define node 
    follower_trajectory_node = Node(
        package='trajectory_generator_ros2_unrep',
        namespace=namespace,
        executable='follower_traj_generator2',
        name='follower_traj_generator2',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('trajectory_generator_ros2_unrep'),
            'config',
            'follower_behavior.yaml'
        )]
    )

    tf_map_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=[
            '0.0', '0.0', '0.0',        # translation (x, y, z) in meters
            '0.0', '0.0', '0.0',       # rotation (roll, pitch, yaw) in radians
            'map',                    # parent frame
            'world'                # child frame
        ],
        output='screen'
    )
    

    launch_group = GroupAction([tf_map_world, follower_trajectory_node])

    return LaunchDescription([
        veh_arg,
        num_arg,
        launch_group
    ])