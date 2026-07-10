# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import AnyLaunchDescriptionSource
# from launch.substitutions import TextSubstitution, LaunchConfiguration
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import PathJoinSubstitution

# def generate_launch_description():
#     # get launch arguments: config_yaml, fcu_url, namespace, tgt_system
#     # rest should not need to be modified

#     config_yaml_arg = LaunchConfiguration('config_yaml_arg')
#     fcu_url_arg = LaunchConfiguration('fcu_url_arg')
#     namespace_arg = LaunchConfiguration('namespace_arg')
#     tgt_system_arg = LaunchConfiguration('tgt_system_arg')



#     return LaunchDescription([

#         DeclareLaunchArgument('config_yaml_arg', default_value="/home/schanna/code/uav_trajectory_simulator_ws/src/uav_trajectory_generator/config/px4_config_SQ01.yaml"),
#         DeclareLaunchArgument('fcu_url_arg', default_value="udp://:14541@127.0.0.1:14561"),
#         DeclareLaunchArgument('namespace_arg', default_value='SQ01/mavros'),
#         DeclareLaunchArgument('tgt_system_arg', default_value='2'),

#         IncludeLaunchDescription(
#             AnyLaunchDescriptionSource(
#                 PathJoinSubstitution([
#                     FindPackageShare("mavros"),
#                     "launch",
#                     "node.launch"
#                 ])
#             ),
#             launch_arguments={
#                 "pluginlists_yaml": "/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml",
#                 "config_yaml": config_yaml_arg,#"/home/schanna/code/uav_trajectory_simulator_ws/src/uav_trajectory_generator/config/px4_config_SQ01.yaml",
#                 "fcu_url": fcu_url_arg,#"udp://:14541@127.0.0.1:14561",
#                 "gcs_url": "",
#                 "namespace": namespace_arg,#"SQ01/mavros",
#                 "tgt_system": tgt_system_arg,#"2",
#                 "tgt_component": "1",
#             }.items(),
#         )
#     ])



import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
    fcu_url_arg = LaunchConfiguration('fcu_url_arg')
    tgt_system_arg = LaunchConfiguration('tgt_system_arg')

    template_yaml = Path(
        "/home/schanna/code/uav_trajectory_simulator_ws/src/uav_trajectory_generator/config/px4_config_template.yaml"
    )

    with open(template_yaml, "r") as f:
        yaml_text = f.read().replace("__VEHICLE__", vehicle_name)

    tmp_file = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp_file.write(yaml_text)
    tmp_file.close()

    return [
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("mavros"),
                    "launch",
                    "node.launch"
                ])
            ),
            launch_arguments={
                "config_yaml": tmp_file.name,
                "pluginlists_yaml": "/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml",
                "fcu_url": fcu_url_arg,
                "namespace": f"{vehicle_name}/mavros",
                "tgt_system": tgt_system_arg,
                "tgt_component": "1",
                "gcs_url": "",

            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_name', default_value='SQ01'),
        DeclareLaunchArgument('fcu_url_arg', default_value="udp://:14541@127.0.0.1:14561"),
        DeclareLaunchArgument('tgt_system_arg', default_value='2'),
        OpaqueFunction(function=launch_setup),
    ])