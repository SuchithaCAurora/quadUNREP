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
        "/home/schanna/code/uav_trajectory_simulator_ws/src/uav_trajectory_generator/config/px4_config_template_hw.yaml"
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