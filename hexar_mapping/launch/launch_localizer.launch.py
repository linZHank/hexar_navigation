from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():
#     return LaunchDescription([
def generate_launch_description():
    mapping_package_path = get_package_share_path("hexar_mapping")

    hexar_core_node = Node(
        package='hexar_bringup',
        executable='hexar_core',
        name='hexar_core'
    )

    odom_pub_node = Node(
        package='hexar_mapping',
        executable='pub_odom',
        name='odom_publisher',
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            str(mapping_package_path / "configs/ekf.yaml"),
        ],
    )

    return LaunchDescription(
        [
            hexar_core_node,
            odom_pub_node,
            robot_localization_node,
        ]
    )


