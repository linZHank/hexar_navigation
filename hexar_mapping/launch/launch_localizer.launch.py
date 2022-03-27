from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdf_package_path = get_package_share_path("hexar_description")
    default_model_path = urdf_package_path / "urdf/hexar.xacro"
    mapping_package_path = get_package_share_path("hexar_mapping")
    rplidar_package_path = get_package_share_path("rplidar_ros2")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    hexar_core_node = Node(
        package="hexar_bringup", executable="hexar_core", name="hexar_core"
    )

    odom_pub_node = Node(
        package="hexar_mapping",
        executable="pub_odom",
        name="odom_publisher",
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

    launch_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rplidar_package_path / "launch/rplidar_launch.py")
        ),
        launch_arguments={
            "frame_id": "lidar_link"
        }.items()
    )

    return LaunchDescription(
        [
            model_arg,
            hexar_core_node,
            odom_pub_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
            robot_localization_node,
            launch_rplidar,
        ]
    )
