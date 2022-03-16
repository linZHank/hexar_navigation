from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexar_bringup',
            executable='hexar_core',
            name='hexar_core'
        ),
        Node(
            package='hexar_mapping',
            executable='pub_odom',
            name='odom_publisher',
        ),
    ])
