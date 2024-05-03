import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value = "0.034"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value = "0.365"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    
    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    mobilebot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mobilebot_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        mobilebot_controller_spawner,
    ])

    
    
      
