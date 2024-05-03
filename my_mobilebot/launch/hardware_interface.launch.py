import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

import xacro



def generate_launch_description():

    robot_description = ParameterValue(Command(["xacro ",os.path.join(
                                                get_package_share_directory("my_mobilebot"), "description", "robot.urdf.xacro"
                                                )]),
                                                value_type=str)
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description" : robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("mobilebot_controller"),
                "config",
                "mobilebot_controllers.yaml"
            )]
    )





    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
    ])



