import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_mobilebot"),
            "launch",
            "hardware_interface.launch.py"
        )
    )
    

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mobilebot_controller"),
            "launch",
            "mobilebot_controller.launch.py"
        )
    )

    velocity_converter_node = Node(
        package="mobilebot_firmware",
        executable="velocityconverter.py",
        name="velocityconverter",

    )

    return LaunchDescription([
        hardware_interface,
        controller,
        velocity_converter_node,
    

    ])