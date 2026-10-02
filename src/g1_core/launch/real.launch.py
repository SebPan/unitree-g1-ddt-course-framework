from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    control_mode = LaunchConfiguration("control_mode")
    core = Node(package="g1_core",
                executable="g1_lowlevel_core",
                name="g1_lowlevel_core",
                parameters=[{"simulation": False,
                             "control_mode": control_mode,}],
                output="screen")

    return LaunchDescription([DeclareLaunchArgument("control_mode",
                                                    default_value="high",
                                                    description="high or low"),
                              core,])