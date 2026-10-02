from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    control_mode = LaunchConfiguration("control_mode")
    start_simulator = LaunchConfiguration("start_simulator")

    # ---------------------------------------------------------
    # MuJoCo simulator
    # ---------------------------------------------------------
    simulator = ExecuteProcess(
        condition=IfCondition(start_simulator),
        cmd=[
            'bash',
            '-lc',
            'exec run-g1-sim'
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # Unitree high-level controller
    # Only required when control_mode == high
    # ---------------------------------------------------------
    g1_ctrl = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                "'",
                control_mode,
                "' == 'high'"
            ])
        ),
        cmd=[
            'bash',
            '-lc',
            (
                'export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH && '
                'cd /workspace/controller/unitree_rl_lab_23/deploy/robots/g1_23dof/build && '
                'exec ./g1_ctrl --network lo'
            )
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # G1 low-level core
    # ---------------------------------------------------------
    core = Node(
        package='g1_core',
        executable='g1_lowlevel_core',
        name='g1_lowlevel_core',
        parameters=[
            {
                'simulation': True,
                'control_mode': control_mode,
            }
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # Simulated head camera -> ROS 2
    #
    # Reads:
    #   /dev/shm/g1_head_camera.rgb
    #
    # Publishes:
    #   /camera/image_raw
    # ---------------------------------------------------------
    camera_bridge = Node(
        package='g1_core',
        executable='g1_camera_bridge',
        name='g1_camera_bridge',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='high',
            description='high or low'
        ),

        DeclareLaunchArgument(
            'start_simulator',
            default_value='true',
            description='start MuJoCo simulator'
        ),

        simulator,
        g1_ctrl,
        core,
        camera_bridge,
    ])
