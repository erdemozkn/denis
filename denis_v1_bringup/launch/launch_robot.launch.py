import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    
    pkg_denis_bringup = get_package_share_directory('denis_v1_bringup')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_denis_bringup, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true'
        }.items()
    )

    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         pkg_denis_bringup, 'launch', 'joystick.launch.py'
    #     )]), launch_arguments={'use_sim_time': 'false'}.items()
    # )

    twist_mux_params = os.path.join(pkg_denis_bringup, 'config', 'twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[
                twist_mux_params,
                {'use_stamped': False}
            ],
            remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(pkg_denis_bringup, 'config', 'denis_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                   controller_params_file
                   ]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        # joystick,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])