from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_description = get_package_share_directory('denis_v1_description')
    pkg_bringup = get_package_share_directory('denis_v1_bringup')

    # -------------------- ROBOT DESCRIPTION --------------------
    urdf_path = os.path.join(
        pkg_description,
        'urdf',
        'denis.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' use_sim:=true']),
        value_type=str
    )

    # -------------------- GAZEBO WORLD --------------------
    world_path = os.path.join(
        pkg_bringup,
        'worlds',
        'nav2_test.sdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}'
        }.items()
    )

    # -------------------- BRIDGE --------------------
    bridge_yaml = os.path.join(pkg_bringup, 'config', 'gazebo_bridge.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[
            {'config_file': os.path.join(
                get_package_share_directory('denis_v1_bringup'),
                'config',
                'gazebo_bridge.yaml'
            )}
        ]
    )
    # -------------------- SPAWN ROBOT --------------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'denis_v1',
            '-x', '-1.0',
            '-y', '1.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # -------------------- STATE PUBLISHER --------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
    )

    # -------------------- ROS2 CONTROL --------------------
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_bringup, 'config', 'denis_controllers.yaml')
        ],
        output='screen',
    )

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    spawn_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # -------------------- RVIZ --------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_description, 'rviz', 'denis_config.rviz')
        ],
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        spawn_jsb,
        spawn_diff,
        rviz,
    ])
