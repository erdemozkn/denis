from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('denis_v1_description'), 'urdf', 'denis.urdf.xacro')

    robot_description= ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    robot_state_publisher_node = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description' : robot_description}])
    
    controller_config_path = os.path.join(get_package_share_path('denis_v1_bringup'), 'config', 'denis_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},controller_config_path]
)

    controller_manager_spawn1 = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])

    controller_manager_spawn2 = Node(package='controller_manager', executable='spawner', arguments=['diff_drive_controller'])

    rviz_config = os.path.join(get_package_share_path('denis_v1_description'), 'rviz', 'config.rviz')

    rviz2 = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config])

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        controller_manager_spawn1,
        controller_manager_spawn2,
        rviz2
    ])