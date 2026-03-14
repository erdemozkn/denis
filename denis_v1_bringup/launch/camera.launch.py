from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='front_camera',
            namespace='camera',
            parameters=[{
                'camera': '/base/soc/i2c0mux/i2c@1/imx219@10',
                'width': 640,
                'height': 480,
                'format': 'RGB888',
                'fps': 30,
            }],
            remappings=[
                ('/camera/front_camera/image_raw', '/camera/image_raw'),
            ]
        )
    ])