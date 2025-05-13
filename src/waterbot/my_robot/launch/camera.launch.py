from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',  # TODO: 替換成你的套件名
            executable='move.py',
            name='serial_twist_node',
            output='screen'
        ),
        Node(
            package='my_robot',  # TODO: 替換成你的套件名
            executable='camera.py',
            name='camera_ws_node',
            output='screen'
        ),
        Node(
            package='my_robot',  # TODO: 替換成你的套件名
            executable='yolorun.py',
            name='yolo_cmd_listener',
            output='screen'
        )
    ])
