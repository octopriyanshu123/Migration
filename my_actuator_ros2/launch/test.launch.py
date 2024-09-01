from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_actuator_ros2',
            executable='testk',
            name='testk',
            output='screen',
            parameters=[{
                '/my_actuator/right_front_wheel/id': 2,
                '/my_actuator/left_front_wheel/id': 1,
                '/my_actuator/right_rear_wheel/id': 3,
                '/my_actuator/left_rear_wheel/id': 4,
                '/my_actuator/right_front_wheel/dir': 1,
                '/my_actuator/left_front_wheel/dir': 1,
                '/my_actuator/right_rear_wheel/dir': 1,
                '/my_actuator/left_rear_wheel/dir': 1,
                '/my_actuator/wheel_base': 0.2913,
                '/my_actuator/wheel_radius': 0.0525,
            }],
        ),
    ])