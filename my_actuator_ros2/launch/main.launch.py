from launch import LaunchDescription
from launch_ros.actions import Node

# Motor IDs and directions
right_front_wheel_id = 2
left_front_wheel_id = 1
right_rear_wheel_id = 3
left_rear_wheel_id = 4

wheel_base = 0.2913
wheel_radius = 0.0525
right_front_wheel_dir = 1
left_front_wheel_dir = 1
right_rear_wheel_dir = 1
left_rear_wheel_dir = 1

# Define parameters for the actuators
paramData = [{
    '/my_actuator/right_front_wheel/id': right_front_wheel_id,
    '/my_actuator/left_front_wheel/id': left_front_wheel_id,
    '/my_actuator/right_rear_wheel/id': right_rear_wheel_id,
    '/my_actuator/left_rear_wheel/id': left_rear_wheel_id,
    '/my_actuator/right_front_wheel/dir': right_front_wheel_dir,
    '/my_actuator/left_front_wheel/dir': left_front_wheel_dir,
    '/my_actuator/right_rear_wheel/dir': right_rear_wheel_dir,
    '/my_actuator/left_rear_wheel/dir': left_rear_wheel_dir,
    '/my_actuator/wheel_base': wheel_base,
    '/my_actuator/wheel_radius': wheel_radius,
}]


joy_teleop_params = [{
    'axis_linear': 3,
    'axis_angular': 0,
}]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_actuator_ros2',
            executable='my_actuator_control',
            name='my_actuator_control',
            output='screen',
            parameters=paramData,
            remappings=[],
        ),
        Node(
            package='my_actuator_ros2',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=joy_teleop_params,  
            remappings=[],
        ),
    ])
