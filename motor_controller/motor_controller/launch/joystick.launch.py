from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory('motor_controller'),
        'config',
        'joystick.yaml'
    )

    twist_mux_params = os.path.join(
        get_package_share_directory('motor_controller'),
        'config',
        'twist_mux.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/openrov/cmd_vel')]
    )

    twist_to_pwm_node = Node(
        package='motor_controller',
        executable='twist_to_pwm',
        name='twist_to_pwm',
        output='screen',
        parameters=[{
            'max_speed': 1900.0,
            'min_speed': 1100.0,
            'neutral': 1500.0,
            'thruster_count': 3
        }],
        remappings=[
            ('/cmd_vel', '/openrov/cmd_vel')
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_to_pwm_node
    ])
