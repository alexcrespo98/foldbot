from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Controllers
        Node(
            package='foldbot',
            executable='main_controller',
            name='main_controller',
            output='screen'
        ),
        Node(
            package='foldbot',
            executable='gantry_controller',
            name='gantry_controller',
            output='screen'
        ),
        Node(
            package='foldbot',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        ),
        Node(
            package='foldbot',
            executable='solenoid_controller',
            name='solenoid_controller',
            output='screen'
        ),
        Node(
            package='foldbot',
            executable='vacuum_controller',
            name='vacuum_controller',
            output='screen'
        ),

        Node(
            package='foldbot',
            executable='left_limit_switch',
            name='left_limit_switch',
            output='screen'
        ),
        Node(
            package='foldbot',
            executable='right_limit_switch',
            name='right_limit_switch',
            output='screen'
        ),
        Node(
            package='foldbot',
            executable='napkin_sensor',
            name='napkin_sensor',
            output='screen'
        ),
    ])

