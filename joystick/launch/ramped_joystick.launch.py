import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='JOYSTICK',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}]
        ),

        # Ramped Joystick node
        Node(
            package='joystick',
            executable='ramped_joystick.py',  # Убедитесь, что скрипт имеет права на выполнение
            name='RAMPED_JOYSTICK',
            output='screen',
        ),
    ])
