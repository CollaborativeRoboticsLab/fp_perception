'''
listener launch file
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for events listener.

    Returns:
        LaunchDescription: The launch description for events listener
    """
    # create bridge composition
    listener = Node(
        package='perception_events',
        executable='perception_events_node',
        name='listener',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # return
    return LaunchDescription([
        listener
    ])
