import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for perception server.
    This function creates a launch description for the perception server.
    It loads the configuration file and creates a composable node container
    for the perception server.

    Returns:
        LaunchDescription: The launch description for perception server.
    """
    # load config file
    perception_config = os.path.join(get_package_share_directory('perception'), 'config', 'config.yaml')

    # create perception node
    perception_server = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        parameters=[perception_config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # create launch description
    # return
    return LaunchDescription([
        perception_server,
    ])