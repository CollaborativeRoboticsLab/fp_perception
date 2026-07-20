import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # load config file
    perception_config = os.path.join(get_package_share_directory('fp_perception'), 'config', 'config.yaml')

    # create bridge composition
    perception_server = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='fp_perception',
                plugin='fp_perception::PerceptionServer',
                name='perception',
                parameters=[perception_config]
            )
        ]
    )
    
    # create launch description
    # return
    return LaunchDescription([
        perception_server,
    ])