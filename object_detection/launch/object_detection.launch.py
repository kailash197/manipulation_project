import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('object_detection')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'object_detection.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    static_transform_node = Node(
        name="static_transform_node",
        package="object_detection",
        executable="static_transform_publisher",
        output="screen",
        parameters=[{
            'use_sim_time': True},
        ],
    )

    object_detection_node = Node(
        name="object_detection_node",
        package="object_detection",
        executable="object_detector",
        output="screen",
        parameters=[{
            'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [static_transform_node,
        object_detection_node,
        rviz_node]
    )