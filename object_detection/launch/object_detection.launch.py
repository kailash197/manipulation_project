import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
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

    static_transform_node = TimerAction(
        period=3.0,  # Delay of 3 seconds
        actions=[
            Node(
                package="object_detection",
                executable="static_transform_publisher",
                name="static_transform_node",
                output="screen",
                parameters=[{
                    'use_sim_time': True},
                ],
            )
        ]
    )

    object_detection_node = TimerAction(
        period=0.0,  # No Delay
        actions=[
            Node(
                package="object_detection",
                executable="object_detector",
                name="object_detection_node",
                output="screen",
                parameters=[{
                    'use_sim_time': False},
                ],
            )
        ]
    )

    return LaunchDescription(
        [rviz_node,
        object_detection_node,
        static_transform_node
        ]
    )
