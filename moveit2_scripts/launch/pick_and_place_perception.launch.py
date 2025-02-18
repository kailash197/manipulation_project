import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get the path to the launch file to be included
    package_dir = os.path.join(get_package_share_directory('object_detection'))
    perception_launch_file = os.path.join(package_dir, 'launch', 'object_detection.launch.py')

    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        name="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        # Include the perception launch file
        # Includes rviz2, static_publisher and object detector
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch_file)
        ),
        moveit_cpp_node
    ])
