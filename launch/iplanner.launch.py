import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    config_file = get_share_file(
        package_name="iplanner", file_name="config/vehicle_sim.yaml"
    )
    rviz_config_file = get_share_file(
        package_name="iplanner", file_name="rviz/default.rviz"
    )

    USE_PATH_FOLLOW = False
    # = DeclareLaunchArgument(
    #     "use_path_follow",
    #     default_value=True,
    #     description="Param for whether or not to use path following"
    # )

    # Temporary way to connect launches
    path_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("iplanner_path_follow" + "/launch/path_follower.launch")
    )

    iplanner = Node(
        package="iplanner",
        executable="iplanner_node.py",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": True
            }
        ],
        respawn=False,
        emulate_tty=True,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(iplanner)
    launch_description.add_action(rviz)
    if USE_PATH_FOLLOW: launch_description.add_action(path_follow_launch)

    return LaunchDescription([launch_description])
