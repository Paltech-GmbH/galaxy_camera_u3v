"""Launch uv3_image_pub stereo left & right in a component container."""

import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = LaunchConfiguration("params")
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace"
        ),
        DeclareLaunchArgument(
            "params",
            default_value=os.path.join(
                get_package_share_directory("galaxy_camera_u3v"),
                "params",
                "galaxy_node_params.yaml",
            ),
            description="Full path to configuration file for stereo cameras",
        ),
    ]
    """Generate launch description with multiple components."""
    left_camera_node = Node(
        package="galaxy_camera_u3v",
        executable="u3v_image_pub",
        name="galaxy_camera_pub",
        namespace="/nuga/front_left/sensors/camera",
        parameters=[params],
    )

    right_camera_node = Node(
        package="galaxy_camera_u3v",
        executable="u3v_image_pub",
        name="galaxy_camera_pub",
        namespace="/nuga/front_right/sensors/camera",
        parameters=[params],
    )

    return launch.LaunchDescription(
        declared_arguments + [left_camera_node, right_camera_node]
    )
