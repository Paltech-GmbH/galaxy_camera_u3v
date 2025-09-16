"""Launch uv3_image_pub stereo left & right in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    """Generate launch description with multiple components."""
    camera_params = [
        {"acquisition_frame_rate": 3.0},
        {"topic": ""},
        {"device_sn": "FCQ24082069"},
    ]

    front_camera_node = Node(
        package="galaxy_camera_u3v",
        executable="u3v_image_pub",
        name="front_camera_pub",
        namespace=PythonExpression(expression=["'", namespace, "'", " + '/cam_front'"]),
        parameters=camera_params,
    )

    return launch.LaunchDescription([declare_namespace_cmd, front_camera_node])
