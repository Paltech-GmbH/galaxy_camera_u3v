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
    left_camera_node = Node(
        package="galaxy_camera_u3v",
        executable="u3v_image_pub",
        name="left_camera_pub",
        namespace=PythonExpression(
            expression=["'", namespace, "'", " + '/cam_front_left'"]
        ),
        parameters=[
            {"acquisition_frame_rate": 3.0},
            {"topic": ""},
            {"device_sn": "FCQ24082069"},
        ],
    )

    right_camera_node = Node(
        package="galaxy_camera_u3v",
        executable="u3v_image_pub",
        name="right_camera_pub",
        namespace=PythonExpression(
            expression=["'", namespace, "'", " + '/cam_front_right'"]
        ),
        parameters=[
            {"acquisition_frame_rate": 3.0},
            {"topic": ""},
            {"device_sn": "FCQ24102860"},
        ],
    )

    return launch.LaunchDescription(
        [declare_namespace_cmd, left_camera_node, right_camera_node]
    )
