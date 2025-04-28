"""Launch uv3_image_pub stereo left & right in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    """Generate launch description with multiple components."""
    camera_params = [
        {"acquisition_frame_rate": 10.0},
        {"topic": ""},
        {"device_sn": "FCQ24082064"},
    ]

    container1 = ComposableNodeContainer(
        name="stereo_image_container",
        namespace=PythonExpression(expression=["'", namespace, "'", " + '/cam_front'"]),
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="galaxy_camera_u3v",
                plugin="camera::U3vImagePub",
                name="right_image_pub",
                namespace=PythonExpression(
                    expression=["'", namespace, "'", " + '/cam_front'"]
                ),
                parameters=camera_params,
            )
        ],
    )

    return launch.LaunchDescription([declare_namespace_cmd, container1])
