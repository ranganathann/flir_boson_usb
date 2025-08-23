from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="flir_boson",
                description="Namespace for the nodes",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="boson_camera",
                description="Frame ID for the camera",
            ),
            DeclareLaunchArgument(
                "dev",
                default_value="/dev/video0",
                description="Linux file descriptor location for the camera",
            ),
            DeclareLaunchArgument(
                "frame_rate",
                default_value="60.0",
                description="Frame rate (valid values are 30.0 or 60.0 for Bosons)",
            ),
            DeclareLaunchArgument(
                "video_mode",
                default_value="YUV",
                description="Video mode (valid values are RAW16 or YUV)",
            ),
            DeclareLaunchArgument(
                "zoom_enable",
                default_value="false",
                description="Enable zoom (valid values are true or false)",
            ),
            DeclareLaunchArgument(
                "sensor_type",
                default_value="Boson_640",
                description="Sensor type (valid values are Boson_320 or Boson_640)",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="package://flir_boson_usb/example_calibrations/customBoson.yaml",
                description="Location of the camera calibration file",
            ),
            GroupAction(
                [
                    ComposableNodeContainer(
                        name="boson_container",
                        namespace=LaunchConfiguration("namespace"),
                        package="rclcpp_components",
                        executable="component_container",
                        composable_node_descriptions=[
                            ComposableNode(
                                package="flir_boson_usb",
                                plugin="flir_boson_usb::BosonCamera",
                                name="boson_camera",
                                parameters=[
                                    {
                                        "frame_id": LaunchConfiguration(
                                            "frame_id"
                                        ),
                                        "dev": LaunchConfiguration("dev"),
                                        "frame_rate": LaunchConfiguration(
                                            "frame_rate"
                                        ),
                                        "video_mode": LaunchConfiguration(
                                            "video_mode"
                                        ),
                                        "zoom_enable": LaunchConfiguration(
                                            "zoom_enable"
                                        ),
                                        "sensor_type": LaunchConfiguration(
                                            "sensor_type"
                                        ),
                                        "camera_info_url": LaunchConfiguration(
                                            "camera_info_url"
                                        ),
                                    }
                                ],
                            ),
                            ComposableNode(
                                package="image_proc",
                                plugin="image_proc::RectifyNode",
                                name="rectify_node",
                                remappings=[
                                    ("image", "/flir_boson/image_raw"),
                                    ("camera_info", "/flir_boson/camera_info"),
                                    # ("image_rect", "/flir_boson/image_rect"),
                                ],
                            ),
                        ],
                    )
                ]
            ),
        ]
    )
