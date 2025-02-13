from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("bag2_recorder")

    config_dir_arg = DeclareLaunchArgument(
        "configuration_directory",
        default_value=os.path.join(pkg_dir, "config"),
        description="Directory containing configuration files for the bag launcher to read",
    )

    data_dir_arg = DeclareLaunchArgument(
        "data_directory",
        default_value=os.path.expanduser("~/data/"),
        description="Directory for the bags to be recorded to",
    )

    storage_format_arg = DeclareLaunchArgument(
        "storage_format",
        default_value="mcap",
        description="Storage format for rosbag (mcap or db3). mcap is recommended for better performance and single-file storage",
        choices=["mcap", "db3"],
    )

    start_topic_arg = DeclareLaunchArgument(
        "start_bag_topic",
        default_value="/record/start",
        description="Topic to listen to for starting recording. Publish any message to this topic to start recording",
    )

    stop_topic_arg = DeclareLaunchArgument(
        "stop_bag_topic",
        default_value="/record/stop",
        description="Topic to listen to for stopping recording. Publish any message to this topic to stop recording",
    )

    publish_name_arg = DeclareLaunchArgument(
        "publish_name",
        default_value="true",
        description="Whether to publish the current bag filename to a topic. Useful for monitoring active recordings",
    )

    name_topic_arg = DeclareLaunchArgument(
        "name_topic",
        default_value="/record/bag_name",
        description="Topic where the current bag filename will be published if publish_name is true",
    )

    default_record_all_arg = DeclareLaunchArgument(
        "default_record_all",
        default_value="false",
        description="If true, records all available topics when config file is not found. If false, exits with error",
    )

    seconds_arg = DeclareLaunchArgument(
        "seconds",
        default_value="30",
        description="Recording duration in seconds. Set to 0 for indefinite recording (until manually stopped)",
    )

    start_now_arg = DeclareLaunchArgument(
        "start_now",
        default_value="true",
        description="If true, starts recording immediately. If false, waits for message on start_bag_topic",
    )

    finish_after_record_arg = DeclareLaunchArgument(
        "finish_after_record",
        default_value="true",
        description="If true, shuts down after recording (one-shot). If false, stays alive for multiple recordings",
    )

    topic_discovery_timeout_arg = DeclareLaunchArgument(
        "topic_discovery_timeout",
        default_value="5.0",
        description="Maximum time (seconds) to wait for configured topics to become available",
    )

    # Node restart configuration
    # Automatically determined based on `finish_after_record` arg
    # Enables automatic recovery from crashes when running continuously
    respawn = PythonExpression(
        [
            "'false' if '",
            LaunchConfiguration("finish_after_record"),
            "' == 'true' else 'true'",
        ]
    )

    bag2_recorder_node = Node(
        package="bag2_recorder",
        executable="bag2_recorder_py_node",
        name="rosbag2_recorder_node",
        parameters=[
            {
                "configuration_directory": LaunchConfiguration(
                    "configuration_directory"
                ),
                "data_directory": LaunchConfiguration("data_directory"),
                "storage_format": LaunchConfiguration("storage_format"),
                "start_bag_topic": LaunchConfiguration("start_bag_topic"),
                "stop_bag_topic": LaunchConfiguration("stop_bag_topic"),
                "publish_name": LaunchConfiguration("publish_name"),
                "name_topic": LaunchConfiguration("name_topic"),
                "default_record_all": LaunchConfiguration("default_record_all"),
                "seconds": LaunchConfiguration("seconds"),
                "start_now": LaunchConfiguration("start_now"),
                "finish_after_record": LaunchConfiguration("finish_after_record"),
                "topic_discovery_timeout": LaunchConfiguration(
                    "topic_discovery_timeout"
                ),
            }
        ],
        output="screen",
        respawn=respawn,
    )

    return LaunchDescription(
        [
            config_dir_arg,
            data_dir_arg,
            storage_format_arg,
            start_topic_arg,
            stop_topic_arg,
            publish_name_arg,
            name_topic_arg,
            default_record_all_arg,
            seconds_arg,
            start_now_arg,
            finish_after_record_arg,
            topic_discovery_timeout_arg,
            bag2_recorder_node,
        ]
    )
