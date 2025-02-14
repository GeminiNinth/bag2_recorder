#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bag2_recorder.msg import Rosbag
from .bag2_recorder import Bag2Recorder

import os
from typing import Dict, List, Optional, Set
from dataclasses import dataclass
from enum import Enum, auto


class StopReason(Enum):
    """Recording stop reason."""

    TOPIC = auto()
    TIMER = auto()


@dataclass
class BLOptions:
    """Bag Launcher Options."""

    configuration_directory: str = "config"
    data_directory: str = "rosbag"
    storage_format: str = "mcap"
    record_start_topic: str = "/record/start"
    record_stop_topic: str = "/record/stop"
    publish_name: bool = False
    name_topic: str = "/record/name"
    seconds: int = 0
    start_now: bool = False
    finish_after_record: bool = True
    topic_discovery_timeout: float = 5.0
    show_topics: bool = False
    config_files: List[str] = None  # Default is None (uses standard config file)

    def __post_init__(self):
        if self.config_files is None:
            self.config_files = ["standard"]


class BagLauncher(Node):
    """Main node for controlling bag recording."""

    def __init__(self, options: BLOptions):
        super().__init__("rosbag2_recorder_node")

        # Store options
        self._options = options
        self._config_location = os.path.join(options.configuration_directory, "")
        self._data_folder = os.path.join(options.data_directory, "")
        self._storage_format = options.storage_format
        self._seconds = options.seconds
        self._finish_after_record = options.finish_after_record
        self._topic_discovery_timeout = options.topic_discovery_timeout
        self._shutdown_requested = False

        # Create publishers and subscribers
        self._record_start_sub = self.create_subscription(
            Rosbag, options.record_start_topic, self._start_recording_callback, 10
        )

        self._record_stop_sub = self.create_subscription(
            String, options.record_stop_topic, self._stop_recording_callback, 10
        )

        self._name_publisher = None
        if options.publish_name:
            self._name_publisher = self.create_publisher(
                Rosbag, self._sanitize_topic(options.name_topic), 10
            )

        # Initialize storage
        self._recorders: Dict[str, Bag2Recorder] = {}
        self._auto_stop_timers: Dict[str, rclpy.timer.Timer] = {}
        self._recording_start_times: Dict[str, float] = {}

        # Start recording if requested
        if options.start_now:
            self._auto_start_recording()

    def _sanitize_topic(self, topic: str) -> str:
        """Ensure topic starts with '/'."""
        return f"/{topic}" if not topic or topic[0] != "/" else topic

    def _start_recording_callback(self, msg: Rosbag):
        """Handle start recording requests."""
        # Create new recorder if needed
        if msg.config not in self._recorders:
            self._recorders[msg.config] = Bag2Recorder(
                self,
                self._data_folder,
                self._storage_format,
                True,
                self._topic_discovery_timeout,
            )

        # Check if already recording
        if self._recorders[msg.config].is_active():
            self.get_logger().warn(
                f"Recorder for config '{msg.config}' is already active"
            )
            return

        # Load topics from config
        topics: List[str] = []
        try:
            if not self._load_config(msg.config, topics):
                self.get_logger().error(f"Failed to load config '{msg.config}'")
                return

            # Exit with warning if topic list is empty
            if not topics:
                self.get_logger().warn(
                    f"No topics specified in config '{msg.config}', skipping this config"
                )
                return

            # トピック数を表示
            self.get_logger().info(
                f"Found {len(topics)} topics in config '{msg.config}'"
            )
            full_bag_name = self._recorders[msg.config].start_recording(
                msg.bag_name, topics
            )

        except Exception as e:
            self.get_logger().error(
                f"Exception while starting recording for config '{msg.config}': {str(e)}"
            )
            return

        if not full_bag_name:
            self.get_logger().warn(f"Recording did not start for config '{msg.config}'")
            return

        # Store start time
        self._recording_start_times[msg.config] = self.get_clock().now().nanoseconds

        # Publish bag name if requested
        if self._name_publisher:
            out_msg = Rosbag()
            out_msg.config = msg.config
            out_msg.bag_name = full_bag_name
            self._name_publisher.publish(out_msg)

        self.get_logger().info(
            f"Started recording for config '{msg.config}' to '{full_bag_name}'"
        )

        # Setup auto-stop if needed
        if self._seconds > 0:
            self._setup_auto_stop_timer(msg.config)

    def _stop_recording(self, config: str, reason: StopReason):
        """Stop recording with specified reason."""
        if config not in self._recorders:
            self.get_logger().info(f"No recorder found for config '{config}'")
            return

        # Calculate elapsed time
        elapsed_seconds = 0
        if config in self._recording_start_times:
            current_time = self.get_clock().now().nanoseconds
            elapsed_seconds = (current_time - self._recording_start_times[config]) / 1e9
            del self._recording_start_times[config]

        # Stop recording
        self._recorders[config].stop_recording()

        # Log appropriate message based on reason
        if reason == StopReason.TOPIC:
            self.get_logger().info(
                f"Stopped recording for config '{config}' due to stop topic received "
                f"(Recording duration: {elapsed_seconds:.1f} seconds)"
            )
        else:  # StopReason.TIMER
            self.get_logger().info(
                f"Stopped recording for config '{config}' due to timer expiration "
                f"(Requested duration: {self._seconds} seconds, "
                f"Actual duration: {elapsed_seconds:.1f} seconds)"
            )

        # Handle shutdown if needed
        if self._finish_after_record and not self._shutdown_requested:
            self.get_logger().info("Recording finished, requesting shutdown...")
            self._shutdown_requested = True
            # Use a one-shot timer to allow the message to be processed
            self.create_timer(0.1, self._delayed_shutdown)

    def _stop_recording_callback(self, msg: String):
        """Handle stop recording requests."""
        self._stop_recording(msg.data, StopReason.TOPIC)

    def _delayed_shutdown(self):
        """Perform delayed shutdown to allow message processing."""
        try:
            # Cleanup all recorders
            for recorder in self._recorders.values():
                if recorder.is_active():
                    recorder.stop_recording()

            # Cancel all timers
            for timer in self._auto_stop_timers.values():
                timer.cancel()

            # Request shutdown
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")

    def _load_config(
        self,
        config_name: str,
        topics: List[str],
        loaded_configs: Optional[Set[str]] = None,
    ) -> bool:
        """Load topics from config files.

        Args:
            config_name: Name of the config file to load
            topics: List to store found topics
            loaded_configs: Set to track loaded configs (unused in new implementation)
        """
        filename = os.path.join(self._config_location, f"{config_name}.config")
        total_topics = set()  # Store all topics
        file_topics: Dict[str, List[str]] = {}  # Store topics for each file

        try:
            with open(filename, "r") as f:
                lines = [line.strip() for line in f.readlines()]
                # Filter out empty lines and comments
                lines = [
                    line
                    for line in lines
                    if line and not line.startswith("#") and not line.startswith(" ")
                ]

                if not lines:
                    self.get_logger().warn(f"Config file '{filename}' is empty")
                    file_topics[config_name] = []
                    return True

                file_topics[config_name] = []
                for line in lines:
                    if line == "*":
                        self.get_logger().info(
                            f"Wildcard (*) detected in '{filename}', will record all topics"
                        )
                        topics.clear()
                        topics.append("*")
                        return True
                    else:
                        sanitized_topic = self._sanitize_topic(line)
                        topics.append(sanitized_topic)
                        file_topics[config_name].append(sanitized_topic)
                        total_topics.add(sanitized_topic)

                # Display only the number of topics
                self.get_logger().info(
                    f"Found {len(file_topics[config_name])} topics in '{config_name}.config'"
                )

                return True

        except FileNotFoundError:
            self.get_logger().error(f"Config file '{filename}' not found")
            return False

    def _auto_start_recording(self):
        """Start recording automatically."""
        all_topics = []  # Accumulate topics from all config files
        valid_configs = []  # Config files with valid topics
        total_topics = set()  # Set of unique topics

        # Collect topics from all config files
        for config_file in self._options.config_files:
            topics = []
            if self._load_config(config_file, topics):
                if topics:  # トピックが存在する場合
                    all_topics.extend(topics)
                    valid_configs.append(config_file)
                    total_topics.update(topics)

        # 統計情報の表示
        total_specified = len(all_topics)
        unique_topics = len(total_topics)
        duplicate_topics = total_specified - unique_topics

        self.get_logger().info(
            f"\nCombined Topic Statistics:\n"
            f"Total topics specified across all files: {total_specified}\n"
            f"Unique topics: {unique_topics}\n"
            f"Duplicate topics: {duplicate_topics}\n"
            f"Topics to be recorded: {unique_topics}"
            + (
                f":\n{chr(10).join(sorted(total_topics))}"
                if self._options.show_topics
                else ""
            )
        )

        # If no valid topics found
        if not total_topics:
            self.get_logger().warn("No valid topics found in any config files")
            self.get_logger().info("Recording finished, requesting shutdown...")
            self._shutdown_requested = True
            self.create_timer(0.1, self._delayed_shutdown)
            return

        # Start recording with combined topics
        combined_config = "+".join(valid_configs)  # Combine valid config names
        msg = Rosbag()
        msg.config = combined_config
        msg.bag_name = "auto_record"

        # Create new recorder instance
        self._recorders[combined_config] = Bag2Recorder(
            self,
            self._data_folder,
            self._storage_format,
            True,
            self._topic_discovery_timeout,
        )

        # Allow referencing the same recorder by individual config names
        for config in valid_configs:
            self._recorders[config] = self._recorders[combined_config]

        # Start recording
        full_bag_name = self._recorders[combined_config].start_recording(
            msg.bag_name, sorted(total_topics)
        )

        if full_bag_name:
            self._recording_start_times[combined_config] = (
                self.get_clock().now().nanoseconds
            )
            # Record the same start time for individual config names
            for config in valid_configs:
                self._recording_start_times[config] = self._recording_start_times[
                    combined_config
                ]

            self.get_logger().info(
                f"Started recording with combined topics from configs: {', '.join(valid_configs)}"
            )
            if self._seconds > 0:
                self._setup_auto_stop_timer(combined_config)
        else:
            self.get_logger().error("Failed to start recording")
            self._shutdown_requested = True
            self.create_timer(0.1, self._delayed_shutdown)

    def _setup_auto_stop_timer(self, config: str):
        """Setup timer for automatic recording stop."""
        if self._seconds <= 0:
            return

        # Create timer
        self.get_logger().info(
            f"Auto-stop timer set for {self._seconds} seconds for config '{config}'"
        )
        timer = self.create_timer(
            self._seconds, lambda: self._stop_recording(config, StopReason.TIMER)
        )
        self._auto_stop_timers[config] = timer


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        # Create temporary node to get parameters
        temp_node = rclpy.create_node("temp_param_node")

        try:
            # Declare parameters with default values
            default_options = BLOptions()
            temp_node.declare_parameter(
                "configuration_directory", default_options.configuration_directory
            )
            temp_node.declare_parameter(
                "data_directory", default_options.data_directory
            )
            temp_node.declare_parameter(
                "storage_format", default_options.storage_format
            )
            temp_node.declare_parameter(
                "start_bag_topic", default_options.record_start_topic
            )
            temp_node.declare_parameter(
                "stop_bag_topic", default_options.record_stop_topic
            )
            temp_node.declare_parameter("publish_name", default_options.publish_name)
            temp_node.declare_parameter("name_topic", default_options.name_topic)
            temp_node.declare_parameter("seconds", default_options.seconds)
            temp_node.declare_parameter("start_now", default_options.start_now)
            temp_node.declare_parameter(
                "finish_after_record", default_options.finish_after_record
            )
            temp_node.declare_parameter(
                "topic_discovery_timeout", default_options.topic_discovery_timeout
            )
            temp_node.declare_parameter("show_topics", default_options.show_topics)
            temp_node.declare_parameter("config_files", "standard")

            # Create options and update from parameters
            options = BLOptions(
                configuration_directory=temp_node.get_parameter(
                    "configuration_directory"
                ).value,
                data_directory=temp_node.get_parameter("data_directory").value,
                storage_format=temp_node.get_parameter("storage_format").value,
                record_start_topic=temp_node.get_parameter("start_bag_topic").value,
                record_stop_topic=temp_node.get_parameter("stop_bag_topic").value,
                publish_name=temp_node.get_parameter("publish_name").value,
                name_topic=temp_node.get_parameter("name_topic").value,
                seconds=int(temp_node.get_parameter("seconds").value),
                start_now=temp_node.get_parameter("start_now").value,
                finish_after_record=temp_node.get_parameter(
                    "finish_after_record"
                ).value,
                topic_discovery_timeout=float(
                    temp_node.get_parameter("topic_discovery_timeout").value
                ),
                show_topics=temp_node.get_parameter("show_topics").value,
                config_files=[
                    x.strip()
                    for x in temp_node.get_parameter("config_files").value.split(",")
                    if x.strip()  # Exclude empty elements (handles trailing commas)
                ],
            )

            # Cleanup temporary node
            temp_node.destroy_node()

            # Create and spin node
            node = BagLauncher(options)
            rclpy.spin(node)

            # Cleanup
            node.destroy_node()

        except Exception as e:
            temp_node.get_logger().error(f"Failed to initialize node: {str(e)}")
            temp_node.destroy_node()

    except Exception as e:
        print(f"Error during initialization: {str(e)}")
    finally:
        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
