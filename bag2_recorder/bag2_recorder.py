#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rosbag2_py
from rosbag2_py import StorageOptions, RecordOptions, ConverterOptions

from datetime import datetime
import os
from typing import List, Dict, Set
import importlib


class Bag2Recorder:
    """
    A recorder class for capturing ROS2 topics using rosbag2.

    This class provides a high-level interface for recording ROS2 topics, with features
    designed to handle common recording scenarios and edge cases in robotics applications.

    Key Features:
    - Automatic topic recording with dynamic discovery
    - Support for all topics or specific topic selection
    - Flexible storage formats (mcap or db3)
    - Robust topic type discovery with timeout
    - Automatic handling of QoS profiles

    ROS2 Concepts Used:
    - Nodes: Basic units in ROS2 that can publish/subscribe to topics
    - Topics: Named buses over which nodes exchange messages
    - Messages: Typed data structures for communication
    - QoS (Quality of Service): Policies that control message delivery
    - Serialization: Converting ROS messages to/from binary format

    Example Usage:
        >>> # Initialize the recorder

        >>> recorder = Bag2Recorder(node, "/path/to/bags")

        >>> # Record specific topics

        >>> recorder.start_recording("my_recording", ["/cmd_vel", "/camera/image_raw"])

        >>> # Record all available topics

        >>> recorder.start_recording("all_topics", [], record_all_topics=True)

        >>> # Stop recording when done

        >>> recorder.stop_recording()
    """

    def __init__(
        self,
        node: Node,
        data_folder: str,
        storage_format: str = "mcap",
        append_date: bool = True,
        topic_discovery_timeout: float = 5.0,
    ):
        """Initialize the Bag2Recorder with specified configuration.

        Args:
            node: ROS2 node instance used for logging and topic management.
                 The node provides access to ROS2 communication facilities.

            data_folder: Directory where bag files will be saved.
                        Will be created if it doesn't exist.

            storage_format: Format for saving bag files. Options:
                - "mcap": Modern format optimized for performance
                    + Single file storage
                    + Better compression
                    + Faster read/write
                - "db3": Traditional SQLite-based format
                    + More compatible with older tools
                    + Multiple file storage

            append_date: If True, adds timestamp to bag filenames
                        Format: {name}_{YYYYMMDD_HH-MM-SS}

            topic_discovery_timeout: Maximum time (seconds) to wait
                                   for topics to become available.
                                   Prevents hanging on missing topics.
        """
        self._node = node
        self._data_folder = data_folder
        self._storage_format = storage_format
        self._append_date = append_date
        self._topic_discovery_timeout = topic_discovery_timeout

        # Internal state tracking
        self._bag_active = False  # Whether recording is currently active
        self._writer = None  # rosbag2 writer instance
        self._bag_filename = ""  # Current bag file path

        # Subscriber management
        self._subscribers: Dict[str, rclpy.subscription.Subscription] = {}
        self._subscribed_topics: Set[str] = set()

        # Ensure data directory exists
        os.makedirs(data_folder, exist_ok=True)

    def start_recording(
        self, bag_name: str, topics: List[str], record_all_topics: bool = False
    ) -> str:
        """Start recording specified topics to a bag file.

        This method handles the complete recording setup process:
        1. Generate unique bag filename (optionally with timestamp)
        2. Initialize rosbag2 writer with storage options
        3. Discover and subscribe to specified topics
        4. Begin recording messages

        ROS2 Concepts:
        - Topic Discovery: Dynamically finding available topics
        - Message Types: Auto-detection of message definitions
        - Serialization: Converting ROS messages to storable format

        Args:
            bag_name: Base name for the bag file
                     If append_date=True: {bag_name}_{timestamp}

            topics: List of topics to record
                   Each topic should be a valid ROS2 topic name
                   e.g., ["/cmd_vel", "/camera/image_raw"]

            record_all_topics: If True, records all available topics
                             Ignores the topics parameter
                             Useful for system-wide recording

        Returns:
            str: Full path to created bag file
                 Empty string if initialization failed

        Note:
            - Topics not immediately available will be waited for
              up to topic_discovery_timeout seconds
            - Already active recording must be stopped first
        """
        if self._bag_active:
            self._node.get_logger().warn("Recording is already active")
            return ""

        # Generate bag filename with optional timestamp
        if self._append_date:
            timestamp = datetime.now().strftime("%Y%m%d_%H-%M-%S")
            bag_name = f"{bag_name}_{timestamp}"

        self._bag_filename = os.path.join(self._data_folder, bag_name)

        try:
            # Initialize rosbag2 writer
            self._writer = rosbag2_py.SequentialWriter()

            # Configure storage options
            storage_options = StorageOptions(
                uri=self._bag_filename, storage_id=self._storage_format
            )

            # Set up serialization format
            converter_options = ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            )

            # Open the bag file for writing
            self._writer.open(storage_options, converter_options)

            # Allow time for topic discovery
            import time

            time.sleep(1.0)

            # Get available topics and their types
            topic_names_and_types = self._node.get_topic_names_and_types()
            topic_dict = {
                name: types[0] for name, types in topic_names_and_types if types
            }

            # Set up subscriptions
            if not topics:
                # Empty topic list means subscribe to all
                for topic_name, topic_type in topic_dict.items():
                    self._subscribe_to_topic(topic_name, topic_type)
            elif len(topics) == 1 and topics[0] == "*":
                # Wildcard means subscribe to all
                for topic_name, topic_type in topic_dict.items():
                    self._subscribe_to_topic(topic_name, topic_type)
            else:
                # Subscribe only to specified topics
                for topic in topics:
                    topic_type = topic_dict.get(topic, "")
                    self._subscribe_to_topic(topic, topic_type)

            self._bag_active = True
            return self._bag_filename

        except Exception as e:
            self._node.get_logger().error(f"Failed to start recording: {str(e)}")
            return ""

    def stop_recording(self):
        """Stop the current recording and clean up resources.

        This method:
        1. Cleans up all topic subscriptions
        2. Closes the bag file
        3. Resets internal state

        Note:
            Safe to call even if recording is not active
        """
        if not self._bag_active:
            return

        # Clean up subscribers
        for sub in self._subscribers.values():
            self._node.destroy_subscription(sub)
        self._subscribers.clear()
        self._subscribed_topics.clear()

        # Close the bag file
        if self._writer:
            self._writer.close()
            self._writer = None

        self._bag_active = False

    def _get_topic_type(self, topic: str) -> str:
        """Discover topic type with timeout mechanism.

        ROS2 requires matching message types for communication.
        This method attempts to discover the correct type for a topic,
        waiting up to topic_discovery_timeout seconds.

        Args:
            topic: The full topic name to look up

        Returns:
            str: Message type (e.g., 'sensor_msgs/msg/Image')
                 Empty string if type not found within timeout
        """
        import time

        start_time = time.time()
        check_interval = 0.1  # Check every 100ms

        while (time.time() - start_time) < self._topic_discovery_timeout:
            topic_names_and_types = self._node.get_topic_names_and_types()
            topic_info = next(
                (info for info in topic_names_and_types if info[0] == topic),
                None,
            )

            if topic_info and topic_info[1]:
                return topic_info[1][0]

            time.sleep(check_interval)

            # Log progress during discovery
            elapsed = time.time() - start_time
            if elapsed % 1.0 < check_interval:
                remaining = self._topic_discovery_timeout - elapsed
                self._node.get_logger().warn(
                    f"Waiting for topic type: {topic}, {remaining:.1f} seconds remaining"
                )

        self._node.get_logger().error(
            f"Topic discovery timeout ({self._topic_discovery_timeout}s) exceeded for topic: {topic}"
        )
        return ""

    def _subscribe_to_topic(self, topic: str, topic_type: str):
        """Set up a subscription for recording a single topic.

        This method handles:
        1. Topic type discovery if not provided
        2. Dynamic message type import
        3. Subscription setup with appropriate QoS
        4. Bag file topic registration

        ROS2 Concepts:
        - Dynamic Message Loading: Import message types at runtime
        - QoS Profiles: Configure reliable message delivery
        - Topic Registration: Set up metadata for recording

        Args:
            topic: Full topic name to subscribe to
            topic_type: Message type (if known) or empty string
        """
        if topic in self._subscribed_topics:
            return

        try:
            # Attempt topic type discovery if not provided
            if not topic_type:
                topic_type = self._get_topic_type(topic)
                if not topic_type:
                    return

            # Dynamically import message type
            parts = topic_type.split("/")
            if len(parts) != 3:
                raise ValueError(f"Invalid message type format: {topic_type}")

            module_name = f"{parts[0]}.msg"
            class_name = parts[2]

            try:
                module = importlib.import_module(module_name)
                msg_type = getattr(module, class_name)
            except Exception as e:
                self._node.get_logger().error(
                    f"Failed to import message type {topic_type}: {str(e)}"
                )
                return

            # Register topic with rosbag2
            topic_info = rosbag2_py.TopicMetadata(
                id=len(self._subscribed_topics),
                name=topic,
                type=topic_type,
                serialization_format="cdr",
            )
            self._writer.create_topic(topic_info)

            # Configure QoS profile for subscription
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

            # Create the subscription
            sub = self._node.create_subscription(
                msg_type=msg_type,
                topic=topic,
                callback=lambda msg: self._subscriber_callback(msg, topic),
                qos_profile=qos,
            )

            # Store subscription references
            self._subscribers[topic] = sub
            self._subscribed_topics.add(topic)

        except Exception as e:
            self._node.get_logger().error(f"Failed to subscribe to {topic}: {str(e)}")

    def _subscriber_callback(self, msg, topic: str):
        """Handle incoming messages for recording.

        This callback:
        1. Gets current ROS time
        2. Serializes the message
        3. Writes to the bag file

        Args:
            msg: The received message (any ROS message type)
            topic: Topic the message was received on
        """
        if not self._bag_active:
            return

        try:
            # Get ROS time in nanoseconds
            timestamp = self._node.get_clock().now().nanoseconds

            # Write serialized message to bag
            self._writer.write(topic, serialize_message(msg), timestamp)
        except Exception as e:
            self._node.get_logger().error(f"Error in subscriber callback: {str(e)}")

    def is_active(self) -> bool:
        """Check if recording is currently active.

        Returns:
            bool: True if recording, False otherwise
        """
        return self._bag_active

    def get_bagname(self) -> str:
        """Get the current bag filename.

        Returns:
            str: Full path to current bag file
        """
        return self._bag_filename

    def get_storage_format(self) -> str:
        """Get the current storage format.

        Returns:
            str: Storage format ("mcap" or "db3")
        """
        return self._storage_format
