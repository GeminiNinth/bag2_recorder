# ROS2 Bag Recorder

## Installing/Running the Recorder
Clone this repository into the src/ folder of a ROS2 workspace. In the ROS2 workspace run the following commands.
```bash
colcon build
source install/setup.bash
```
The code should compile. You can then run the node using ros2 launch.
```bash
ros2 launch bag2_recorder default.launch.py
```

## Launch File Usage
To use this ROS2 bag recorder you need to modify the launch file to fit your project. The following parameters can be configured:

- `data_directory`: Directory where the bags will be stored (default: ~/data/)
- `configuration_directory`: Directory containing configuration files (default: package's config directory)
- `storage_format`: Storage format for rosbag (choices: ["mcap", "db3"], default: "mcap")
- `seconds`: Recording duration in seconds (default: 3, set to 0 to record until manually stopped)
- `start_now`: Whether to start recording automatically when launched (default: true)

If neither data_directory nor configuration_directory parameters are set then the node will not run.

When `start_now` is set to true, the recorder will automatically start recording using the "standard" configuration and "auto_record" as the bag name. If `seconds` is greater than 0, the recording will automatically stop after the specified duration. Note that even with automatic stop enabled, you can still manually stop the recording at any time.

## Storage Format
ROS2 Jazzy supports two storage formats for rosbag files:

- `mcap`: The default and recommended format. MCAP is a new serialization format designed for robotics data.
- `db3`: The legacy SQLite-based format from earlier ROS2 versions.

You can specify the format in the launch file:
```python
ros2 launch bag2_recorder default.launch.py storage_format:=mcap  # or storage_format:=db3
```

## Configuration File Usage
### Basic Usage
A configuration file is identified by its name, which is then followed by ".config". If you wanted a configuration called "standard" you would make a file "standard.config" in your configuration folder. In this document you can list out all the topics you want the rosbag2_recorder subscribed to.

Here is an example *standard.config*
```
/topic1
/topic2/subtopic1
/topic2/subtopic2
/topic3
/topic4
```

### Subscribe to All Topics
If you wanted this "standard" configuration to subscribe to all topics it could be the following:
```
*
```
A single line containing an asterisk. Note that current behavior also subscribes to all topics when the config file doesn't exist or is empty.

### Comments
You can write comments in the config file a few ways. With a '#' or a space at the beginning of the line. The parser ignores any blank lines or ones starting with ' ' or '#'.
```
# This is a comment
 This will also be ignored by the parser.
/This/will/be/subscribed/to
```

## Starting the Recorder
The start topic is set in the launch file as well.
```python
start_bag_topic:=/record/start
```
You can start the recorder by publishing to this topic with the rosbag message type. This message type has two key values, config and bag_name. Config is the name of the configuration you want the bag launcher to load and subscribe to. To load the config defined above (standard.config) you would set the config value to "standard". The bag_name value is the base name for the bag that will be recorded to. The software is also set up to append the timestamp to this bag name.

This command looks like:
```bash
ros2 topic pub /record/start bag2_recorder/msg/Rosbag "{config: 'standard', bag_name: 'test_bag'}"
```

To see what the full bag name is you can use the bag name publisher. To do this use the following launch options:
```python
publish_name:=true
name_topic:=/record/bag_name
```
This will publish the full bag name to name_topic.

To see this data you must echo this topic in another terminal window before the start command is sent:
```bash
ros2 topic echo /record/bag_name
```

### Starting the recorder programmatically (Python)
Here's an example of how to start recording using Python:

```python
import rclpy
from rclpy.node import Node
from bag2_recorder.msg import Rosbag

def main():
    rclpy.init()
    node = Node("test_node")
    
    bag_pub = node.create_publisher(Rosbag, "/record/start", 10)
    
    message = Rosbag()
    message.config = "standard"
    message.bag_name = "test_bag"
    
    try:
        while rclpy.ok():
            node.get_logger().info("Publishing...")
            bag_pub.publish(message)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Stopping the recorder
The stop topic is defined in the launch file as well.
```python
stop_bag_topic:=/record/stop
```
To stop a recording of a certain configuration you can publish that configuration to stop_bag_topic. This topic uses std_msgs::msg::String type. Set this message's data value to the configuration you want to stop recording.

This command looks like:
```bash
ros2 topic pub /record/stop std_msgs/msg/String "data: standard"
```

### Stopping the recorder programmatically (Python)
Here's an example of how to stop recording programmatically:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node("test_node")
    
    stop_pub = node.create_publisher(String, "/record/stop", 10)
    
    message = String()
    message.data = "standard"
    
    try:
        while rclpy.ok():
            node.get_logger().info("Publishing...")
            stop_pub.publish(message)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Notes

Tested on ROS2 jazzy.

This repository is a fork of the [bag_recorder](https://github.com/joshs333/bag_recorder.git) repository, which was originally implemented in C++ for ROS1.

In this fork, the project has been migrated to ROS2 and reimplemented in Python instead of C++ (because I'm not very good at C++!).
