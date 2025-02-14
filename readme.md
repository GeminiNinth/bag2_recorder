# ROS2 Bag Recorder

A ROS2 bag recording tool that allows you to specify topics to record using configuration files (`config/standard.config`) and control recording automatically or manually through launch file parameters.

## Note

Tested on ROS2 jazzy.

This repository is a fork of the [bag_recorder](https://github.com/joshs333/bag_recorder.git) repository, which was originally implemented in C++ for ROS1.

In this fork, the project has been migrated to ROS2 and reimplemented in Python instead of C++ (because I'm not very good at C++!).

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
- `seconds`: Recording duration in seconds (default: 10, set to 0 to record until manually stopped)
- `start_now`: Whether to start recording automatically when launched (default: true)
- `config_files`: List of configuration files to use (default: ["standard"])
- `show_topics`: Whether to display the list of topics being recorded (default: false)

If neither data_directory nor configuration_directory parameters are set then the node will not run.

When `start_now` is set to true, the recorder will automatically start recording using the specified configuration files and "auto_record" as the bag name. If `seconds` is greater than 0, the recording will automatically stop after the specified duration. Note that even with automatic stop enabled, you can still manually stop the recording at any time.

### Example Output and Explanation
Below are examples of standard output in various scenarios.

#### 1. Normal Recording with Auto-stop
```
[INFO] [launch]: All log files can be found below /home/gemini-ninth/.ros/log/2025-02-13-21-35-07-228364-gemini-ninth-01-187395
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [bag2_recorder_py_node-1]: process started with pid [187399]
```
↑ Basic startup information
- Log file location
- Log level setting
- Process ID information

```
[bag2_recorder_py_node-1] [INFO] [1739450108.324657797] [rosbag2_recorder_node]: Found 1 topics in 'standard.config'
[bag2_recorder_py_node-1] [INFO] [1739450108.324973709] [rosbag2_recorder_node]: Found 3 topics in 'special.config'
[bag2_recorder_py_node-1] [INFO] [1739450108.325205883] [rosbag2_recorder_node]: 
[bag2_recorder_py_node-1] Combined Topic Statistics:
[bag2_recorder_py_node-1] Total topics specified across all files: 4
[bag2_recorder_py_node-1] Unique topics: 4
[bag2_recorder_py_node-1] Duplicate topics: 0
[bag2_recorder_py_node-1] Topics to be recorded: 4
```
↑ Configuration file analysis results
- Number of topics in each config file
- Duplicate topic check results
- Total number of topics to be recorded

```
[bag2_recorder_py_node-1] [INFO] [1739450109.375877486] [rosbag2_recorder_node]: Started recording with combined topics from configs: standard, special
[bag2_recorder_py_node-1] [INFO] [1739450109.376203673] [rosbag2_recorder_node]: Auto-stop timer set for 30 seconds for config 'standard+special'
```
↑ Recording start notification
- Configuration file combination being used
- Auto-stop timer setting

```
[bag2_recorder_py_node-1] [INFO] [1739450139.497939370] [rosbag2_recorder_node]: Stopped recording for config 'standard+special' due to timeout (Recording duration: 30.0 seconds)
[bag2_recorder_py_node-1] [INFO] [1739450139.498490484] [rosbag2_recorder_node]: Recording finished, requesting shutdown...
[INFO] [bag2_recorder_py_node-1]: process has finished cleanly [pid 187399]
```
↑ Recording end notification
- Reason for stopping (timeout in this case)
- Actual recording duration
- Clean process termination

#### 2. Manual Stop Example
```
[bag2_recorder_py_node-1] [INFO] [1739450109.375877486] [rosbag2_recorder_node]: Started recording with combined topics from configs: standard, special
[bag2_recorder_py_node-1] [INFO] [1739450112.497939370] [rosbag2_recorder_node]: Stopped recording for config 'standard+special' due to stop topic received (Recording duration: 3.1 seconds)
[bag2_recorder_py_node-1] [INFO] [1739450112.498490484] [rosbag2_recorder_node]: Recording finished, requesting shutdown...
```
↑ Output when recording is stopped by receiving a stop topic

#### 3. Configuration File Error Examples
```
[ERROR] [bag2_recorder_py_node-1]: Configuration file 'nonexistent.config' not found in directory '/home/gemini-ninth/ros2_ws/src/bag2_recorder/config'
[ERROR] [bag2_recorder_py_node-1]: Failed to load configuration files
[bag2_recorder_py_node-1]: process has finished with exit code 1
```
↑ Error when specifying a non-existent configuration file

```
[WARNING] [bag2_recorder_py_node-1]: Duplicate topic '/camera/image_raw' found in configs: standard.config, special.config
[WARNING] [bag2_recorder_py_node-1]: Topic will only be recorded once
```
↑ Warning when the same topic is specified in multiple configuration files

#### 4. Other Error Examples
```
[ERROR] [bag2_recorder_py_node-1]: Failed to create data directory: Permission denied
[ERROR] [bag2_recorder_py_node-1]: Unable to initialize recorder
[bag2_recorder_py_node-1]: process has finished with exit code 1
```
↑ Error when lacking permissions to create data directory

```
[ERROR] [bag2_recorder_py_node-1]: Failed to open bag file: Disk space full
[ERROR] [bag2_recorder_py_node-1]: Recording failed
[bag2_recorder_py_node-1]: process has finished with exit code 1
```
↑ Error due to insufficient disk space

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
There are three ways to subscribe to all topics:

1. Add a single asterisk line to the config file:
    ```
    *
    ```

2. Leave the config file empty (or with comments only):
    ```bash
    # Comments only, treated as empty
    ```

3. Don't create a config file
    
    Note: This will generate a warning log

### Multiple Configuration Files
You can specify multiple configuration files in the launch file using the `config_files` parameter:
```python
config_files:=['standard', 'special']
```

When multiple configuration files are specified:
1. The recorder combines topics from all specified files
2. Duplicate topics are automatically removed
3. The combined configuration is named using a '+' separator (e.g., "standard+special")
4. Statistics about total, unique, and duplicate topics are displayed
5. When stopping recording:
   - Use `"data: standard+special"` to stop all recording
   - Use `"data: standard"` to stop only topics from standard.config
   - Use `"data: special"` to stop only topics from special.config

Example configuration files:
```bash
# standard.config
/topica
/topicb
/topicc

# special.config
/extra/topic1
/extra/topic2
/extra/topic3
```

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
ros2 topic pub /record/start bag2_recorder/msg/Rosbag "{config: 'standard', bag_name: 'test_bag'}" --times 5
```
Note: Using `--times 5` is recommended over `--once` as it ensures reliable message delivery.

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

### Starting the recorder programmatically (C++)
Here's an example of how to start recording using C++:

```cpp
#include <rclcpp/rclcpp.hpp>
#include "bag2_recorder/msg/rosbag.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_node");
    
    auto bag_pub = node->create_publisher<bag2_recorder::msg::Rosbag>("/record/start", 10);

    auto message = bag2_recorder::msg::Rosbag();
    message.config = "standard";
    message.bag_name = "test_bag";

    while(rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Publishing...");
        bag_pub->publish(message);
        rclpy::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
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

For multiple configuration files:
- Use `"data: standard+special"` to stop all recording
- Use `"data: standard"` to stop only topics from standard.config
- Use `"data: special"` to stop only topics from special.config

Command example:
```bash
ros2 topic pub /record/stop std_msgs/msg/String "data: standard" --times 5
```
Note: Using `--times 5` is recommended over `--once` as it ensures reliable message delivery.

### Stopping the recorder programmatically (C++)
Here's an example of how to stop recording using C++:

```cpp
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_node");
    
    auto bag_pub = node->create_publisher<std_msgs::msg::String>("/record/stop", 10);

    auto message = std_msgs::msg::String();
    message.data = "standard";

    while(rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Publishing...");
        bag_pub->publish(message);
        rclpy::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
```

### Stopping the recorder programmatically (Python)
Here's an example of how to stop recording using Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node("test_node")
    
    bag_pub = node.create_publisher(String, "/record/stop", 10)
    
    message = String()
    message.data = "standard"
    
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

## Troubleshooting

### Common Errors and Solutions

#### 1. Configuration File Issues
- **Error**: `Configuration file 'xxx.config' not found`
  - **Cause**: Specified configuration file does not exist
  - **Solution**: Verify the existence and location of the config file. By default, it should be in the package's config directory

- **Warning**: `Duplicate topic found in configs`
  - **Description**: Same topic specified in multiple configuration files
  - **Impact**: Warning only. Topic will be recorded once

#### 2. Directory Issues
- **Error**: `Failed to create data directory: Permission denied`
  - **Cause**: Lack of permissions to create data directory
  - **Solution**: Check directory permissions and grant necessary access

- **Error**: `Failed to open bag file: Disk space full`
  - **Cause**: Insufficient disk space
  - **Solution**: Free up space or specify a different disk location

#### 3. Recording Control Issues
- **Problem**: Recording won't start
  - **Check**:
    1. Correct topic type (/record/start: bag2_recorder/msg/Rosbag)
    2. Correct message content (config, bag_name fields)
    3. Using `--times 5` option

- **Problem**: Recording won't stop
  - **Check**:
    1. Correct topic type (/record/stop: std_msgs/msg/String)
    2. Correct configuration name (use '+' for combined configs)
    3. Using `--times 5` option

## TODO
- [ ] Add tests
- [ ] Allow configuration file specification from default.launch.py arguments (considering interaction with bag_name in /record/start topic)
- [ ] Automatically close active bag files when node is terminated with Ctrl + C
- [ ] Fix issue where bag name is not displayed with `ros2 topic echo /record/bag_name`
- [ ] Improve handling of multiple configuration files:
  - [ ] Add ability to stop individual configurations when using combined recording
  - [ ] Add support for "all" keyword to stop all recordings
  - [ ] Better handling of configuration combinations