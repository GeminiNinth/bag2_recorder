cmake_minimum_required(VERSION 3.8)
project(bag2_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Rosbag.msg"
)

# Set Python install directory
set(PYTHON_INSTALL_DIR lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)

# Install Python modules
install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}
)

# Install Python executables
install(PROGRAMS
  scripts/bag2_recorder_py_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()