cmake_minimum_required(VERSION 3.8)
project(param_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# CPP
# Add the executable
ament_auto_add_executable(cpp_param_node 
  src/cpp_param_node.cpp
  src/classes/cpp_param_class.cpp # dependencies
  )
# Install the executable
install(TARGETS cpp_param_node
  DESTINATION lib/${PROJECT_NAME}
)
# Add the executable
ament_auto_add_executable(cpp_complex_param_node 
src/cpp_complex_param_node.cpp
src/classes/cpp_complex_param_class.cpp # dependencies
)
# Install the executable
install(TARGETS cpp_complex_param_node
  DESTINATION lib/${PROJECT_NAME}
)

# PYTHON
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/python_param_node.py
  scripts/python_complex_param_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)