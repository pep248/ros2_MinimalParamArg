cmake_minimum_required(VERSION 3.8)
project(generate_parameter_library_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# C++
# Config 1 parameters
generate_parameter_library(cpp_config_1_parameters # cmake target name for the parameter library
  config/config_1_parameters.yaml # path to input yaml file
)

# Config 2 parameters
generate_parameter_library(cpp_config_2_parameters # cmake target name for the parameter library
  config/config_2_parameters.yaml # path to input yaml file
)

# Generate Parameter Library
ament_auto_add_executable(cpp_generate_parameter_library_node
  src/cpp_generate_parameter_library_node.cpp
  src/classes/cpp_generate_parameter_library_class.cpp
)
target_link_libraries(cpp_generate_parameter_library_node
  cpp_config_1_parameters
  cpp_config_2_parameters
)
target_include_directories(cpp_generate_parameter_library_node
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/cpp_config_1_parameters/include
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/cpp_config_2_parameters/include
)
install(TARGETS cpp_generate_parameter_library_node
  DESTINATION lib/${PROJECT_NAME}
)


# Python
# Config 1 parameters
generate_parameter_module(python_config_1_parameters # cmake target name for the parameter library
  config/config_1_parameters.yaml # path to input yaml file
)

# Config 2 parameters
generate_parameter_module(python_config_2_parameters # cmake target name for the parameter library
  config/config_2_parameters.yaml # path to input yaml file
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/python_generate_parameter_library_node.py
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