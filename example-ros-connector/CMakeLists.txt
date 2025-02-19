cmake_minimum_required(VERSION 3.5)
project(trace_example_ros_connector)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(example_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trace_executive REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time program_options serialization regex) 
# find_package(trace_ros_msgs_autocoder REQUIRED)

# Include directories
include_directories(
  include
  ${trace_executive_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/connector.cc
  # src/determine_average.cc
  # src/action_client_factory.cc 
  # src/example_action_translator.cc
  src/specific_client.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  action_msgs
  trace_executive
  example_msgs
  rclcpp_action
  std_msgs
  # trace_ros_msgs_autocoder
)

install(PROGRAMS
  publisher/incremental_publisher.cc
  DESTINATION bin/${PROJECT_NAME}
)

# Declare a C++ executable
add_executable(increment_publisher publisher/incremental_publisher.cc)
ament_target_dependencies(increment_publisher
  rclcpp
  action_msgs
  trace_executive
  std_msgs
  # trace_ros_msgs_autocoder
)
target_link_libraries(increment_publisher ${PROJECT_NAME})

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  increment_publisher
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export information to downstream packages
ament_export_include_directories(include)
ament_package()
