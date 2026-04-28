cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  "srv/GetLastPositions.srv"
  "action/Navigation.action"
)

# Export dependencies for other packages to use
ament_export_dependencies(rosidl_default_runtime)

ament_package()
