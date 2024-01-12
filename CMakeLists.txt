cmake_minimum_required(VERSION 3.8)
project(emlid_reach_ros2 VERSION 0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial CONFIG REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)

### sat_nav_lib ###

# Declare library
add_library(sat_nav_lib
  src/sat_nav.cpp
  src/nmea/conversion.cpp
)
# Specify where the library header files exist
target_include_directories(sat_nav_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# Declare ROS2 package dependencies
set(sat_nav_lib_dependencies
  nmea_msgs
  rclcpp
  geometry_msgs
  sensor_msgs
)
# Link library with ROS2 package dependencies
ament_target_dependencies(sat_nav_lib
  ${sat_nav_lib_dependencies}
)

### reach_driver_lib ###

# Declare library
add_library(reach_driver_lib
  src/reach_driver/reach_driver.cpp
  src/reach_driver/reach_serial_driver.cpp
  src/nmea/nmea_parser.cpp
  src/nmea/nmea_sentence.cpp
)
# Specify where the library header files exist
target_include_directories(reach_driver_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# Declare ROS2 package dependencies
set(reach_driver_lib_dependencies
  nmea_msgs
  sensor_msgs
  geometry_msgs
  serial
  rclcpp
)
# Link library with ROS2 package dependencies
ament_target_dependencies(reach_driver_lib
  ${reach_driver_lib_dependencies}
)

### reach_node ###

# Declare the main executable
add_executable(reach_node src/reach_node.cpp)
# Declare ROS2 package dependencies
set(reach_node_dependencies
  nmea_msgs
)
# Link library with dependencies
ament_target_dependencies(reach_node
  ${reach_node_dependencies}
)
# Link the main ROS2 package executable with libraries
target_link_libraries(reach_node
  sat_nav_lib
  reach_driver_lib
)

# Install the library and executable
install(TARGETS
    reach_node
    sat_nav_lib
    reach_driver_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${reach_node_dependencies})
ament_package()