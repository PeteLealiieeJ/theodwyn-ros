cmake_minimum_required(VERSION 3.8)
project(theo_comm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(theo_msgs REQUIRED)
find_package(theo_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED) # NO VERSIONING ATM

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

include_directories(include)

# Create Cpp executable
add_executable(broker_node src/broker.cpp)
ament_target_dependencies(broker_node rclcpp theo_msgs theo_srvs)

add_executable(transmitter_node src/trajectory_transmitter.cpp)
ament_target_dependencies(transmitter_node rclcpp Boost theo_msgs theo_srvs )

# Install Cpp executables
install(TARGETS
  broker_node
  transmitter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package( ${PROJECT_NAME} )

# # Install Python executables
# install(PROGRAMS
#   scripts/pantilt_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch and config directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} )
ament_export_include_directories(include)

ament_package()
