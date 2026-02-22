cmake_minimum_required(VERSION 3.5)
project(theo_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(theo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED) # NO VERSIONING ATM
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

# Continued


# Include Cpp "include" directory
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Create Cpp executable
add_executable(mixer_node src/mixer.cpp)
ament_target_dependencies(mixer_node rclcpp Eigen3 theo_msgs geometry_msgs)

add_executable(sabertooth_node src/sabertooth.cpp)
ament_target_dependencies(sabertooth_node rclcpp Boost theo_msgs)


# Install Cpp executables
install(TARGETS
  mixer_node
  sabertooth_node
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

ament_package()
