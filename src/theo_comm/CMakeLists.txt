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

# add shared libary
add_library(theo_comm_lib SHARED 
    src/broker_client_node.cpp 
)
set_target_properties(
  theo_comm_lib
  PROPERTIES VERSION "${theo_comm_VERSION}"
)
target_link_libraries(
  theo_comm_lib 
  rclcpp::rclcpp
  ${theo_msgs_TARGETS}
  ${theo_srvs_TARGETS}
)
install(
  TARGETS theo_comm_lib
  EXPORT export_theo_comm_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/
)
ament_export_targets(export_theo_comm_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp theo_msgs theo_srvs)

# Create Cpp executable
add_executable(broker_node src/broker.cpp)
ament_target_dependencies(broker_node rclcpp theo_msgs theo_srvs)

add_executable(transmitter_node src/trajectory_transmitter.cpp)
target_link_libraries(transmitter_node 
  theo_comm_lib
  Boost::boost 
)

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
