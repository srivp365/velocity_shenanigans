cmake_minimum_required(VERSION 3.5)
project(safety_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# add dependencies for publisher?
add_executable(talker src/pub_trial.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/sub_trial.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
add_executable(aeb src/safety_node.cpp)
ament_target_dependencies(aeb rclcpp sensor_msgs std_msgs nav_msgs ackermann_msgs)

# install targets so ros2 can find them
install(TARGETS
  talker
  listener
  aeb
  DESTINATION lib/${PROJECT_NAME})



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
