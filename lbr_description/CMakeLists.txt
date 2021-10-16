cmake_minimum_required(VERSION 3.16.3)
project(lbr_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) 

add_executable(dummy_joint_states src/dummy_joint_states.cpp)
ament_target_dependencies(dummy_joint_states rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# install
install(
  DIRECTORY config gazebo launch meshes ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  dummy_joint_states
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

ament_package()
