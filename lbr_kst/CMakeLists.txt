cmake_minimum_required(VERSION 3.8)
project(lbr_kst)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
  "msg/EEFCartesian.msg"
  "msg/JointPosition.msg"
  "msg/PointCloudPair.msg"
)

set(srv_files
  "srv/GetEEF.srv"
  "srv/GetJoints.srv"
  "srv/RegisterPntCloud.srv"
  "srv/SmtServoStartEEF.srv"
  "srv/SmtServoStop.srv"
  "srv/SpawnKnmtcsCalculation.srv"
)

set(action_files
  "action/NetiiwaClose.action"
  "action/PTPJointSpace.action"
  "action/PTPLineEEF.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs
)

add_executable(command_node src/command_node.cpp)
target_include_directories(command_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(command_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS command_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
