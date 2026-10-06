cmake_minimum_required(VERSION 3.8)
project(lynxmotion_al5d_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RPY.msg"
  "msg/Pose.msg"
  "srv/SpawnBrick.srv"
  "srv/KillBrick.srv"
  "srv/TeleportAbsolute.srv"
  "srv/TeleportRelative.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- core library (pure logic, unit-testable without a simulator) ------------------
add_library(al5d_core SHARED
  src/pose_utils.cpp
  src/brick.cpp
  src/manager.cpp
  src/gz_backend.cpp
  src/cli_args.cpp
)
target_include_directories(al5d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(al5d_core PUBLIC "${typesupport_target}")
ament_target_dependencies(al5d_core PUBLIC
  rclcpp geometry_msgs std_msgs std_srvs tf2 tf2_msgs ros_gz_interfaces ament_index_cpp)

add_executable(manager_node src/manager_node.cpp)
target_link_libraries(manager_node al5d_core)
add_executable(spawn_brick src/spawn_brick.cpp)
target_link_libraries(spawn_brick al5d_core)
add_executable(kill_brick src/kill_brick.cpp)
target_link_libraries(kill_brick al5d_core)
add_library(al5d_gz_conversion SHARED src/gz_pose_conversion.cpp)
target_include_directories(al5d_gz_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(al5d_gz_conversion PUBLIC gz-msgs::core)
ament_target_dependencies(al5d_gz_conversion PUBLIC tf2_msgs)

add_executable(gz_pose_relay src/gz_pose_relay.cpp)
target_link_libraries(gz_pose_relay al5d_gz_conversion gz-transport::core)
ament_target_dependencies(gz_pose_relay rclcpp tf2_msgs)

add_executable(slider_command src/slider_command.cpp)
ament_target_dependencies(slider_command rclcpp std_msgs sensor_msgs)
target_include_directories(slider_command PRIVATE include)

add_executable(command_relay src/command_relay.cpp)
ament_target_dependencies(command_relay rclcpp std_msgs)
target_include_directories(command_relay PRIVATE include)

# al5d_core and al5d_gz_conversion are internal libraries (not exported to other packages)
install(TARGETS al5d_core al5d_gz_conversion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS manager_node spawn_brick kill_brick command_relay gz_pose_relay slider_command
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch meshes models urdf worlds rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Generic lint checks (xmllint, flake8 ...). Style checkers that enforce a specific
  # house style (copyright header, cpplint, uncrustify) are not enabled.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp ENV ROS_DOMAIN_ID=41)
  target_link_libraries(test_pose_utils al5d_core)

  ament_add_gtest(test_cli_args test/test_cli_args.cpp ENV ROS_DOMAIN_ID=41)
  target_link_libraries(test_cli_args al5d_core)

  ament_add_gtest(test_gripper_mapping test/test_gripper_mapping.cpp ENV ROS_DOMAIN_ID=41)
  target_include_directories(test_gripper_mapping PRIVATE include)

  ament_add_gtest(test_gz_pose_conversion test/test_gz_pose_conversion.cpp ENV ROS_DOMAIN_ID=41)
  target_link_libraries(test_gz_pose_conversion al5d_gz_conversion)

  ament_add_gtest(test_slider_mapping test/test_slider_mapping.cpp ENV ROS_DOMAIN_ID=41)
  target_include_directories(test_slider_mapping PRIVATE include)

  ament_add_gtest(test_manager test/test_manager.cpp TIMEOUT 120 ENV ROS_DOMAIN_ID=42)
  target_link_libraries(test_manager al5d_core)

  # Static description tests: xacro expands, URDF parses, joints/limits are sane
  ament_add_pytest_test(test_description test/test_description.py TIMEOUT 120)

  # Manager node against a fake simulator (no Gazebo needed)
  add_launch_test(test/test_manager_fake_sim.py TIMEOUT 180 ENV ROS_DOMAIN_ID=43)

  # Full simulation (headless Gazebo). Needs software rendering and ~2 GB RAM.
  option(AL5D_ENABLE_SIM_TESTS "Run Gazebo-based integration tests" ON)
  if(AL5D_ENABLE_SIM_TESTS)
    add_launch_test(test/test_sim_integration.py TIMEOUT 420 ENV ROS_DOMAIN_ID=44)
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()
