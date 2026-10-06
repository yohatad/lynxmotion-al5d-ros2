cmake_minimum_required(VERSION 3.8)
project(module3)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(coro_common REQUIRED)

set(MODULE3_DEPS rclcpp geometry_msgs nav_msgs std_srvs turtlesim)

# The implementation files are compiled once and shared by the programs and the unit tests.
add_library(goToPosition_impl OBJECT src/goToPositionImplementation.cpp)
target_include_directories(goToPosition_impl PUBLIC include)
ament_target_dependencies(goToPosition_impl PUBLIC ${MODULE3_DEPS})
target_link_libraries(goToPosition_impl PUBLIC coro_common::coro_common)

add_library(goToPoseCreate_impl OBJECT src/goToPoseCreateImplementation.cpp)
target_include_directories(goToPoseCreate_impl PUBLIC include)
ament_target_dependencies(goToPoseCreate_impl PUBLIC ${MODULE3_DEPS})
target_link_libraries(goToPoseCreate_impl PUBLIC coro_common::coro_common)

add_executable(goToPosition src/goToPositionApplication.cpp $<TARGET_OBJECTS:goToPosition_impl>)
target_include_directories(goToPosition PRIVATE include)
ament_target_dependencies(goToPosition ${MODULE3_DEPS})
target_link_libraries(goToPosition coro_common::coro_common)

add_executable(goToPoseCreate src/goToPoseCreateApplication.cpp
  $<TARGET_OBJECTS:goToPoseCreate_impl>)
target_include_directories(goToPoseCreate PRIVATE include)
ament_target_dependencies(goToPoseCreate ${MODULE3_DEPS})
target_link_libraries(goToPoseCreate coro_common::coro_common)

install(TARGETS goToPosition goToPoseCreate DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_locomotion_parameters test/test_locomotion_parameters.cpp
    ENV ROS_DOMAIN_ID=48)
  target_sources(test_locomotion_parameters PRIVATE $<TARGET_OBJECTS:goToPoseCreate_impl>)
  target_include_directories(test_locomotion_parameters PRIVATE include)
  ament_target_dependencies(test_locomotion_parameters ${MODULE3_DEPS})
  target_link_libraries(test_locomotion_parameters coro_common::coro_common)
  target_compile_definitions(test_locomotion_parameters PRIVATE
    MODULE3_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

  add_launch_test(test/test_go_to_position.py TIMEOUT 240 ENV ROS_DOMAIN_ID=49)
  ament_add_pytest_test(test_go_to_pose_create test/test_go_to_pose_create.py
    TIMEOUT 300 ENV ROS_DOMAIN_ID=50)
endif()

ament_package()
