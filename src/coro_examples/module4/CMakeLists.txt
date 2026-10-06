cmake_minimum_required(VERSION 3.8)
project(module4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lynxmotion_al5d_description REQUIRED)
find_package(coro_common REQUIRED)

set(MODULE4_DEPS rclcpp std_msgs sensor_msgs lynxmotion_al5d_description)

# Kinematics, robot configuration and simulator/robot drivers shared by both programs and the tests
add_library(lynxmotion_utilities OBJECT src/lynxmotionUtilities.cpp)
target_include_directories(lynxmotion_utilities PUBLIC include)
ament_target_dependencies(lynxmotion_utilities PUBLIC ${MODULE4_DEPS})
target_link_libraries(lynxmotion_utilities PUBLIC coro_common::coro_common)

foreach(program robotProgramming pickAndPlace)
  add_executable(${program}
    src/${program}Application.cpp
    src/${program}Implementation.cpp
    $<TARGET_OBJECTS:lynxmotion_utilities>)
  target_include_directories(${program} PRIVATE include)
  ament_target_dependencies(${program} ${MODULE4_DEPS})
  target_link_libraries(${program} coro_common::coro_common)
  install(TARGETS ${program} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_kinematics test/test_kinematics.cpp
    $<TARGET_OBJECTS:lynxmotion_utilities> ENV ROS_DOMAIN_ID=51)
  target_include_directories(test_kinematics PRIVATE include)
  ament_target_dependencies(test_kinematics ${MODULE4_DEPS})
  target_link_libraries(test_kinematics coro_common::coro_common)
  target_compile_definitions(test_kinematics PRIVATE
    MODULE4_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

  # Runs the real pickAndPlace program against the headless Gazebo simulation
  option(MODULE4_ENABLE_SIM_TESTS "Run the Gazebo-based pick-and-place test" ON)
  if(MODULE4_ENABLE_SIM_TESTS)
    add_launch_test(test/test_pick_and_place_sim.py TIMEOUT 600 ENV ROS_DOMAIN_ID=52)
  endif()
endif()

ament_package()
