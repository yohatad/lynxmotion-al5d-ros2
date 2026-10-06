cmake_minimum_required(VERSION 3.8)
project(coro_common)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ament_index_cpp::ament_index_cpp rclcpp::rclcpp)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(std_msgs REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_package_path test/test_package_path.cpp ENV ROS_DOMAIN_ID=47)
  target_link_libraries(test_package_path ${PROJECT_NAME})

  ament_add_gtest(test_service_call test/test_service_call.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 90 ENV ROS_DOMAIN_ID=47)
  target_link_libraries(test_service_call ${PROJECT_NAME})
  ament_target_dependencies(test_service_call std_srvs)

  ament_add_gtest(test_spin test/test_spin.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 90 ENV ROS_DOMAIN_ID=47)
  target_link_libraries(test_spin ${PROJECT_NAME})
  ament_target_dependencies(test_spin std_msgs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp rclcpp)
ament_package()
