cmake_minimum_required(VERSION 3.8)
project(module5)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Curses REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(lynxmotion_al5d_description REQUIRED)
find_package(coro_common REQUIRED)

# One executable per directory under src/, named after the directory.
set(MODULE5_PROGRAMS
  binaryThresholding
  binaryThresholdingOtsu
  cameraCalibration
  cameraInvPerspectiveBinocular
  cameraInvPerspectiveMonocular
  cameraModel
  cameraModelData
  cameraModelDataSimulator
  cannyEdgeDetection
  colourSegmentation
  colourToGreyscale
  colourToHIS
  connectedComponents
  contourExtraction
  faceDetection
  featureExtraction
  gaussianFiltering
  grabCut
  imageAcquisitionFromImageFile
  imageAcquisitionFromSimulatorCamera
  imageAcquisitionFromUSBCamera
  imageAcquisitionFromVideoFile
  moveRobot
  sobelEdgeDetection
)

foreach(program ${MODULE5_PROGRAMS})
  file(GLOB program_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/${program}/*.cpp)
  add_executable(${program} ${program_sources})
  target_include_directories(${program} PRIVATE include ${CURSES_INCLUDE_DIRS})
  ament_target_dependencies(${program}
    rclcpp cv_bridge image_transport sensor_msgs std_msgs std_srvs
    ros_gz_interfaces lynxmotion_al5d_description OpenCV)
  target_link_libraries(${program} coro_common::coro_common ${CURSES_LIBRARIES})
  install(TARGETS ${program} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Lint the new code only: the vendored course data (OpenCV cascades) is not valid UTF-8 XML
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_cppcheck()
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint(${CMAKE_CURRENT_SOURCE_DIR}/package.xml)

  # Data integrity and smoke tests (programs start under a virtual display and show their banner)
  ament_add_pytest_test(test_programs test/test_programs.py TIMEOUT 600
    ENV ROS_DOMAIN_ID=53)
endif()

ament_package()
