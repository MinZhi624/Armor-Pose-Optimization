cmake_minimum_required(VERSION 3.8)
project(armor_detector)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(armor_detector_node
  src/detector_node.cpp
  src/detector/armor_detector.cpp
  src/detector/detector.cpp
  src/camera_provider.cpp
  src/pose_solver.cpp
  src/detector/light_detector.cpp
  src/debug/debug_gui.cpp
  src/debug/debug_msg.cpp
)

target_include_directories(armor_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(armor_detector_node
  rclcpp
  ament_index_cpp
  cv_bridge
  sensor_msgs
)

target_link_libraries(armor_detector_node ${OpenCV_LIBS} yaml-cpp)

install(TARGETS armor_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
