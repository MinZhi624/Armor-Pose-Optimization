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
find_package(Eigen3 REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(armor_interfaces REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_executable(armor_detector_node
  src/DetectorNode.cpp
  src/detector/ArmorDetector.cpp
  src/detector/Detector.cpp
  src/CameraProvider.cpp
  src/pose/PoseSolver.cpp
  src/pose/PoseProjection.cpp
  src/pose/Pose4DofCostEvaluator.cpp
  src/pose/PoseLandscapeAnalyzer.cpp
  src/pose/PoseRefineData.cpp
  src/pose/PoseRefineRunner.cpp
  src/pose/IndependentPoseRefiner.cpp
  src/pose/DualArmorBa3DofYPDRefiner.cpp
  src/pose/NoneRefiner.cpp
  src/pose/YawSearchRefiner.cpp
  src/pose/YawSearchThenDistanceRefiner.cpp
  src/pose/PoseOnlyBa6DofRefiner.cpp
  src/pose/PoseOnlyBa4DofXYZRefiner.cpp
  src/pose/PoseOnlyBa4DofYPDRefiner.cpp
  src/detector/LightDetector.cpp
  src/detector/TraditionalBackend.cpp
  src/detector/LightBarCorrector.cpp
  src/detector/ArmorCornerCorrector.cpp
  src/NumberClassifier.cpp
  src/debug/DebugGUI.cpp
  src/debug/DebugMsg.cpp
  src/debug/DebugKeyHandler.cpp
  src/debug/DebugPreprocessView.cpp
  src/debug/DebugLightView.cpp
  src/debug/DebugArmorMatchView.cpp
  src/debug/DebugClassificationView.cpp
  src/debug/DebugResultView.cpp
  src/debug/DebugTiming.cpp
  src/debug/DebugLayerController.cpp
  src/debug/DebugPoseMarkerPublisher.cpp
  src/debug/DebugYolo.cpp
  src/debug/DebugCornerCorrectionView.cpp
  src/debug/DebugPoseRefineStats.cpp
  src/debug/DebugPoseRefineCsvWriter.cpp
  src/debug/DebugPoseLandscapeCsvWriter.cpp
  src/debug/DebugPoseRefineView.cpp
  src/debug/DebugArmorYawPublisher.cpp
  src/detector/YoloBackend.cpp
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
  rosbag2_interfaces
  visualization_msgs
  armor_interfaces
)

target_include_directories(armor_detector_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(armor_detector_node ${OpenCV_LIBS} yaml-cpp Eigen3::Eigen Ceres::ceres openvino::runtime)

install(TARGETS armor_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  model
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gtest(pose_landscape_test
    test/PoseLandscapeTest.cpp
    src/pose/Pose4DofCostEvaluator.cpp
    src/pose/PoseLandscapeAnalyzer.cpp
    src/pose/PoseProjection.cpp
    src/pose/PoseRefineData.cpp
    src/pose/PoseRefineRunner.cpp
    src/pose/IndependentPoseRefiner.cpp
    src/pose/DualArmorBa3DofYPDRefiner.cpp
    src/pose/NoneRefiner.cpp
    src/pose/PoseOnlyBa6DofRefiner.cpp
    src/pose/PoseOnlyBa4DofXYZRefiner.cpp
    src/pose/PoseOnlyBa4DofYPDRefiner.cpp
    src/pose/YawSearchRefiner.cpp
    src/pose/YawSearchThenDistanceRefiner.cpp
  )
  if(TARGET pose_landscape_test)
    ament_target_dependencies(pose_landscape_test rclcpp)
    target_include_directories(pose_landscape_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(pose_landscape_test ${OpenCV_LIBS} Eigen3::Eigen Ceres::ceres)
  endif()

  ament_add_gtest(dual_armor_ba_3dof_ypd_test
    test/DualArmorBa3DofYPDTest.cpp
    src/pose/Pose4DofCostEvaluator.cpp
    src/pose/PoseProjection.cpp
    src/pose/PoseRefineData.cpp
    src/pose/PoseRefineRunner.cpp
    src/pose/IndependentPoseRefiner.cpp
    src/pose/DualArmorBa3DofYPDRefiner.cpp
    src/pose/NoneRefiner.cpp
    src/pose/PoseOnlyBa6DofRefiner.cpp
    src/pose/PoseOnlyBa4DofXYZRefiner.cpp
    src/pose/PoseOnlyBa4DofYPDRefiner.cpp
    src/pose/YawSearchRefiner.cpp
    src/pose/YawSearchThenDistanceRefiner.cpp
  )
  if(TARGET dual_armor_ba_3dof_ypd_test)
    ament_target_dependencies(dual_armor_ba_3dof_ypd_test rclcpp)
    target_include_directories(dual_armor_ba_3dof_ypd_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(dual_armor_ba_3dof_ypd_test ${OpenCV_LIBS} Eigen3::Eigen Ceres::ceres)
  endif()

  ament_lint_auto_find_test_dependencies()
  set(armor_detector_lint_hook_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  ament_register_extension("ament_lint_auto" "armor_detector_lint_hook" "disable_uncrustify.cmake")
endif()

ament_package()
