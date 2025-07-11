cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_lane_change_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/manager.cpp
  src/scene.cpp
  src/utils/calculation.cpp
  src/utils/markers.cpp
  src/utils/utils.cpp
  src/utils/path.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_behavior_path_planner_node_interface.cpp
    test/test_lane_change_utils.cpp
    test/test_lane_change_scene.cpp
    test/test_planning_factor.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  test_data
)
