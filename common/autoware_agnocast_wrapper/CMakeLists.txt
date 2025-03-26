cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

option(USE_AGNOCAST "Enable Agnocast integration" OFF)

set(_use_agnocast ${USE_AGNOCAST})
if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  set(_use_agnocast ON)
endif()

if(_use_agnocast)
  find_package(agnocastlib REQUIRED)
  ament_export_definitions(USE_AGNOCAST_ENABLED)

  include_directories(
    ${agnocastlib_INCLUDE_DIRS}
  )

  ament_export_include_directories(
    ${agnocastlib_INCLUDE_DIRS}
  )

  ament_export_dependencies(agnocastlib)
endif()

find_package(rclcpp REQUIRED)

include_directories(
  include
)

ament_export_include_directories(
  include
)

ament_auto_package()
