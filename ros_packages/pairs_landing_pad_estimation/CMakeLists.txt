cmake_minimum_required(VERSION 3.20.0)
project(pairs_landing_pad_estimation)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  geometry_msgs
  pairs_lib
  pairs_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  apriltag_msgs
)

set(LIBRARIES
  PairsLandingPadEstimation_LandingPadEstimation
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# LandingPadEstimation

add_library(PairsLandingPadEstimation_LandingPadEstimation SHARED
  src/landing_pad_estimation.cpp
)

target_include_directories(PairsLandingPadEstimation_LandingPadEstimation PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(PairsLandingPadEstimation_LandingPadEstimation
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(PairsLandingPadEstimation_LandingPadEstimation PLUGIN "pairs_landing_pad_estimation::LandingPadEstimation" EXECUTABLE PairsLandingPadEstimation_LandingPadEstimation)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
