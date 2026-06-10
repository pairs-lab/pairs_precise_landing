cmake_minimum_required(VERSION 3.20.0)
project(pairs_precise_landing_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tmux    USE_SOURCE_PERMISSIONS DESTINATION share/${PROJECT_NAME})

# expose the gazebo model path to gz

ament_package()
