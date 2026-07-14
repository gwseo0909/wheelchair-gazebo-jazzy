cmake_minimum_required(VERSION 3.8)
project(wheelchair_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/driving_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()