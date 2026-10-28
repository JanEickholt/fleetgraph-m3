cmake_minimum_required(VERSION 3.8)
project(fleetgraph_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds maps config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
