cmake_minimum_required(VERSION 3.14)
project(task_planner)


find_package(autoware_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

autoware_package()

include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_executable(${PROJECT_NAME}
  src/utils/utils.cpp
  src/task_planner.cpp
  src/task_planner_node.cpp
)
ament_target_dependencies(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

# ★ 多目标车调度器
ament_auto_add_executable(charge_scheduler
  src/charge_scheduler.cpp
)
ament_target_dependencies(charge_scheduler)
target_link_libraries(charge_scheduler
  Eigen3::Eigen
)
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  scripts
)
