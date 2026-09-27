cmake_minimum_required(VERSION 3.8)
project(palletizer_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(palletizer_interfaces REQUIRED)

add_executable(orchestrator src/orchestrator.cpp)
target_link_libraries(orchestrator
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${palletizer_interfaces_TARGETS}
  behaviortree_cpp::behaviortree_cpp)

install(TARGETS orchestrator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/metrics_logger
  scripts/benchmark
  scripts/pick_place_test
  scripts/status_overlay
  scripts/sku_runs
  DESTINATION lib/${PROJECT_NAME})

ament_package()
