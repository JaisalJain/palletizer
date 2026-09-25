cmake_minimum_required(VERSION 3.8)
project(palletizer_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
# Use the ROS-vendored Gazebo (the same build gz_ros2_control and ros_gz_sim use),
# never the /usr gz-harmonic install.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

add_library(palletizer_suction_gripper SHARED src/SuctionGripper.cc)
target_link_libraries(palletizer_suction_gripper
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport)

add_library(palletizer_model_pose_publisher SHARED src/ModelPosePublisher.cc)
target_link_libraries(palletizer_model_pose_publisher
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport)

install(TARGETS palletizer_suction_gripper palletizer_model_pose_publisher
  LIBRARY DESTINATION lib)
install(DIRECTORY config launch rviz worlds DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
