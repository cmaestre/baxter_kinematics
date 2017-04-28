cmake_minimum_required(VERSION 2.8.3)
project(baxter_kinematics)


find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

#find_package(Eigen3 REQUIRED)
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
find_package(Boost REQUIRED COMPONENTS system )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

add_service_files(
  FILES
  GetEefPose.srv
  MoveToPos.srv
  Trajectory.srv
  RestartRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
#  LIBRARIES crustcrawler_cppkdl
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

file ( GLOB_RECURSE all_sources src/lib/* )

## Service files

add_executable(move_to_position_service src/services/move_to_position_service.cpp ${all_sources})
target_link_libraries(move_to_position_service ${catkin_LIBRARIES})

add_executable(execute_trajectory_service src/services/execute_trajectory_service.cpp ${all_sources})
target_link_libraries(execute_trajectory_service ${catkin_LIBRARIES})

add_executable(get_sim_eef_pose_service src/services/get_sim_eef_pose_service.cpp ${all_sources})
target_link_libraries(get_sim_eef_pose_service ${catkin_LIBRARIES})

add_executable(restart_robot_service src/services/restart_robot_service.cpp ${all_sources})
target_link_libraries(restart_robot_service ${catkin_LIBRARIES})

## Test files


