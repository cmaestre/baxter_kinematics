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
  baxter_core_msgs
  actionlib_msgs
  mocap_optitrack
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
#find_package( PkgConfig )
#pkg_check_modules( EIGEN3 REQUIRED eigen3 )
find_package(Boost REQUIRED COMPONENTS system )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

add_message_files(
  FILES
  TrajectoryTopic.msg  
)

add_service_files(
  FILES
  GetEefPose.srv
  MoveToPos.srv
  Trajectory.srv
  RestartRobot.srv
  GripperAction.srv
  GripperState.srv
)

add_action_files(
  DIRECTORY action 
  FILES moveGoal.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  mocap_optitrack
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

include_directories(include ${Eigen_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

file ( GLOB_RECURSE all_sources src/lib/* )

## Service files

add_executable(move_to_position_service src/services/move_to_position_service.cpp ${all_sources})
target_link_libraries(move_to_position_service ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(execute_trajectory_service src/services/execute_trajectory_service.cpp ${all_sources})
target_link_libraries(execute_trajectory_service ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(execute_trajectory_topic src/services/execute_trajectory_topic.cpp ${all_sources})
target_link_libraries(execute_trajectory_topic ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(restart_robot_service src/services/restart_robot_service.cpp ${all_sources})
target_link_libraries(restart_robot_service ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(gripper_action_service src/services/gripper_action_service.cpp ${all_sources})
target_link_libraries(gripper_action_service ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(get_gripper_state_service src/services/get_gripper_state_service.cpp ${all_sources})
target_link_libraries(get_gripper_state_service ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

## Test files

add_executable(test_planner src/tests/test_planner.cpp ${all_sources})
target_link_libraries(test_planner ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_service_execute_trajectory src/tests/test_service_execute_trajectory.cpp ${all_sources})
target_link_libraries(test_service_execute_trajectory ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_service_move_to_position src/tests/test_service_move_to_position.cpp ${all_sources})
target_link_libraries(test_service_move_to_position ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_service_move_to_position_up_down src/tests/test_service_move_to_position_up_down.cpp ${all_sources})
target_link_libraries(test_service_move_to_position_up_down ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_service_restart_robot src/tests/test_service_restart_robot.cpp ${all_sources})
target_link_libraries(test_service_restart_robot ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
