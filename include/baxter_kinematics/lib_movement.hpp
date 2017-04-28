#ifndef __LIB_MOVEMENT_HPP__
#define __LIB_MOVEMENT_HPP__

#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <boost/timer.hpp>
#include <tf/tf.h>
#include <exception>
#include <tf/tf.h>
#include <fstream>
#include <cmath>
#include <tuple>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/exceptions/exceptions.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit/robot_state/conversions.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

#include "../../src/lib/eef_values.hpp"

void jocommCallback_sim(const sensor_msgs::JointState::ConstPtr &jo_state,
                         std::vector<double> &left_arm_joint_values,
                         std::vector<double> &right_arm_joint_values);

void jocommCallback_real(const sensor_msgs::JointState::ConstPtr& jo_state,
                         std::vector<double> &left_arm_joint_values,
                         std::vector<double> &right_arm_joint_values);

//return roll pitch yaw from the transformation matrix transform_l_ee_w
Eigen::Vector3d extract_angles(Eigen::Matrix4d& transform_l_ee_w);

/* */
int find_nearest(std::vector<Eigen::Vector3d>& my_points,
                 Eigen::Vector3d& my_goal);

/*get baxter left eef pose with orientation expressed as RPY
 * input: a baxter core msg that holds left eef status (including the pose as geometry msgs), and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose& l_eef_feedback, Kinematic_values &eef_values, const std::string gripper = "left_gripper");

/**
 * @brief return eef position and orientation
 * @param
 * @return
 */
std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_eef_pose(
        robot_state::RobotState my_robot_state,
        std::string arm_name,
        std::vector<double> arm_joint_values);

/** restart arms to initial safe position (before any movement)
 * @brief a
 * @param b
**/
bool restart_robot_initial_position(moveit::core::RobotState robot_state,
                                    Kinematic_values &eef_values,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac_left,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac_right,
                                    ros::NodeHandle nh,
                                    std::vector<double> &left_arm_joint_values,
                                    std::vector<double> &right_arm_joint_values,
                                    std::vector<double> &all_joint_values);

/**
 * @brief execute the descartes motion to a desired final position
 * @param desired position and ros handle
 * @param argv
 * @return
 */
bool plan_path_to_desired_position(moveit::core::RobotState robot_state, Eigen::Vector3d goal,
                                   std::string my_side,
                                   std::vector<Eigen::Vector3d> mid_point_position_vector,
                                   std::vector<Eigen::Vector3d> &path,
                                   std::vector<Eigen::Vector3d> &final_path,
                                   Kinematic_values &eef_values);

/**
 * @brief find the largest difference between elements of two std::vectors of doubles
 * @param first std::vector
 * @param second std::vector
 * @return double (the largest difference)
 */
double largest_difference(std::vector<double> &first, std::vector<double> &second);

/**
 * @brief a
 * @param b
**/
//trajectory_msgs::JointTrajectory path_to_safe_pos(Eigen::Vector3D safe_pose,
void path_to_safe_pos(trajectory_msgs::JointTrajectory &trajectory,
                      ros::ServiceClient baxter_right_ik_solver,
                      Eigen::Vector3d safe_pose,
                      Eigen::Vector3d rotation,
                      std::vector<double> right_arm_joint_values);

/**
 * @brief Given a directed trajectory, compute the related waypoints
 * @param b
**/
std::vector<geometry_msgs::Pose> compute_directed_waypoints(bool initialize_setup, moveit::core::RobotState robot_state,
                                                            Eigen::Vector3d goal_position,
                                                            std::string cube_side,
                                                            std::vector<Eigen::Vector3d> predefined_wp_trajectory,
                                                            Kinematic_values eef_values);

/**
 * @brief Plan and execute waypoint trajectory
 * @param b
**/
//trajectory_msgs::JointTrajectory plan_trajectory(
int plan_and_execute_waypoint_traj(std::string selected_eef,
                                    std::vector<geometry_msgs::Pose> waypoints,
                                    robot_state::RobotState robot_state,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                                    std::string object_name,
                                    std::vector<Eigen::Vector3d>& eef_position_vector,
                                    std::vector<Eigen::Vector3d>& eef_orientation_vector,
                                    std::vector<Eigen::Vector3d>& object_position_vector,
                                    std::vector<Eigen::Vector3d>& object_orientation_vector,
                                    Kinematic_values &eef_values,
                                    ros::NodeHandle nh,
                                    bool feedback_data = false,
                                    bool publish_topic = false,
                                    int feedback_frequency = 1,
                                    ros::Publisher traj_res_pub = ros::Publisher());
/**
 * @brief This method should help retrun the cube to a reachable position for robot arm
 *
 * @return
 */
bool move_object_close_to_robot(std::string selected_eef,
                                std::vector<geometry_msgs::Pose> waypoints,
                                moveit::core::RobotState robot_state,
                                Eigen::Vector3d goal_position,
                                std::string cube_side,
                                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                                Kinematic_values &eef_values);
/**
 * @brief a
 * @param b
**/
//trajectory_msgs::JointTrajectory plan_trajectory(Eigen::Vector3d position,
bool plan_traj_to_goal_joint_config(Kinematic_values eef_values,
                                    bool initialize_setup,
                                    ros::ServiceClient &ik_solver,
                                    robot_state::RobotState robot_state,
                                    trajectory_msgs::JointTrajectory &trajectory,
                                    Eigen::Vector3d position,
                                    std::string cube_side,
                                    std::vector<Eigen::Vector3d> mid_point_position_vector,
                                    ros::NodeHandle nh,
                                    bool try_different_seed = false);

/** execute a trajectory (nothing returned)
 * @brief a
 * @param b
**/
void execute_joint_config_traj(ros::Publisher &pub_msg,
                               trajectory_msgs::JointTrajectory &trajectory);

///**
// * @brief execute a trajectory (return eef trajectory position and orientation)
// * @param b
//**/
//bool execute_trajectory_feedback(
//        robot_state::RobotState my_robot_state,
//        ros::Publisher &pub_msg,
//        ros::ServiceClient &gazebo_model_state,
//        trajectory_msgs::JointTrajectory &trajectory,
//        std::vector<Eigen::Vector3d>& eef_position_vector,
//        std::vector<Eigen::Vector3d>& eef_orientation_vector,
//        std::vector<Eigen::Vector3d>& object_position_vector,
//        std::vector<Eigen::Vector3d>& object_orientation_vector);

///**
// * @brief execute a trajectory via joint action server
// * @param the action server and the trajectory msgs
//**/
//std::vector<int> execute_trajectory_action_server(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac,
//                                      moveit_msgs::RobotTrajectory &moveit_trajectory,
//                                      control_msgs::FollowJointTrajectoryActionResult &action_result,
//                                      control_msgs::FollowJointTrajectoryActionFeedback &action_feedback,
//                                      robot_state::RobotState &my_robot_state,
//                                      std::string limb,
//                                      ros::ServiceClient &gazebo_model_state,
//                                      ros::ServiceClient &gazebo_link_state,
//                                      std::string link_name,
//                                      std::vector<Eigen::Vector3d>& eef_position_vector,
//                                      std::vector<Eigen::Vector3d>& eef_orientation_vector,
//                                      std::vector<Eigen::Vector3d>& object_position_vector,
//                                      std::vector<Eigen::Vector3d>& object_orientation_vector,
//                                      bool feedback);

///**
// * @brief smooth a joint trajectory by respecting velocity and acceleration limits
// * @param joint trajectory
// * @return boolean
//**/
//bool smooth_joint_trajectory(trajectory_msgs::JointTrajectory& my_traj,
//                             robot_model::RobotModelPtr &robot_model,
//                             std::string limb,
//                             std::vector<double> &joints_values,
//                             moveit_msgs::RobotTrajectory &moveit_trajectory);

#endif /* __LIB_MOVEMENT_HPP__ */
