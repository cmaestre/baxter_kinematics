#ifndef __LIB_MOVEMENT_HPP__
#define __LIB_MOVEMENT_HPP__

#include "../../src/lib/eef_values.hpp"

/* */
int find_nearest(std::vector<Eigen::Vector3d>& my_points,
                 Eigen::Vector3d& my_goal);

int find_farthest(std::vector<Eigen::Vector3d>& my_points,
                  Eigen::Vector3d& my_goal);

/*get baxter left eef pose with orientation expressed as RPY
 * input: a baxter core msg that holds left eef status (including the pose as geometry msgs), and the Data_config class
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose l_eef_feedback, Kinematic_values &eef_values, const std::string gripper = "left_gripper");


/** restart arms to initial safe position (before any movement)
 * @brief a
 * @param b
**/
bool restart_robot_initial_position(Kinematic_values &eef_values,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac_left,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac_right,
                                    ros::NodeHandle nh);

/**
 * @brief execute the descartes motion to a desired final position
 * @param desired position and ros handle
 * @param argv
 * @return
 */
bool plan_path_to_desired_position(Eigen::Vector3d goal,
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
 * @brief Given a directed trajectory, compute the related waypoints
 * @param b
**/
std::vector<geometry_msgs::Pose> compute_directed_waypoints(bool initialize_setup,
                                                            Eigen::Vector3d goal_position,
                                                            std::string cube_side,
                                                            std::vector<Eigen::Vector3d> predefined_wp_trajectory,
                                                            Kinematic_values eef_values);

//get rid of repeqted waypoints in a trajectory
bool optimize_trajectory(std::vector<geometry_msgs::Pose>& vector_to_optimize,
                         double min_wp_dist);

/**
 * @brief Plan and execute to go to initial position
 * @param b
**/
void goto_initial_position(std::string selected_eef,
                           geometry_msgs::Pose& pose,
                           Kinematic_values& eef_values);

/**
 * @brief Plan and execute waypoint trajectory
 * @param b
**/
//trajectory_msgs::JointTrajectory plan_trajectory(
int plan_and_execute_waypoint_traj(std::string selected_eef,
                                    std::vector<geometry_msgs::Pose> waypoints,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                                    Kinematic_values &eef_values,
                                    ros::NodeHandle nh,
                                    bool force_orien = false,
                                    bool feedback_data = false,
                                    ros::Publisher traj_res_pub = ros::Publisher());


/** execute a trajectory (nothing returned)
 * @brief a
 * @param b
**/
void execute_joint_config_traj(ros::Publisher &pub_msg,
                               trajectory_msgs::JointTrajectory &trajectory);
#endif /* __LIB_MOVEMENT_HPP__ */
