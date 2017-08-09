#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/RestartRobot.h"
#include "baxter_kinematics/GripperAction.h"

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr&  l_eef_feedback,
                       Kinematic_values& eef_values){
    locate_eef_pose(l_eef_feedback->pose, eef_values, "left_gripper");
}

//call back that register baxter right end effector pose and rearrange the orientation in RPY
void right_eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr&  r_eef_feedback,
                        Kinematic_values& eef_values){
    locate_eef_pose(r_eef_feedback->pose, eef_values, "right_gripper");
}

int restart_robot(baxter_kinematics::RestartRobot::Request &req,
                  baxter_kinematics::RestartRobot::Response &res,
                  ros::NodeHandle& nh,
                  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right,
                  ros::ServiceClient& client_gripper,
                  Kinematic_values& eef_values){

    ROS_INFO("Establish communication tools");
    // Required for communication with moveit components
    ros::AsyncSpinner spinner (1);
    spinner.start();

    // Set gripper by default action
    baxter_kinematics::GripperAction srv;
    std::string current_action;
    nh.getParam("gripper_state/left", current_action);
    srv.request.eef_name = "left";
    srv.request.action = current_action;
    if (!client_gripper.call(srv))
        ROS_ERROR_STREAM("restart_robot - Failed to execute action " << current_action << " on left gripper");

    nh.getParam("gripper_state/right", current_action);
    srv.request.eef_name = "right";
    srv.request.action = current_action;
    if (!client_gripper.call(srv))
        ROS_ERROR_STREAM("restart_robot - Failed to execute action " << current_action << " on right gripper");

    bool mov_res = restart_robot_initial_position(eef_values,
                                                  ac_left,
                                                  ac_right,
                                                  nh);
    if (!mov_res){
        ROS_ERROR_STREAM("restart_robot : Problem restarting robot");
        res.success = false;
        return 0;
    }

    res.success = true;
    spinner.stop();
    ROS_INFO("Done!");
    return 1;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "restart_world_node");
    ros::NodeHandle nh;
    Kinematic_values eef_values_;

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10,
                                                                                  boost::bind(left_eef_Callback, _1, boost::ref(eef_values_)));
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10,
                                                                                  boost::bind(right_eef_Callback, _1, boost::ref(eef_values_)));
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_left("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_right("/robot/limb/right/follow_joint_trajectory", true);
    ros::ServiceClient client_gripper = nh.serviceClient<baxter_kinematics::GripperAction>("/baxter_kinematics/gripper_action");

    ros::ServiceServer service = nh.advertiseService<baxter_kinematics::RestartRobot::Request,
            baxter_kinematics::RestartRobot::Response>("baxter_kinematics/restart_robot", boost::bind(restart_robot, _1, _2, nh,
                                                                                                      boost::ref(ac_left),
                                                                                                      boost::ref(ac_right),
                                                                                                      boost::ref(client_gripper),
                                                                                                      boost::ref(eef_values_)));
    ROS_INFO("Ready to restart robot.");
    ros::spin();

    return 0;
}
