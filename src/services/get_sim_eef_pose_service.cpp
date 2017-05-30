#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/GetEefPose.h"

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
Kinematic_values eef_values;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
}

bool get_sim_eef_pose_callback(baxter_kinematics::GetEefPose::Request &req,
                               baxter_kinematics::GetEefPose::Response &res,
                               ros::NodeHandle& nh){
    

    ROS_INFO("Establish communication tools");
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);

    ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");    

    // Required to trigger the previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    usleep(1e6);
    std::string gripper;
    if (req.eef_name == "left")
        gripper = "left_gripper";
    else
        gripper = "right_gripper";
    res.pose = {eef_values.get_eef_position(gripper)(0), eef_values.get_eef_position(gripper)(1), eef_values.get_eef_position(gripper)(2),
                eef_values.get_eef_rpy_orientation(gripper)(0), eef_values.get_eef_rpy_orientation(gripper)(1), eef_values.get_eef_rpy_orientation(gripper)(2)};


    return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_eef_pose_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService<baxter_kinematics::GetEefPose::Request,
                                                  baxter_kinematics::GetEefPose::Response>("baxter_kinematics/get_sim_eef_pose",
                                                                                           boost::bind(get_sim_eef_pose_callback, _1, _2, n));
  ROS_INFO("Ready to get eef pose.");
  ros::spin();

  return 0;
}
