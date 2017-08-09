#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/GetEefPose.h"

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/

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

bool get_sim_eef_pose_callback(baxter_kinematics::GetEefPose::Request &req,
                               baxter_kinematics::GetEefPose::Response &res,
                               ros::NodeHandle& nh,
                               Kinematic_values& eef_values){
    

    ROS_INFO("Establish communication tools");
//    ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

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
  Kinematic_values eef_values;

  ros::Subscriber sub_l_eef_msg = n.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10,
                                                                                boost::bind(left_eef_Callback, _1, boost::ref(eef_values)));
  ros::Subscriber sub_r_eef_msg = n.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10,
                                                                                boost::bind(right_eef_Callback, _1, boost::ref(eef_values)));

  ros::ServiceServer service = n.advertiseService<baxter_kinematics::GetEefPose::Request,
                                                  baxter_kinematics::GetEefPose::Response>("baxter_kinematics/get_sim_eef_pose",
                                                                                           boost::bind(get_sim_eef_pose_callback, _1, _2, n,
                                                                                                       boost::ref(eef_values)));
  ROS_INFO("Ready to get eef pose.");
  ros::spin();

  return 0;
}
