#include "../../include/baxter_kinematics/lib_movement.hpp"
#include <baxter_core_msgs/EndpointState.h>
#include "baxter_kinematics/MoveToPos.h"
#include "baxter_kinematics/RestartRobot.h"
#include <time.h>

Kinematic_values eef_values;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_service_restart_robot_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);
    ros::ServiceClient move_to_position = nh.serviceClient<baxter_kinematics::MoveToPos>("baxter_kinematics/move_to_position");
    ros::ServiceClient restart_robot = nh.serviceClient<baxter_kinematics::RestartRobot>("baxter_kinematics/restart_robot");

    ros::AsyncSpinner spinner (1);
    spinner.start();

    baxter_kinematics::RestartRobotRequest req;
    baxter_kinematics::RestartRobotResponse res;

    restart_robot.call(req, res);
    return 0;
}
