#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/GripperState.h"
#include "baxter_core_msgs/EndEffectorState.h"

Kinematic_values eef_values;

void leftGripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg)
{
  eef_values.set_left_gripper_openness(msg->position);
}

void rightGripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg)
{
  eef_values.set_right_gripper_openness(msg->position);
}

bool get_gripper_status(baxter_kinematics::GripperState::Request &req,
                        baxter_kinematics::GripperState::Response &res){

    // Select arm and create publisher
    if(strcmp(req.eef_name.c_str(), "left") == 0)
        res.openness = eef_values.get_left_gripper_openness();
    else if(strcmp(req.eef_name.c_str(), "right") == 0)
        res.openness = eef_values.get_right_gripper_openness();
    else{
        ROS_ERROR("gripper_action - please specify in service request, left or right arm");
        return false;
    }

    ROS_INFO("Done!");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_gripper_state_node");
  ros::NodeHandle nh;

  ros::Subscriber left_gripper_listener = nh.subscribe("/robot/end_effector/left_gripper/state", 1, leftGripperCallback);
  ros::Subscriber right_gripper_listener = nh.subscribe("/robot/end_effector/right_gripper/state", 1, rightGripperCallback);

  ros::ServiceServer service = nh.advertiseService<baxter_kinematics::GripperState::Request,
          baxter_kinematics::GripperState::Response>("baxter_kinematics/get_gripper_openness", boost::bind(get_gripper_status, _1, _2));
  ROS_INFO("Ready to move to a position.");
  ros::spin();

  return 1;
}

