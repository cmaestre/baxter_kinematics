#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/GripperState.h"
#include "baxter_core_msgs/EndEffectorState.h"

void leftGripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg,
                         Kinematic_values& eef_values)
{
  eef_values.set_gripper_openness("left", msg->position);
}

void rightGripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg,
                          Kinematic_values& eef_values)
{
  eef_values.set_gripper_openness("right", msg->position);
}

bool get_gripper_status(baxter_kinematics::GripperState::Request &req,
                        baxter_kinematics::GripperState::Response &res,
                        Kinematic_values& eef_values){

    // Select arm and create publisher
    if(strcmp(req.eef_name.c_str(), "left") == 0)
        res.openness = eef_values.get_gripper_openness("left");
    else if(strcmp(req.eef_name.c_str(), "right") == 0)
        res.openness = eef_values.get_gripper_openness("right");
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
  Kinematic_values eef_values;

  ros::Subscriber left_gripper_listener = nh.subscribe<baxter_core_msgs::EndEffectorState>("/robot/end_effector/left_gripper/state", 1, boost::bind(leftGripperCallback, _1,
                                                                                                                  boost::ref(eef_values)));
  ros::Subscriber right_gripper_listener = nh.subscribe<baxter_core_msgs::EndEffectorState>("/robot/end_effector/right_gripper/state", 1, boost::bind(rightGripperCallback, _1,
                                                                                                                  boost::ref(eef_values)));

  ros::ServiceServer service = nh.advertiseService<baxter_kinematics::GripperState::Request,
          baxter_kinematics::GripperState::Response>("baxter_kinematics/get_gripper_openness", boost::bind(get_gripper_status, _1, _2,
                                                                                                           boost::ref(eef_values)));
  ROS_INFO("Ready to move to a position.");
  ros::spin();

  return 1;
}

