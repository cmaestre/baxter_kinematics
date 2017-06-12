#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/GripperAction.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

bool gripper_action(baxter_kinematics::GripperAction::Request &req,
                      baxter_kinematics::GripperAction::Response &res,
                      ros::NodeHandle& nh,
                      ros::Publisher& left_gripper_pub,
                      ros::Publisher& right_gripper_pub){

    ROS_INFO("Establish communication tools");
    ros::AsyncSpinner spinner (1);
    spinner.start();

    // Select arm and create publisher
    ros::Publisher current_pub;
    if(strcmp(req.eef_name.c_str(), "left") == 0)
        current_pub = left_gripper_pub;
    else if(strcmp(req.eef_name.c_str(), "right") == 0)
        current_pub = right_gripper_pub;
    else{
        ROS_ERROR("gripper_action - please specify in service request, left or right arm");
        return false;
    }

    // Select action and execute it
    baxter_core_msgs::EndEffectorCommand gripperMsg;
    if(strcmp(req.action.c_str(), "open") == 0)
        gripperMsg.args = "{position: 100.0}";
    else if(strcmp(req.action.c_str(), "close") == 0)
        gripperMsg.args = "{position: 0.0}";
    else{
        ROS_ERROR("gripper_action - please specify in service request, open or close command");
        return false;
    }
    gripperMsg.command = "go";
    gripperMsg.id = 65538;
    current_pub.publish(gripperMsg);

    ROS_INFO("Done!");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_position_node");
  ros::NodeHandle nh;

  ros::Publisher left_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", true);
  ros::Publisher right_gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", true);

  ros::ServiceServer service = nh.advertiseService<baxter_kinematics::GripperAction::Request,
          baxter_kinematics::GripperAction::Response>("baxter_kinematics/gripper_action", boost::bind(gripper_action, _1, _2, nh,
                                                                                                      boost::ref(left_gripper_pub),
                                                                                                      boost::ref(right_gripper_pub)));
  ROS_INFO("Ready to move to a position.");
  ros::spin();

  return 1;
}

