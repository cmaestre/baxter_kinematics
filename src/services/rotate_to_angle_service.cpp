#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/RotateToAngle.h"

#include <ros/ros.h>
#include "baxter_core_msgs/JointCommand.h"

using namespace std;

Kinematic_values eef_values;

bool rotate_to_angle(baxter_kinematics::RotateToAngle::Request &req,
                     baxter_kinematics::RotateToAngle::Response &res,
                     ros::NodeHandle &nh)
{

    ROS_INFO("Establish communication tools");

    // Required for communication with moveit components
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    std::string left_arm = "left_arm";
    std::string right_arm = "right_arm";
    std::string joint_name = "";
    std::string topic = "";

    if (strcmp(req.eef_name.c_str(), "left") == 0)
    {
        eef_values.set_baxter_arm(left_arm);
        joint_name = "left_w2";
        topic = "/robot/limb/left/joint_command";
    }
    else if (strcmp(req.eef_name.c_str(), "right") == 0)
    {
        eef_values.set_baxter_arm(right_arm);
        joint_name = "right_w2";
        topic = "/robot/limb/right/joint_command";
    }
    else
    {
        ROS_ERROR("please specify in service request, left or right arm");
        return false;
    }

    ros::Publisher left_cmd_pub = nh.advertise<baxter_core_msgs::JointCommand>(topic, 1);
    ros::Rate loop_rate(20);

    baxter_core_msgs::JointCommand cmd;
    cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    cmd.names.push_back(joint_name);
    cmd.command.resize(1);
    cmd.command.resize(cmd.names.size());

    ROS_INFO("Request to rotate to %f rad", req.angle);
    cmd.command[0] = req.angle;

    int count = 0;

    // while (ros::ok())
    while (count < 10)
    {
        left_cmd_pub.publish(cmd);
        ros::spinOnce();
        // ros::spin();
        loop_rate.sleep();

        count++;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_to_angle");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService<baxter_kinematics::RotateToAngle::Request,
                                                     baxter_kinematics::RotateToAngle::Response>("baxter_kinematics/rotate_to_angle", boost::bind(rotate_to_angle, _1, _2, nh));
    ROS_INFO("Ready to rotate to angle.");
    ros::spin();

    return 0;
}
