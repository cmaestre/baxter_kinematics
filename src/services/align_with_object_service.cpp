#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/AlignWithObject.h"
#include "baxter_kinematics/RotateToAngle.h"
#include "baxter_kinematics/GetEefPose.h"

#include <ros/ros.h>
#include "baxter_core_msgs/JointCommand.h"

#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/GetLinkState.h"

#include <cmath>
#include <math.h>

using namespace std;

Kinematic_values eef_values;

bool align_with_object(baxter_kinematics::AlignWithObject::Request &req,
                       baxter_kinematics::AlignWithObject::Response &res,
                       ros::NodeHandle &nh)
{

    ROS_INFO("Establish communication tools");

    // Required for communication with moveit components
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    std::string left_arm = "left_arm";
    std::string right_arm = "right_arm";

    std::string link_name = "";
    std::string joint_name = "";
    std::string eef_name = "";

    if (strcmp(req.eef_name.c_str(), "left") == 0)
    {
        eef_values.set_baxter_arm(left_arm);
        link_name = "baxter::left_wrist";
        joint_name = "left_w2";
        eef_name = "left";
    }
    else if (strcmp(req.eef_name.c_str(), "right") == 0)
    {
        eef_values.set_baxter_arm(right_arm);
        link_name = "baxter::right_wrist";
        joint_name = "right_w2";
        eef_name = "right";
    }
    else
    {
        ROS_ERROR("please specify in service request, left or right arm");
        return false;
    }

    // Get object orientation
    ros::ServiceClient client_get_object_state = nh.serviceClient<environment_functionalities::GetObjectState>("/env/get_object_state");
    environment_functionalities::GetObjectState getObjectStateSrv;
    getObjectStateSrv.request.object_name = req.object;
    client_get_object_state.call(getObjectStateSrv);
    std::vector<double> object_state_vector = getObjectStateSrv.response.object_state;
    ROS_WARN("Object orientation (rpy): %f; %f; %f", object_state_vector[3], object_state_vector[4], object_state_vector[5]);

    // Get value of joint angle
    ros::ServiceClient get_joint_properties_state = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    gazebo_msgs::GetJointProperties getJointPropertiesSrv;
    getJointPropertiesSrv.request.joint_name = joint_name;

    // Get eef orientation
    ros::ServiceClient get_link_state = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    gazebo_msgs::GetLinkState getLinkStateSrv;
    getLinkStateSrv.request.link_name = link_name;

    // Rotate eef to angle 'new_angle'
    ros::ServiceClient rotate_to_angle = nh.serviceClient<baxter_kinematics::RotateToAngle>("/baxter_kinematics/rotate_to_angle");
    baxter_kinematics::RotateToAngle rotateToAngleSrv;
    rotateToAngleSrv.request.eef_name = eef_name;

    bool is_oriented = false;
    float offset = 0.1;
    float direction = -1.0;
    float epsilon = 0.1;

    // int nb_tries = 0;
    // while (!is_oriented && nb_tries < 5000)
    while (!is_oriented)
    {
        ROS_WARN("  ");

        // Get value of joint angle
        get_joint_properties_state.call(getJointPropertiesSrv);
        double joint_angle = getJointPropertiesSrv.response.position[0];
        ROS_WARN(" >>>>> Current joint angle: %f <<<<<", joint_angle);

        // Get eef orientation
        get_link_state.call(getLinkStateSrv);

        // Transform msg quaternion from response to TF quaternion
        tf::Quaternion eef_q;
        tf::quaternionMsgToTF(getLinkStateSrv.response.link_state.pose.orientation, eef_q);

        // Transform quaternion to RPY
        tf::Matrix3x3 m(eef_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_WARN(" >>>>> Current joint eef orientation: %f <<<<<", yaw);

        if (std::abs(object_state_vector[5] - yaw) < epsilon)
        {
            is_oriented = true;
            ROS_WARN(" >>>>> EEF and object are aligned <<<<<");
        }
        else
        {
            // Upper and lower joint limit
            if (joint_angle > 3.059)
                direction = -1 * direction;

            if (joint_angle < -3.059)
                direction = -1 * direction;

            double new_angle = joint_angle + direction * offset;

            // Rotate eef to angle 'new_angle'
            rotateToAngleSrv.request.angle = new_angle;
            rotate_to_angle.call(rotateToAngleSrv);
        }

        // nb_tries++;
    }

    // ROS_INFO("Done!");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_with_object");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService<baxter_kinematics::AlignWithObject::Request,
                                                     baxter_kinematics::AlignWithObject::Response>("baxter_kinematics/align_with_object", boost::bind(align_with_object, _1, _2, nh));
    ROS_INFO("Ready to align with the object.");
    ros::spin();

    return 0;
}
