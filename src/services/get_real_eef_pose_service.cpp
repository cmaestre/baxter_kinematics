#include "../parameters.hpp"
#include "baxter_kinematics/lib_recording.hpp"
#include "baxter_kinematics/GetEefPose.h"
#include <boost/bind.hpp>

#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/DigitalIOState.h>

#include <sensor_msgs/PointCloud2.h>

// The parameters structure is used by all call backs, main and service
Data_config parameters;

//call back that register end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_left_eef_pose(l_eef_feedback, parameters);
}

bool get_real_eef_pose_service_callback(baxter_kinematics::GetEefPose::Request &req,
                                           baxter_kinematics::GetEefPose::Response &res,
                                           ros::NodeHandle& nh,
                                           image_transport::ImageTransport& it_){

    //subscribers
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);

    ros::AsyncSpinner my_spinner(1);
    my_spinner.start();
    usleep(1e6);

    if (req.eef_name == "left"){

        //get current eef pose
        Eigen::VectorXd current_values(6);
        current_values = parameters.get_left_eef_pose_rpy();

        res.pose = {current_values(0), current_values(1), current_values(2),
                    current_values(3), current_values(4), current_values(5)};
    } else {
        ROS_ERROR_STREAM("get_real_eef_pose_service_callback - only left eef coded now");
        res.pose = {};
    }

    ROS_INFO("Done.");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_real_eef_pose_service");
  ros::NodeHandle n;

  image_transport::ImageTransport it_(n);
  //int test;
  ros::ServiceServer service = n.advertiseService<baxter_kinematics::GetEefPose::Request,
                                                  baxter_kinematics::GetEefPose::Response>("baxter_kinematics/get_real_eef_pose",
                                                  boost::bind(get_real_eef_pose_service_callback, _1, _2, n, it_));
  ROS_INFO("Ready to get real eef pose.");
  ros::spin();

  return 0;
}
