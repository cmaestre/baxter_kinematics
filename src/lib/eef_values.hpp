#ifndef __EEF_VALUES_HPP__
#define __EEF_VALUES_HPP__

#include <string>
#include <math.h>
#include <Eigen/Core>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

struct Eef_values {

    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;

    std::string baxter_arm;

};

class Kinematic_values{

public:

    Eef_values eef_values;

    std::string& get_baxter_arm(){
        return eef_values.baxter_arm;
    }

    ////////////////////////////////////////////

    //left and right grippers pose variables getters
    Eigen::VectorXd& get_eef_rpy_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return eef_values.l_eef_rpy_pose;
        else
            return eef_values.r_eef_rpy_pose;
    }

    geometry_msgs::Pose& get_eef_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0){
            ROS_WARN("******************************* returning left gripper pose *****************************");
            return eef_values.l_eef_pose;
        }
        else{
            ROS_WARN("*************************** returning right gripper pose *******************************");
            return eef_values.r_eef_pose;
        }
    }

    Eigen::Vector3d& get_eef_position(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return eef_values.l_eef_position;
        else
            return eef_values.r_eef_position;
    }

    Eigen::Vector3d& get_eef_rpy_orientation(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return eef_values.l_eef_rpy_orientation;
        else
            return eef_values.r_eef_rpy_orientation;
    }

    //left and right grippers pose variables getters
    void set_eef_pose(geometry_msgs::Pose& eef_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            eef_values.l_eef_pose = eef_pose;
        else
            eef_values.r_eef_pose = eef_pose;
    }

    void set_eef_rpy_pose(Eigen::VectorXd& eef_rpy_pose, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            eef_values.l_eef_rpy_pose = eef_rpy_pose;
        else
            eef_values.r_eef_rpy_pose = eef_rpy_pose;
    }

    void set_eef_position(Eigen::Vector3d& eef_position, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            eef_values.l_eef_position = eef_position;
        else
            eef_values.r_eef_position = eef_position;
    }

    void set_eef_rpy_orientation(Eigen::Vector3d& eef_rpy_orientation, const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            eef_values.l_eef_rpy_orientation = eef_rpy_orientation;
        else
            eef_values.r_eef_rpy_orientation = eef_rpy_orientation;
    }

    void set_baxter_arm(std::string& baxter_arm){
        eef_values.baxter_arm = baxter_arm;
    }
};

#endif
