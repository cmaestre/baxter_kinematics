#ifndef __EEF_VALUES_HPP__
#define __EEF_VALUES_HPP__

#include <Eigen/Core>
#include <tf/tf.h>

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_kinematics/GripperAction.h>

#include <moveit/move_group_interface/move_group.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

#include <geometry_msgs/Pose.h>
#include "environment_functionalities/GetObjectState.h"

struct Eef_values {

    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;

    std::string baxter_arm;

    double left_gripper_openness, right_gripper_openness;

};

class Kinematic_values{

public:

    Eef_values eef_values;

    std::string& get_baxter_arm(){
        return eef_values.baxter_arm;
    }

    ////////////////////////////////////////////

    // getters
    Eigen::VectorXd& get_eef_rpy_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0)
            return eef_values.l_eef_rpy_pose;
        else
            return eef_values.r_eef_rpy_pose;
    }

    geometry_msgs::Pose& get_eef_pose(const std::string gripper){
        if(strcmp(gripper.c_str(), "left_gripper") == 0){
            return eef_values.l_eef_pose;
        }
        else{
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

    double get_left_gripper_openness(){
        return eef_values.left_gripper_openness;
    }

    double get_right_gripper_openness(){
        return eef_values.right_gripper_openness;
    }

    // getters
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

    void set_left_gripper_openness(double openness){
        eef_values.left_gripper_openness = openness;
    }

    void set_right_gripper_openness(double openness){
        eef_values.right_gripper_openness = openness;
    }


};

#endif
