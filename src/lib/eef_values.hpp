#ifndef __EEF_VALUES_HPP__
#define __EEF_VALUES_HPP__

#include <Eigen/Core>
#include <tf/tf.h>

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_kinematics/GripperAction.h>
#include "baxter_core_msgs/EndEffectorCommand.h"

#include <moveit/move_group_interface/move_group.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
//#include "environment_functionalities/GetObjectState.h"
//#include "visual_functionalities/GetObjectStateBlob.h"
//#include "pcl_tracking/ObjectPosition.h"

#include <chrono>  // for high_resolution_clock

struct Eef_values {

    geometry_msgs::Pose l_eef_pose, r_eef_pose;
    Eigen::VectorXd l_eef_rpy_pose, r_eef_rpy_pose;
    Eigen::Vector3d l_eef_position, r_eef_position;
    Eigen::Vector3d l_eef_rpy_orientation, r_eef_rpy_orientation;

    std::string baxter_arm;

    double left_gripper_openness, right_gripper_openness;

    boost::shared_ptr<moveit::planning_interface::MoveGroup> move_group_left_pt_;
    boost::shared_ptr<moveit::planning_interface::MoveGroup> move_group_right_pt_;

    ros::ServiceClient client_get_object_pose;
    std::string visual_type;

//    std::vector< std::vector<double> > obj_pos_vector;

};

class Kinematic_values{

public:

    Eef_values eef_values;

    Kinematic_values(){
        eef_values.move_group_left_pt_.reset(new moveit::planning_interface::MoveGroup("left_arm"));
        eef_values.move_group_left_pt_->setPlannerId("RRTConnectkConfigDefault");
        eef_values.move_group_right_pt_.reset(new moveit::planning_interface::MoveGroup("right_arm"));
        eef_values.move_group_right_pt_->setPlannerId("RRTConnectkConfigDefault");
    }

    std::string& get_baxter_arm(){
        return eef_values.baxter_arm;
    }

    ////////////////////////////////////////////

    // getters
    boost::shared_ptr<moveit::planning_interface::MoveGroup> get_move_group(std::string arm_selected){
        if(strcmp(arm_selected.c_str(), "left_arm") == 0)
            return eef_values.move_group_left_pt_;
        else
            return eef_values.move_group_right_pt_;
    }

//    std::vector< std::vector<double> > get_object_state_vector(){
//        return eef_values.obj_pos_vector;
//    }

//    void set_object_state_vector(std::vector< std::vector<double> > obj_pos_vector_){
//        eef_values.obj_pos_vector = obj_pos_vector_;
//    }


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

    double get_gripper_openness(std::string eef_name){

        if(strcmp(eef_name.c_str(), "left") == 0)
            return eef_values.left_gripper_openness;
        else if(strcmp(eef_name.c_str(), "right") == 0)
            return eef_values.right_gripper_openness;
        else{
            ROS_ERROR("get_gripper_openness - please specify in service request, left or right arm");
            return -1;
        }
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

    void set_gripper_openness(std::string eef_name, double openness){
        if(strcmp(eef_name.c_str(), "left") == 0)
            eef_values.left_gripper_openness = openness;
        else if(strcmp(eef_name.c_str(), "right") == 0)
            eef_values.right_gripper_openness = openness;
        else{
            ROS_ERROR("set_gripper_openness - please specify in service request, left or right arm");
        }

    }


};

#endif
