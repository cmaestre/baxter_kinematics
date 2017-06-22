#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/Trajectory.h"
#include <baxter_kinematics/GripperAction.h>
#include <boost/bind.hpp>

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
Kinematic_values eef_values;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
}

bool trajectory_execution(baxter_kinematics::Trajectory::Request &req,
                          baxter_kinematics::Trajectory::Response &res,
                          ros::NodeHandle& nh,
                          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right,
                          ros::Publisher& traj_res_pub,
                          ros::ServiceClient& gripper_client){

    ROS_INFO("Establish communication tools");

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);
    // Required to trigger previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    int curr_iter = 0;
    bool found = false;
    while (!found and curr_iter < 5){
        ROS_INFO("Load robot description");
        // set arm
        std::string left_arm = "left_arm";
        std::string right_arm = "right_arm";
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            eef_values.set_baxter_arm(left_arm);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            eef_values.set_baxter_arm(right_arm);
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }

        // get initial pose orientation
        std::string gripper;
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            gripper = "left_gripper";
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            gripper = "right_gripper";
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }
        geometry_msgs::Pose start_pose = eef_values.get_eef_pose(gripper);
        ROS_ERROR_STREAM("eef orientation is: " << start_pose);

        // get trajectory
        std::vector<double> received_traj_vector = req.trajectory;
        std::vector<geometry_msgs::Pose> waypoints;
        for (std::size_t i=0; i<received_traj_vector.size(); i=i+3){
            start_pose.position.x = received_traj_vector[i];
            start_pose.position.y = received_traj_vector[i+1];
            start_pose.position.z = received_traj_vector[i+2];
            waypoints.push_back(start_pose);
            ROS_ERROR_STREAM(start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z);
        }

        // move arms
        std::vector<std::string> gripper_values_vector = req.gripper_values;
        int traj_res;
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            traj_res = plan_and_execute_waypoint_traj("left",
                                                      waypoints,
                                                      ac_left,
                                                      eef_values,
                                                      nh,
                                                      false, // force_orien
                                                      req.feedback,
                                                      traj_res_pub,
                                                      "cube", //for feedback
                                                      gripper_values_vector,
                                                      gripper_client);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            traj_res = plan_and_execute_waypoint_traj("right",
                                                      waypoints,
                                                      ac_right,
                                                      eef_values,
                                                      nh,
                                                      false, // force_orien
                                                      req.feedback,
                                                      traj_res_pub,
                                                      "cube", //for feedback
                                                      gripper_values_vector,
                                                      gripper_client);
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }
        if (curr_iter == 3 or traj_res == 0)
            res.success = false;
        else if (traj_res == 1){
            res.success = true;
            found = true;
        }
        else if (traj_res == 2)
            ROS_ERROR_STREAM("plan_and_execute_waypoint_traj - try: " << curr_iter);
        curr_iter++;
    }

    // Wait till user kills the process (Control-C)
    ROS_INFO("Done!\n");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_trajectory_node");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);
    ros::Publisher traj_res_pub = nh.advertise<std_msgs::Float64MultiArray>("/baxter_kinematics/traj_exec_info", 1);
    ros::ServiceClient gripper_client = nh.serviceClient<baxter_kinematics::GripperAction>("/baxter_kinematics/gripper_action");

    ros::ServiceServer service = nh.advertiseService<
            baxter_kinematics::Trajectory::Request,
            baxter_kinematics::Trajectory::Response>("baxter_kinematics/execute_trajectory", boost::bind(trajectory_execution, _1, _2, nh,
                                                                                                    boost::ref(ac_l),
                                                                                                    boost::ref(ac_r),
                                                                                                    boost::ref(traj_res_pub),
                                                                                                    boost::ref(gripper_client)));
    ROS_INFO("Ready to execute trajectory (set of delta motions).");
    ros::spin();

    return 0;
}
