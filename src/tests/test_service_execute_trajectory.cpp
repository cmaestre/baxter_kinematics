#include "../../include/baxter_kinematics/lib_movement.hpp"
#include <baxter_core_msgs/EndpointState.h>
#include "baxter_kinematics/MoveToPos.h"
#include <baxter_kinematics/Trajectory.h>
#include <time.h>

Kinematic_values eef_values;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_service_execute_trajectory_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);
    ros::ServiceClient move_to_position = nh.serviceClient<baxter_kinematics::MoveToPos>("baxter_kinematics/move_to_position");
    ros::ServiceClient execute_trajectory = nh.serviceClient<baxter_kinematics::Trajectory>("baxter_kinematics/execute_trajectory");

    ros::AsyncSpinner spinner (1);
    spinner.start();

    baxter_kinematics::MoveToPosRequest req;
    baxter_kinematics::MoveToPosResponse res;

    baxter_kinematics::Trajectory::Request req_traj;
    baxter_kinematics::Trajectory::Response res_traj;

    //move right arm to hame position
    req.eef_name = "right";
    req.x = 0.18;
    req.y = -0.6;
    req.z = 0.17;
    move_to_position.call(req, res);

    while(ros::ok()){
        srand(time(0));

        Eigen::Vector3d current_position;
        current_position = eef_values.get_eef_position("left_gripper");
        double min_val = -0.1, max_val = 0.1;
        Eigen::Vector3d goal_position;
        goal_position << current_position(0) + (max_val - min_val) * ( (double)rand() / (double)RAND_MAX ) + min_val,
                current_position(1) + (max_val - min_val) * ( (double)rand() / (double)RAND_MAX ) + min_val,
                current_position(2) + (max_val - min_val) * ( (double)rand() / (double)RAND_MAX ) + min_val;

        ROS_WARN_STREAM("start position is: " << current_position);
        ROS_WARN_STREAM("goal position is: " << goal_position);

        req_traj.eef_name = "left";
        req_traj.feedback = false;
        req_traj.trajectory = {current_position(0), current_position(1), current_position(2),
                              goal_position(0), goal_position(1), goal_position(2)};
        execute_trajectory.call(req_traj, res_traj);
    }
    return 0;
}
