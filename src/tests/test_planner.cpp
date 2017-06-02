#include "../../include/baxter_kinematics/lib_movement.hpp"
#include <baxter_core_msgs/EndpointState.h>
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
    ros::init(argc, argv, "test_planner_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);

    ros::AsyncSpinner spinner (1);
    spinner.start();

    usleep(1e6);

    std::string left_arm = "left_arm";
    std::string right_arm = "right_arm";
    eef_values.set_baxter_arm(right_arm);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(eef_values.get_eef_pose("right_gripper"));
    geometry_msgs::Pose start_pose = eef_values.get_eef_pose("right_gripper");
    start_pose.position.x = 0.18;
    start_pose.position.y = -0.6;
    start_pose.position.z = 0.17;
    waypoints.push_back(start_pose);
//        ROS_ERROR_STREAM("Right arm initial position : " << start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z);

//    std::vector<Eigen::Vector3d> dummy;
//    bool right_res =
//            plan_and_execute_waypoint_traj("right",
//                                           waypoints,
//                                           ac_r,
//                                           eef_values,
//                                           nh);

    eef_values.set_baxter_arm(left_arm);
    while(ros::ok()){
        srand(time(0));
        Eigen::Vector3d current_position;
        current_position = eef_values.get_eef_position("left_gripper");
        double min_val = -0.1, max_val = 0.1;

        std::vector<Eigen::Vector3d> mid_point_position_vector(4);
        Eigen::Vector3d goal_position;
        goal_position << current_position(0) + (max_val - min_val) * ( (double)rand() / (double)RAND_MAX ) + min_val,
                current_position(1) + (max_val - min_val) * ( (double)rand() / (double)RAND_MAX ) + min_val,
                current_position(2) + (max_val - min_val) * ( (double)rand() / (double)RAND_MAX ) + min_val;

        ROS_WARN_STREAM("start position is: " << current_position);
        ROS_WARN_STREAM("goal position is: " << goal_position);

        std::string temp_side = "no_side"; //eef_values.get_cube_side_value(4); //NO_SIDE
        std::vector<Eigen::Vector3d> dummy;
        std::vector<geometry_msgs::Pose> waypoints =
                compute_directed_waypoints(true,
                                           goal_position,
                                           temp_side,
                                           mid_point_position_vector,
                                           eef_values);

        plan_and_execute_waypoint_traj("left",
                                       waypoints,
                                       ac_l,                                       
                                       eef_values,
                                       nh);
    }
    return 0;
}
