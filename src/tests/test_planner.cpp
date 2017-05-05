#include "../../include/baxter_kinematics/lib_movement.hpp"
#include <baxter_core_msgs/EndpointState.h>
#include <time.h>

Kinematic_values eef_values;
std::vector<double> left_arm_joint_values(7), right_arm_joint_values(7), all_joint_values;
std::vector<std::string> all_joint_names;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
    //    ROS_ERROR_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position("left_gripper")
    //                     << "\n and for orientation: " << eef_values.get_eef_rpy_orientation("left_gripper"));
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
    //    ROS_ERROR_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position("left_gripper")
    //                     << "\n and for orientation: " << eef_values.get_eef_rpy_orientation("left_gripper"));
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_planner_node");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);

    ros::Subscriber sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,boost::bind(jocommCallback_sim,
                                                                                                             _1,
                                                                                                             boost::ref(left_arm_joint_values),
                                                                                                             boost::ref(right_arm_joint_values)));
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);


    ros::AsyncSpinner spinner (1);
    spinner.start();

    ROS_INFO("Load robot description");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(robot_model);

    std::vector<std::string> left_joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    std::vector<std::string> right_joint_names = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);
    robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);

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

    std::vector<Eigen::Vector3d> dummy;
    bool right_res =
            plan_and_execute_waypoint_traj("right",
                                           waypoints,
                                           robot_state,
                                           ac_r,
                                           "",
                                           dummy,
                                           dummy,
                                           dummy,
                                           dummy,
                                           eef_values,
                                           nh);

    eef_values.set_baxter_arm(left_arm);
    while(ros::ok()){
        robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);
        robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);
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
                                           robot_state,
                                           goal_position,
                                           temp_side,
                                           mid_point_position_vector,
                                           eef_values);

        plan_and_execute_waypoint_traj("left",
                                       waypoints,
                                       robot_state,
                                       ac_l,
                                       "cube",
                                       dummy,
                                       dummy,
                                       dummy,
                                       dummy,
                                       eef_values,
                                       nh);

    }
    return 0;
}
