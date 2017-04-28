#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/RestartRobot.h"

std::vector<double> left_arm_joint_values(7), right_arm_joint_values(7), all_joint_values;
std::vector<std::string> all_joint_names;
Kinematic_values eef_values;

void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position()
                //    << "\n and for orientation: " << eef_values.get_eef_rpy_orientation());
}

//call back that register baxter right end effector pose and rearrange the orientation in RPY
void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position()
                //    << "\n and for orientation: " << eef_values.get_eef_rpy_orientation());
}

int restart_robot(baxter_kinematics::RestartRobot::Request &req,
                  baxter_kinematics::RestartRobot::Response &res,
                  ros::NodeHandle& nh,
                  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right){

    ROS_INFO("Establish communication tools");
    bool real_robot;
    nh.getParam("real_robot", real_robot);
    ros::Subscriber sub_jointmsg;
    if (!real_robot){
        sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,boost::bind(jocommCallback_sim,
                                                                                                 _1,
                                                                                                 boost::ref(left_arm_joint_values),
                                                                                                 boost::ref(right_arm_joint_values)));
    }
    else{
        sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,boost::bind(jocommCallback_real,
                                                                                                 _1,
                                                                                                 boost::ref(left_arm_joint_values),
                                                                                                 boost::ref(right_arm_joint_values)));
    }
    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);

    // Required for communication with moveit components
    ros::AsyncSpinner spinner (1);
    spinner.start();

    ROS_INFO("Load robot description");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(robot_model);

//    // Move to initial position
//    robot_state.setVariablePositions(all_joint_names, all_joint_values);

    bool mov_res = restart_robot_initial_position(robot_state,
                                                  eef_values,
                                                  ac_left,
                                                  ac_right,
                                                  nh,
                                                  left_arm_joint_values,
                                                  right_arm_joint_values,
                                                  all_joint_values);
    if (!mov_res){
        ROS_ERROR_STREAM("restart_robot : Problem restarting robot");
        res.success = false;
        return 0;
    }

    res.success = true;
    spinner.stop();
    ROS_INFO("Done!");
    return 1;
}

int main(int argc, char** argv)
{

    // Initialize ROS
    ros::init(argc, argv, "restart_world_node");
    ros::NodeHandle nh;    

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_left("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_right("/robot/limb/right/follow_joint_trajectory", true);
    ros::ServiceServer service = nh.advertiseService<baxter_kinematics::RestartRobot::Request,
            baxter_kinematics::RestartRobot::Response>("baxter_kinematics/restart_robot", boost::bind(restart_robot, _1, _2, nh,
                                                                                                      boost::ref(ac_left), boost::ref(ac_right)));
    ROS_INFO("Ready to restart robot.");
    ros::spin();

    return 0;
}
