#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/Trajectory.h"
#include <boost/bind.hpp>

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
std::vector<double> left_arm_joint_values(7), right_arm_joint_values(7), all_joint_values;
std::vector<std::string> all_joint_names;
Kinematic_values eef_values;

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

bool trajectory_execution(baxter_kinematics::Trajectory::Request &req,
                          baxter_kinematics::Trajectory::Response &res,
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
                                                                                                 boost::ref(right_arm_joint_values),
                                                                                                 boost::ref(all_joint_names),
                                                                                                 boost::ref(all_joint_values)));
    }

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);
    ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Publisher traj_res_pub = nh.advertise<std_msgs::Float64MultiArray>("/baxter_kinematics/traj_exec_info", 1);
    // Required to trigger previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    int curr_iter = 0;
    bool found = false;
    while (!found and curr_iter < 5){
        ROS_INFO("Load robot description");
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
        robot_state::RobotState robot_state(robot_model);

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

        // get current position
        std::vector<std::string> left_joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
        std::vector<std::string> right_joint_names = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
//        if(!real_robot)
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }
//        else
//            robot_state.setVariablePositions(all_joint_names, all_joint_values);

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

        // match joints
//        if(!real_robot)
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }
//        else
//            robot_state.setVariablePositions(all_joint_names, all_joint_values);

        // move arms
        std::vector<Eigen::Vector3d> eef_position_vector;
        std::vector<Eigen::Vector3d> eef_orientation_vector;
        std::vector<Eigen::Vector3d> object_position_vector;
        std::vector<Eigen::Vector3d> object_orientation_vector;
        eef_position_vector.clear();
        eef_orientation_vector.clear();
        object_position_vector.clear();
        object_orientation_vector.clear();
        int traj_res;
        int feedback_frequency = 1;
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            traj_res = plan_and_execute_waypoint_traj("left",
                                                      waypoints,
                                                      robot_state,
                                                      ac_left,
                                                      "cube", //for feedback
                                                      eef_position_vector,
                                                      eef_orientation_vector,
                                                      object_position_vector,
                                                      object_orientation_vector,
                                                      eef_values,
                                                      nh,
                                                      req.feedback,
                                                      true, //publish topic
                                                      feedback_frequency,
                                                      traj_res_pub);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            traj_res = plan_and_execute_waypoint_traj("right",
                                                      waypoints,
                                                      robot_state,
                                                      ac_right,
                                                      "cube", //for feedback
                                                      eef_position_vector,
                                                      eef_orientation_vector,
                                                      object_position_vector,
                                                      object_orientation_vector,
                                                      eef_values,
                                                      nh,
                                                      req.feedback,
                                                      true, //publish topic
                                                      feedback_frequency,
                                                      traj_res_pub);
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

    //    //make sure the end effector is at desired position
    //    f_trans_mat = robot_state.getGlobalLinkTransform("left_gripper");
    //    std::cout << "position after motion is: \n" << f_trans_mat.translation() << std::endl << "************************************" << std::endl;
    //    // if((goal - f_trans_mat.translation()).norm() < 0.005){
    //    if((current_wp - f_trans_mat.translation()).norm() < 0.005){
    //        ROS_INFO("SUCCESSFUL DELTA MOTION");
    //        res.success = true;
    //    }
    //    else{
    //        ROS_ERROR("FAILED DELTA MOTION");
    //        res.success = false;
    //    }

    // Wait till user kills the process (Control-C)
    spinner.stop();
    ROS_INFO("Done!\n");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_trajectory_node");
    ros::NodeHandle n;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);

    ros::ServiceServer service = n.advertiseService<
            baxter_kinematics::Trajectory::Request,
            baxter_kinematics::Trajectory::Response>("baxter_kinematics/execute_trajectory", boost::bind(trajectory_execution, _1, _2, n,
                                                                                                    boost::ref(ac_l),
                                                                                                    boost::ref(ac_r)));
    ROS_INFO("Ready to execute trajectory (set of delta motions).");
    ros::spin();

    return 0;
}
