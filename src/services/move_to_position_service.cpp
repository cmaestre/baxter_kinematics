#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/MoveToPos.h"

std::vector<double> left_arm_joint_values(7), right_arm_joint_values(7), all_joint_values;
std::vector<std::string> all_joint_names;
Kinematic_values eef_values;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
//    ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position("left_gripper")
//                    << "\n and for orientation: " << eef_values.get_eef_rpy_orientation("left_gripper"));
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
    //    ROS_ERROR_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position("left_gripper")
    //                     << "\n and for orientation: " << eef_values.get_eef_rpy_orientation("left_gripper"));
}

bool move_to_pos(baxter_kinematics::MoveToPos::Request &req,
                      baxter_kinematics::MoveToPos::Response &res,
                      ros::NodeHandle& nh,
                      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right){

    ROS_INFO("Establish communication tools");
    bool real_robot;
    nh.getParam("real_robot", real_robot);
    if (!real_robot)
        ros::Subscriber sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,boost::bind(jocommCallback_sim,
                                                                                                                _1,
                                                                                                                left_arm_joint_values,
                                                                                                                right_arm_joint_values));
    else
        ros::Subscriber sub_jointmsg = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,boost::bind(jocommCallback_real,
                                                                                                                _1,
                                                                                                                left_arm_joint_values,
                                                                                                                right_arm_joint_values));

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);
    ros::Publisher pub_msg_left = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
    ros::ServiceClient baxter_left_ik_solver =
            nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
    ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // Required for communication with moveit components
    ros::AsyncSpinner spinner (1);
    spinner.start();

    ROS_INFO("Load robot description");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(robot_model);

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

//    //make sure that requested initial position is within available positions
//    if(req.initial_position_number > eef_values.get_nb_init_pos()){
//        ROS_ERROR("INVALID INITIAL POSITION NUMBER, Please enter a number less or equal to : %f", eef_values.get_nb_init_pos());
//        res.success = false;
//        return false;
//    }
//
//    //computing demanded init_pos
//    Eigen::Vector3d init_pos;
//    std::vector<Eigen::Vector3d> mid_point_position_vector(4);
//    geometry_msgs::Pose object_pose = eef_values.get_model_pose("cube");
//    double current_angle = req.initial_position_number * (2 * M_PI / eef_values.get_nb_init_pos());
//    eef_values.set_left_pos_x(object_pose.position.x + eef_values.get_radius() * cos(current_angle));
//    eef_values.set_left_pos_y(object_pose.position.y + eef_values.get_radius() * sin(current_angle));
//    init_pos << eef_values.get_left_pos_x(),
//                eef_values.get_left_pos_y(),
//                eef_values.get_left_pos_z();

    // Get init pos
    std::vector<Eigen::Vector3d> mid_point_position_vector(4);
    Eigen::Vector3d init_pos;
    init_pos << req.x,
                req.y,
                req.z;

    std::string temp_side = "no_side"; //eef_values.get_cube_side_value(4); //NO_SIDE
    std::vector<Eigen::Vector3d> dummy;

    // get current position
    std::vector<std::string> left_joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    std::vector<std::string> right_joint_names = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    if(!real_robot)
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }
    else
        robot_state.setVariablePositions(all_joint_names, all_joint_values);

    std::vector<geometry_msgs::Pose> waypoints =
        compute_directed_waypoints(true,
                                   robot_state,
                                   init_pos, // goal
                                   temp_side,
                                   mid_point_position_vector,
                                   eef_values);

    if(strcmp(req.eef_name.c_str(), "left") == 0)
        plan_and_execute_waypoint_traj("left",
                                       waypoints,
                                       robot_state,
                                       ac_left,
                                       gazebo_model_state,
                                       "cube",
                                       dummy,
                                       dummy,
                                       dummy,
                                       dummy,
                                       eef_values);
    else if(strcmp(req.eef_name.c_str(), "right") == 0)
        plan_and_execute_waypoint_traj("right",
                                       waypoints,
                                       robot_state,
                                       ac_right,
                                       gazebo_model_state,
                                       "cube",
                                       dummy,
                                       dummy,
                                       dummy,
                                       dummy,
                                       eef_values);

    //make sure the end effector is at desired initial position
    // get current position
    if(!real_robot)
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);
        else{
            ROS_ERROR("please specify in service request, left or right arm");
            return false;
        }
    else
        robot_state.setVariablePositions(all_joint_names, all_joint_values);

    Eigen::Affine3d f_trans_mat;
    if(strcmp(req.eef_name.c_str(), "left") == 0)
        f_trans_mat = robot_state.getGlobalLinkTransform("left_gripper");
    else if(strcmp(req.eef_name.c_str(), "right") == 0)
        f_trans_mat = robot_state.getGlobalLinkTransform("right_gripper");
    else{
        ROS_ERROR("please specify in service request, left or right arm");
        return false;
    }

    if((init_pos - f_trans_mat.translation()).norm() < 0.025){
        ROS_INFO_STREAM("SUCCESSFUL INIT MOTION : " << (init_pos - f_trans_mat.translation()).norm());
        res.success = true;
    }
    else{
        ROS_ERROR_STREAM("FAILED INIT MOTION : " << (init_pos - f_trans_mat.translation()).norm());
        res.success = false;
    }

    // Wait till user kills the process (Control-C)
    spinner.stop();
    ROS_INFO("Done!");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_position_node");
  ros::NodeHandle n;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);

  ros::ServiceServer service = n.advertiseService<baxter_kinematics::MoveToPos::Request,
          baxter_kinematics::MoveToPos::Response>("baxter_kinematics/move_to_position", boost::bind(move_to_pos, _1, _2, n,
                                                                                                     boost::ref(ac_l),
                                                                                                     boost::ref(ac_r)));
  ROS_INFO("Ready to move to a position.");
  ros::spin();

  return 0;
}
