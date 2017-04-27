#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/GetEefPose.h"

std::vector<double> left_arm_joint_values(7), right_arm_joint_values(7), all_joint_values;
std::vector<std::string> all_joint_names;

bool get_sim_eef_pose_callback(baxter_kinematics::GetEefPose::Request &req,
                               baxter_kinematics::GetEefPose::Response &res,
                               ros::NodeHandle& nh){
    

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

    ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");    

    // Required to trigger the previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(robot_model);      
    if (req.eef_name == "left"){
        auto eef_values_tmp = get_eef_pose(robot_state,
                                        "left",
                                        left_arm_joint_values);
        Eigen::Vector3d eef_position = std::get<0>(eef_values_tmp);
        Eigen::Vector3d eef_rotation = std::get<1>(eef_values_tmp);
        res.pose = {eef_position(0), eef_position(1), eef_position(2),
                    eef_rotation(0), eef_rotation(1), eef_rotation(2)};

    } 
    else if (req.eef_name == "right"){
        auto eef_values_tmp = get_eef_pose(robot_state,
                                        "right",
                                        right_arm_joint_values);
        Eigen::Vector3d eef_position = std::get<0>(eef_values_tmp);
        Eigen::Vector3d eef_rotation = std::get<1>(eef_values_tmp);
        res.pose = {eef_position(0), eef_position(1), eef_position(2),
                    eef_rotation(0), eef_rotation(1), eef_rotation(2)};                    
    } else {
      ROS_ERROR_STREAM("get_eef_pose_callback - wrong eef name");
      res.pose = {};
    }

    return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_eef_pose_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService<baxter_kinematics::GetEefPose::Request,
                                                  baxter_kinematics::GetEefPose::Response>("baxter_kinematics/get_sim_eef_pose",
                                                                                           boost::bind(get_sim_eef_pose_callback, _1, _2, n));
  ROS_INFO("Ready to get eef pose.");
  ros::spin();

  return 0;
}
