#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/Trajectory.h"
#include <baxter_kinematics/GripperAction.h>
#include <boost/bind.hpp>

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr&  l_eef_feedback,
                       Kinematic_values& eef_values){
    locate_eef_pose(l_eef_feedback->pose, eef_values, "left_gripper");
}

//call back that register baxter right end effector pose and rearrange the orientation in RPY
void right_eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr&  r_eef_feedback,
                        Kinematic_values& eef_values){
    locate_eef_pose(r_eef_feedback->pose, eef_values, "right_gripper");
}

//store obj pos vector in eef values
void obj_state_cloud_Callback(const pcl_tracking::ObjectPosition::ConstPtr& topic_message,
                              Kinematic_values& eef_values){
    //std::vector< geometry_msgs::PointStamped > raw_pos_vector = topic_message->object_position;
    std::vector< mocap_optitrack::ObjectPositionID > raw_id_pos_vector = topic_message->object_position;
    std::vector< std::pair<int, std::vector<double> > > obj_pos_vector;
    std::vector<double> curr_obj_pos;
    int id;
    for(int i=0; i < raw_id_pos_vector.size(); i++){
        id = raw_id_pos_vector[i].ID;
        curr_obj_pos =  {raw_id_pos_vector[i].object_position.point.x,
                        raw_id_pos_vector[i].object_position.point.y,
                        raw_id_pos_vector[i].object_position.point.z};
        obj_pos_vector.push_back(std::make_pair(id,curr_obj_pos));

//        ROS_ERROR_STREAM("obj_state_cloud_Callback : obj_state_cloud_Callback: " << curr_obj_pos[0] << " " << curr_obj_pos[1] << " " << curr_obj_pos[2]);
    }

    eef_values.set_object_state_vector(obj_pos_vector);
}

bool trajectory_execution(baxter_kinematics::Trajectory::Request &req,
                          baxter_kinematics::Trajectory::Response &res,
                          ros::NodeHandle& nh,
                          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right,
                          ros::Publisher& traj_res_pub,
                          ros::ServiceClient& gripper_client,
                          Kinematic_values& eef_values){

    ROS_INFO("Establish communication tools");
    // Record start time
    auto start = std::chrono::high_resolution_clock::now();

    // Required to trigger previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    int curr_iter = 0;
    bool found = false;
    while (!found && curr_iter < 5){
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
        usleep(1e4);

        // move arms
        auto exec_start = std::chrono::high_resolution_clock::now();

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
                                                      gripper_client,
                                                      gripper_values_vector,
                                                      traj_res_pub);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            traj_res = plan_and_execute_waypoint_traj("right",
                                                      waypoints,
                                                      ac_right,
                                                      eef_values,
                                                      nh,
                                                      false, // force_orien
                                                      req.feedback,
                                                      gripper_client,
                                                      gripper_values_vector,
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

        auto exec_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> exec_elapsed = exec_finish - exec_start;
        ROS_ERROR_STREAM("Execution elapsed time: " << exec_elapsed.count() << " seconds");
    }

    ROS_INFO("Done trajectory_execution!\n");
    // Record end time
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ROS_ERROR_STREAM("Total elapsed time: " << elapsed.count() << " seconds");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_trajectory_node");
    ros::NodeHandle nh;
    Kinematic_values eef_values;

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10,
                                                                                  boost::bind(left_eef_Callback, _1, boost::ref(eef_values)));
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10,
                                                                                  boost::bind(right_eef_Callback, _1, boost::ref(eef_values)));
    std::string topic_name;
    bool real_robot;
    nh.getParam("real_robot", real_robot);
    if (real_robot)
        topic_name = "/visual/obj_pos_vector";
    ros::Subscriber sub_obj_state = nh.subscribe<pcl_tracking::ObjectPosition>(topic_name, 1,
                                                                                  boost::bind(obj_state_cloud_Callback, _1, boost::ref(eef_values)));

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);
    ros::ServiceClient gripper_client = nh.serviceClient<baxter_kinematics::GripperAction>("/baxter_kinematics/gripper_action");
    std::string feedback_topic_name;
    nh.getParam("feedback_topic", feedback_topic_name);
    ros::Publisher traj_res_pub = nh.advertise<std_msgs::Float64MultiArray>(feedback_topic_name, 1);

    ros::ServiceServer service = nh.advertiseService<
            baxter_kinematics::Trajectory::Request,
            baxter_kinematics::Trajectory::Response>("baxter_kinematics/execute_trajectory", boost::bind(trajectory_execution, _1, _2, nh,
                                                                                                    boost::ref(ac_l),
                                                                                                    boost::ref(ac_r),
                                                                                                    boost::ref(traj_res_pub),
                                                                                                    boost::ref(gripper_client),
                                                                                                    boost::ref(eef_values)));
    ROS_INFO("Ready to execute trajectory (service).");
    ros::spin();

    return 0;
}
