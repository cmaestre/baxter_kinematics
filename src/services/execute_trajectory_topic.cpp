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
    std::vector< std::pair<int, std::vector<double> > > obj_pos_vector;
    for(unsigned i=0; i < topic_message->object_position.size(); i++){
        std::pair<int, std::vector<double> > curr_pair (
                      topic_message->object_position[i].ID,
                    { topic_message->object_position[i].object_position.point.x,
                      topic_message->object_position[i].object_position.point.y,
                      topic_message->object_position[i].object_position.point.z } );
        obj_pos_vector.push_back(curr_pair);
    }
//        ROS_ERROR_STREAM("obj_state_cloud_Callback : obj_state_cloud_Callback: " << curr_obj_pos[0] << " " << curr_obj_pos[1] << " " << curr_obj_pos[2]);

    eef_values.set_object_state_vector(obj_pos_vector);
}

// a
void left_goal_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg,
                        Kinematic_values& eef_values)
{
    if (!msg->status_list.empty()){
        if (msg->status_list.back().status == 3){ // SUCCESS
            ROS_WARN("SUCCESSFUL GOAL !");
            eef_values.set_left_goal_status(1);
//            ros::Duration(0.2).sleep();
        }

        else if ((msg->status_list.size() > 1) &&
                 (msg->status_list.back().status == 0) &&
                 (msg->status_list[msg->status_list.size()-2].status == 3)){ // PREEMTED
            ROS_WARN("PREEMTED GOAL !");
            eef_values.set_left_goal_status(2);
//            ros::Duration(0.2).sleep();
        }
    } else
        eef_values.set_left_goal_status(0);
}

// a
void right_goal_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg,
                        Kinematic_values& eef_values)
{
//    if (!msg->status_list.empty()){
//        int curr_status = msg->status_list[0].status;
//        eef_values.set_right_goal_status(curr_status);
//    } else
//        eef_values.set_right_goal_status(0);

    if (!msg->status_list.empty()){
        if (msg->status_list.back().status == 3) // SUCCESS
            eef_values.set_right_goal_status(1);
        else if ((msg->status_list.size() > 1) &&
                 (msg->status_list.back().status == 0) &&
                 (msg->status_list[msg->status_list.size()-2].status == 3)) // PREEMTED
            eef_values.set_right_goal_status(2);
    } else
        eef_values.set_right_goal_status(0);

}

void execute_traj_Callback(const baxter_kinematics::TrajectoryTopic::ConstPtr& topic_message,
                          ros::NodeHandle& nh,
                          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                          actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right,
                          ros::Publisher& traj_res_pub,
                          ros::ServiceClient& gripper_client,
                          Kinematic_values& eef_values){
    // Record start time
    auto start = std::chrono::high_resolution_clock::now();

    // Required to trigger previous callbacks
    ros::AsyncSpinner spinner (1);
    spinner.start();

    // read message
    std::string eef_name = topic_message->eef_name;
    bool feedback = topic_message->feedback;
    std::vector<double> received_traj_vector = topic_message->trajectory;
    std::vector<std::string> gripper_values_vector = topic_message->gripper_values;

    bool success;
    int curr_iter = 0;
    bool found = false;
    while (!found && curr_iter < 5){
        ROS_INFO("Load robot description");
        // set arm
        std::string left_arm = "left_arm";
        std::string right_arm = "right_arm";
        if(strcmp(eef_name.c_str(), "left") == 0)
            eef_values.set_baxter_arm(left_arm);
        else if(strcmp(eef_name.c_str(), "right") == 0)
            eef_values.set_baxter_arm(right_arm);
        else{
            ROS_ERROR("please specify in message request, left or right arm");
        }

        // get initial pose orientation
        std::string gripper;
        if(strcmp(eef_name.c_str(), "left") == 0)
            gripper = "left_gripper";
        else if(strcmp(eef_name.c_str(), "right") == 0)
            gripper = "right_gripper";
        else{
            ROS_ERROR("please specify in message request, left or right arm");
        }
        geometry_msgs::Pose start_pose = eef_values.get_eef_pose(gripper);
        ROS_ERROR_STREAM("eef orientation is: " << start_pose);

        // get trajectory
        std::vector<geometry_msgs::Pose> waypoints;
        for (std::size_t i=0; i<received_traj_vector.size(); i=i+3){
            start_pose.position.x = received_traj_vector[i];
            start_pose.position.y = received_traj_vector[i+1];
            start_pose.position.z = received_traj_vector[i+2];
            waypoints.push_back(start_pose);
            ROS_ERROR_STREAM(start_pose.position.x << " " << start_pose.position.y << " " << start_pose.position.z);
        }
//        usleep(1e4);

        // move arms
        auto exec_start = std::chrono::high_resolution_clock::now();

        std::vector<std::string> gripper_values_vector = topic_message->gripper_values;
        int traj_res;
        if(strcmp(eef_name.c_str(), "left") == 0)
            traj_res = plan_and_execute_waypoint_traj("left",
                                                      waypoints,
                                                      ac_left,
                                                      eef_values,
                                                      nh,
                                                      false, // force_orien
                                                      feedback,
                                                      gripper_client,
                                                      gripper_values_vector,
                                                      traj_res_pub);
        else if(strcmp(eef_name.c_str(), "right") == 0)
            traj_res = plan_and_execute_waypoint_traj("right",
                                                      waypoints,
                                                      ac_right,
                                                      eef_values,
                                                      nh,
                                                      false, // force_orien
                                                      feedback,
                                                      gripper_client,
                                                      gripper_values_vector,
                                                      traj_res_pub);
        else{
            ROS_ERROR("please specify in message request, left or right arm");
        }
        if (curr_iter == 3 or traj_res == 0)
            success = false;
        else if (traj_res == 1){
            success = true;
            found = true;
        }
        else if (traj_res == 2)
            ROS_ERROR_STREAM("plan_and_execute_waypoint_traj - try: " << curr_iter);
        curr_iter++;

        auto exec_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> exec_elapsed = exec_finish - exec_start;
        ROS_ERROR_STREAM("Execution elapsed time: " << exec_elapsed.count() << " seconds");
    }

    ROS_INFO_STREAM("Trajectory execution success : " << success);
    ROS_INFO("Done execute_traj_Callback!\n");
    // Record end time
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ROS_ERROR_STREAM("Total elapsed time: " << elapsed.count() << " seconds");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_trajectory_topic_node");

    ros::NodeHandle nh;
    Kinematic_values eef_values;

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10,
                                                                                  boost::bind(left_eef_Callback, _1, boost::ref(eef_values)));
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10,
                                                                                  boost::bind(right_eef_Callback, _1, boost::ref(eef_values)));
    std::string obj_pos_topic_name;
    bool real_robot;
    nh.getParam("real_robot", real_robot);
    if (real_robot)
        obj_pos_topic_name = "/visual/obj_pos_vector";
    ros::Subscriber sub_obj_state = nh.subscribe<pcl_tracking::ObjectPosition>(obj_pos_topic_name, 1,
                                                                               boost::bind(obj_state_cloud_Callback, _1, boost::ref(eef_values)));

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_r("/robot/limb/right/follow_joint_trajectory", true);
    ros::Subscriber sub_goal_feedback_l =
            nh.subscribe<actionlib_msgs::GoalStatusArray>("/robot/limb/left/follow_joint_trajectory/status", 1,
                                                                            boost::bind(left_goal_Callback, _1, boost::ref(eef_values)));
    ros::Subscriber sub_goal_feedback_r =
            nh.subscribe<actionlib_msgs::GoalStatusArray>("/robot/limb/right/follow_joint_trajectory/status", 1,
                                                                            boost::bind(right_goal_Callback, _1, boost::ref(eef_values)));


    ros::ServiceClient gripper_client = nh.serviceClient<baxter_kinematics::GripperAction>("/baxter_kinematics/gripper_action");
    std::string feedback_topic_name;
    nh.getParam("feedback_topic", feedback_topic_name);
    ros::Publisher traj_res_pub = nh.advertise<std_msgs::Float64MultiArray>(feedback_topic_name, 1);

    std::string execute_topic_name;
    nh.getParam("execute_traj_topic", execute_topic_name);
    ros::Subscriber traj_exec_msg = nh.subscribe<baxter_kinematics::TrajectoryTopic>(execute_topic_name, 1000,
                                                                               boost::bind(execute_traj_Callback, _1,
                                                                                           boost::ref(nh),
                                                                                           boost::ref(ac_l),
                                                                                           boost::ref(ac_r),
                                                                                           boost::ref(traj_res_pub),
                                                                                           boost::ref(gripper_client),
                                                                                           boost::ref(eef_values)));
    ROS_INFO("Ready to execute trajectory (topic).");
    ros::spin();

    return 0;
}
