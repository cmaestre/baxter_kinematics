#include <ros/ros.h>

#include "../../include/baxter_kinematics/lib_movement.hpp"

void left_eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr&  l_eef_feedback,
                       Kinematic_values& eef_values){
        locate_eef_pose(l_eef_feedback->pose, eef_values, "left_gripper");
    }

void right_eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr&  r_eef_feedback,
                        Kinematic_values& eef_values){
        locate_eef_pose(r_eef_feedback->pose, eef_values, "right_gripper");
    }

void joint_states_Callback(const sensor_msgs::JointState::ConstPtr& jo_state,
                           Kinematic_values& eef_values){
        ROS_INFO_STREAM("saving joint state with lenght : " << jo_state->position.size());
        if(jo_state->position.size() > 7)
            eef_values.set_joint_state(jo_state);
    }

int main(int argc, char **argv)
    {
        ros::init (argc, argv, "testing_trajectories_batching_node");
        ros::NodeHandle nh;

        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_l("/robot/limb/left/follow_joint_trajectory", true);

        Kinematic_values eef_values;

        std::string left_arm = "left_arm";
        eef_values.set_baxter_arm(left_arm);
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10,
                                                                                      boost::bind(left_eef_Callback, _1, boost::ref(eef_values)));
        ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10,
                                                                                      boost::bind(right_eef_Callback, _1, boost::ref(eef_values)));
//        ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10,
//                                                                                boost::bind(joint_states_Callback, _1, boost::ref(eef_values)));

        if (!ac_l.waitForServer(ros::Duration(2.0)))
            {
                ROS_ERROR("Could not connect to action server");
                return 0;
            }

        std::ofstream trajectories_file;
        trajectories_file.open("trajectories_speeds_feedback.csv", std::ofstream::out);
        std::map<int, std::vector<std::vector<double>>> complete_trajectories_speed;
        std::vector<double> q_1, q_2, q_3, q_4, q_5, q_6, q_7;
        //eef_values.get_baxter_mover_class().reset(new baxter_mover::BAXTER_Mover(nh));
        double x = 0.65;
        double y = 0.6;
        double z = 0.25;
        std::vector<geometry_msgs::Pose> waypoints;


        geometry_msgs::Pose first_point;
        first_point.position.x = x;
        first_point.position.y = y;
        first_point.position.z = z;
        waypoints.push_back(first_point);
        //ros::Rate rate(100);
        bool first = true;
        int iteration = 1;
        while(ros::ok() && first_point.position.y > 0.0){
                //while(ros::ok() && iteration < 3){
                //eef_values.get_get
                ROS_WARN("ANOTHER NEW TRAJECTORY!!!");
                eef_values.get_current_trajectory_speed().clear();
                eef_values.set_execute_new_trajectory(false);
                if(first){
                        first = false;
                        plan_and_execute_waypoint_traj("left", waypoints, ac_l, eef_values, nh, true, false);
                    }
                else{
                        waypoints.clear();
                        waypoints.push_back(first_point);
                        first_point.position.y -= 0.03;
                        waypoints.push_back(first_point);
                        plan_and_execute_waypoint_traj("left", waypoints, ac_l, eef_values, nh, true, true);
//                        complete_trajectories_speed.emplace(iteration, eef_values.get_current_trajectory_speed());
                        complete_trajectories_speed.insert(std::make_pair(iteration, eef_values.get_current_trajectory_speed()));
                        iteration++;
                    }
                ros::spinOnce();


                //rate.sleep();
            }

//        ROS_INFO_STREAM("SO Whole trajectories number is : " << iteration);
//        ROS_INFO_STREAM("Size of trajectories vector is : " << complete_trajectories_speed.size());
        ROS_INFO("**********************  PRINTING TIME  *********************");
//        //for(int j = 0; j < 7; j++){
                int i = 0;
                std::map<int, std::vector<std::vector<double>>>::iterator it = complete_trajectories_speed.begin();
                while(it != complete_trajectories_speed.end()){
                        ROS_INFO_STREAM("Size of trajectory no : " << i << " is : " << it->second.size());
                        for(size_t k = 0; k < it->second.size(); k++)
                            ROS_INFO_STREAM("Size of vector : " << k << " in trajectory : " << i << " is : " << it->second[k].size());
                        ROS_WARN("---------------------------------------------------------");
                        it++;
                        i++;
                    }
//            //}
        return 0;
    }
