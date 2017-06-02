#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "baxter_kinematics/MoveToPos.h"

Kinematic_values eef_values;

//call back that register baxter left end effector pose and rearrange the orientation in RPY
void left_eef_Callback(baxter_core_msgs::EndpointState l_eef_feedback){
    locate_eef_pose(l_eef_feedback.pose, eef_values, "left_gripper");
}

void right_eef_Callback(baxter_core_msgs::EndpointState r_eef_feedback){
    locate_eef_pose(r_eef_feedback.pose, eef_values, "right_gripper");
}

bool move_to_pos(baxter_kinematics::MoveToPos::Request &req,
                      baxter_kinematics::MoveToPos::Response &res,
                      ros::NodeHandle& nh,
                      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right){

    ROS_INFO("Establish communication tools");

    ros::Subscriber sub_l_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/left/endpoint_state", 10, left_eef_Callback);
    ros::Subscriber sub_r_eef_msg = nh.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state", 10, right_eef_Callback);

    // Required for communication with moveit components
    ros::AsyncSpinner spinner (1);
    spinner.start();

    int curr_iter = 0;
    bool found = false;
    int max_nb_tries = 3;
    while (!found and curr_iter < max_nb_tries){

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

        // Get init pos
        std::vector<Eigen::Vector3d> mid_point_position_vector(4);
        Eigen::Vector3d init_pos;
        init_pos << req.x,
                    req.y,
                    req.z;

        std::string temp_side = "no_side"; //eef_values.get_cube_side_value(4); //NO_SIDE

        std::vector<geometry_msgs::Pose> waypoints =
            compute_directed_waypoints(true,
                                       init_pos, // goal
                                       temp_side,
                                       mid_point_position_vector,
                                       eef_values);

        int traj_res;
        if(strcmp(req.eef_name.c_str(), "left") == 0)
            traj_res = plan_and_execute_waypoint_traj("left",
                                           waypoints,
                                           ac_left,
                                           eef_values,
                                           nh,
                                           req.force_orien);
        else if(strcmp(req.eef_name.c_str(), "right") == 0)
            traj_res = plan_and_execute_waypoint_traj("right",
                                           waypoints,
                                           ac_right,
                                           eef_values,
                                           nh,
                                           req.force_orien);

        if (curr_iter == max_nb_tries or traj_res == 0)
            res.success = false;
        else if (traj_res == 1){
            res.success = true;
            found = true;
        }
        else if (traj_res == 2)
            ROS_ERROR_STREAM("plan_and_execute_waypoint_traj - try: " << curr_iter);
        curr_iter++;
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
