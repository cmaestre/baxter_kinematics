#include "../../include/baxter_kinematics/lib_movement.hpp"
#include "environment_functionalities/GetObjectState.h"

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
void jocommCallback_sim(const sensor_msgs::JointState::ConstPtr& jo_state,
                        std::vector<double> &left_arm_joint_values,
                        std::vector<double> &right_arm_joint_values)
{
    //ROS_WARN_STREAM("I am at simulated robot because params is: ");

    left_arm_joint_values[0] = jo_state->position[5];
    left_arm_joint_values[1] = jo_state->position[6];
    left_arm_joint_values[2] = jo_state->position[3];
    left_arm_joint_values[3] = jo_state->position[4];
    left_arm_joint_values[4] = jo_state->position[7];
    left_arm_joint_values[5] = jo_state->position[8];
    left_arm_joint_values[6] = jo_state->position[9];

    right_arm_joint_values[0] = jo_state->position[14];
    right_arm_joint_values[1] = jo_state->position[15];
    right_arm_joint_values[2] = jo_state->position[12];
    right_arm_joint_values[3] = jo_state->position[13];
    right_arm_joint_values[4] = jo_state->position[16];
    right_arm_joint_values[5] = jo_state->position[17];
    right_arm_joint_values[6] = jo_state->position[18];
}

/**
 * @brief jocommCallback: call back to get feedback about joints angles
 * @param the topic msgs about joints states
**/
void jocommCallback_real(const sensor_msgs::JointState::ConstPtr& jo_state,
                         std::vector<double> &left_arm_joint_values,
                         std::vector<double> &right_arm_joint_values,
                         std::vector<std::string> &all_joint_names,
                         std::vector<double> &all_joint_values)
{
    //ROS_WARN_STREAM("I am at real robot because params is: ");
    if(jo_state->position.size() > 9){
        all_joint_names = jo_state->name;
        all_joint_values = jo_state->position;

        left_arm_joint_values[0] = jo_state->position[4];
        left_arm_joint_values[1] = jo_state->position[5];
        left_arm_joint_values[2] = jo_state->position[2];
        left_arm_joint_values[3] = jo_state->position[3];
        left_arm_joint_values[4] = jo_state->position[6];
        left_arm_joint_values[5] = jo_state->position[7];
        left_arm_joint_values[6] = jo_state->position[8];

        right_arm_joint_values[0] = jo_state->position[11];
        right_arm_joint_values[1] = jo_state->position[12];
        right_arm_joint_values[2] = jo_state->position[9];
        right_arm_joint_values[3] = jo_state->position[10];
        right_arm_joint_values[4] = jo_state->position[13];
        right_arm_joint_values[5] = jo_state->position[14];
        right_arm_joint_values[6] = jo_state->position[15];
    }
}

//return roll pitch yaw from the transformation matrix transform_l_ee_w
Eigen::Vector3d extract_angles(Eigen::Matrix4d& transform_l_ee_w){
    Eigen::Vector3d my_angles;
    double Roll, Pitch, Yaw;
    Roll = atan2(transform_l_ee_w(1, 0), transform_l_ee_w(0, 0));
    Pitch = atan2(-transform_l_ee_w(2, 0), cos(Roll) * transform_l_ee_w(0, 0) + sin(Roll) * transform_l_ee_w(1, 0));
    Yaw = atan2(sin(Roll) * transform_l_ee_w(0, 2) - cos(Roll) * transform_l_ee_w(1, 2), -sin(Roll) * transform_l_ee_w(0, 1) + cos(Roll) * transform_l_ee_w(1, 1));
    my_angles << Roll, Pitch, Yaw;
    return my_angles;
}

//find nearest vector from my_points to my_goal and return the index of this vector
int find_nearest(std::vector<Eigen::Vector3d>& my_points,
                 Eigen::Vector3d& my_goal){
    double distance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d distance_vector;
    int index_nearest;
    for (unsigned int i = 0; i < my_points.size(); ++i){
        distance_vector = my_goal - my_points[i];
        if (distance_vector.norm() < distance){
            distance = distance_vector.norm();
            index_nearest = i;
        }
    }
    return index_nearest;
}

//find farthest vector from my_points to my_goal and return the index of this vector
int find_farthest(std::vector<Eigen::Vector3d>& my_points,
                  Eigen::Vector3d& my_goal){
    double distance = 0;
    Eigen::Vector3d distance_vector;
    int index_farthest;
    for (unsigned int i = 0; i < my_points.size(); ++i){
        distance_vector = my_goal - my_points[i];
        if (distance_vector.norm() > distance){
            distance = distance_vector.norm();
            index_farthest = i;
        }
    }
    return index_farthest;
}

//get a baxter eef pose "either the right one or the left one"
void locate_eef_pose(geometry_msgs::Pose &eef_feedback, Kinematic_values& eef_values, const std::string gripper){
    Eigen::VectorXd end_effector_pose(6);
    geometry_msgs::Pose eef_pose_quat = eef_feedback;
    tf::Quaternion eef_rpy_orientation;

    tf::quaternionMsgToTF(eef_pose_quat.orientation, eef_rpy_orientation);

    double roll, yaw, pitch;
    tf::Matrix3x3 m(eef_rpy_orientation);
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d eef_current_position;
    Eigen::Vector3d eef_current_orientation;
    eef_current_position << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z;

    eef_current_orientation <<    roll,
            pitch,
            yaw;
    end_effector_pose << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z,
            roll,
            pitch,
            yaw;
    eef_values.set_eef_position(eef_current_position, gripper);
    eef_values.set_eef_rpy_orientation(eef_current_orientation, gripper);
    eef_values.set_eef_pose(eef_pose_quat, gripper);
    eef_values.set_eef_rpy_pose(end_effector_pose, gripper);
    //    ROS_ERROR_STREAM("locating eef stuff gave for position: " << eef_values.get_eef_position(gripper));// eef_current_position); // << "\n and for orientation: " << eef_current_angles);
}

/**
 * @brief return eef position and orientation
 * @param
 * @return
 */
std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_eef_pose(
        robot_state::RobotState my_robot_state,
        std::string arm_name,
        std::vector<double> arm_joint_values){
    typedef std::tuple<Eigen::Vector3d, Eigen::Vector3d> res;
    std::vector <std::string> variable_names(7);
    Eigen::Affine3d my_pose;
    if (strcmp(arm_name.c_str(), "left") == 0){
        variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
        variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
        variable_names[6] = "left_w2";
        my_robot_state.setVariablePositions(variable_names, arm_joint_values);
        my_pose = my_robot_state.getGlobalLinkTransform("left_gripper");
    } else if (strcmp(arm_name.c_str(), "right") == 0){
        variable_names[0] = "right_s0"; variable_names[1] = "right_s1"; variable_names[2] = "right_e0";
        variable_names[3] = "right_e1"; variable_names[4] = "right_w0"; variable_names[5] = "right_w1";
        variable_names[6] = "right_w2";
        my_robot_state.setVariablePositions(variable_names, arm_joint_values);
        my_pose = my_robot_state.getGlobalLinkTransform("right_gripper");
    }
    Eigen::Vector3d current_position = my_pose.translation();
    
    Eigen::Matrix4d transform_l_ee_w = my_pose.matrix();
    Eigen::Vector3d current_angles = extract_angles(transform_l_ee_w);

    return res(current_position, current_angles);
}

/** restart arms to initial safe position (before any movement)
 * @brief a
 * @param b
**/
bool restart_robot_initial_position(robot_state::RobotState robot_state,
                                    Kinematic_values& eef_values,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right,
                                    ros::NodeHandle nh,
                                    std::vector<double> &left_arm_joint_values,
                                    std::vector<double> &right_arm_joint_values,
                                    std::vector<double> &all_joint_values){

    // get current position
    std::vector<std::string> left_joint_names = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
    std::vector<std::string> right_joint_names = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
//    std::vector<std::string> all_joint_names;

//    bool real_robot;
//    nh.getParam("real_robot", real_robot);
//    if(real_robot)
//        robot_state.setVariablePositions(all_joint_names, all_joint_values);

    std::vector<Eigen::Vector3d> dummy;
    double tmp_x, tmp_y, tmp_z;
    std::vector<geometry_msgs::Pose> waypoints;
    //////////////////////////////////////////////////////////////////////////
    // Left arm
    ROS_INFO("\nLeft arm to initial pose");

//    if(!real_robot)
    robot_state.setVariablePositions(left_joint_names, left_arm_joint_values);

    Eigen::Vector3d left_eef_initial_pos;
    nh.getParam("left_eef/initial_pos/x", tmp_x);
    nh.getParam("left_eef/initial_pos/y", tmp_y);
    nh.getParam("left_eef/initial_pos/z", tmp_z);
    left_eef_initial_pos << tmp_x,
                            tmp_y,
                            tmp_z;

    std::string gripper = "left_gripper";
    geometry_msgs::Pose initial_pose = eef_values.get_eef_pose(gripper);
    initial_pose.position.x = left_eef_initial_pos[0];
    initial_pose.position.y = left_eef_initial_pos[1];
    initial_pose.position.z = left_eef_initial_pos[2];

    waypoints.clear();
    waypoints.push_back(initial_pose);
    ROS_ERROR_STREAM("Left eef initial position :" << initial_pose.position.x << " " << initial_pose.position.y << " " << initial_pose.position.z);

    bool left_res =
            plan_and_execute_waypoint_traj("left",
                                           waypoints,
                                           robot_state,
                                           ac_left,
                                           "",
                                           dummy,
                                           dummy,
                                           dummy,
                                           dummy,
                                           eef_values,
                                           nh);
    if (!left_res){
        ROS_ERROR_STREAM("restart_robot_initial_position : Failed executing left arm motion");
        return false;
    }

    ////////////////////////////////////////////////////////////////////////
    // Right arm
    ROS_INFO("\nRight arm to initial position");

//    if(!real_robot)
    robot_state.setVariablePositions(right_joint_names, right_arm_joint_values);

    Eigen::Vector3d right_eef_initial_pos;
    nh.getParam("right_eef/initial_pos/x", tmp_x);
    nh.getParam("right_eef/initial_pos/y", tmp_y);
    nh.getParam("right_eef/initial_pos/z", tmp_z);
    right_eef_initial_pos << tmp_x,
                             tmp_y,
                             tmp_z;

    gripper = "right_gripper";
    initial_pose = eef_values.get_eef_pose(gripper); // to get the rotation
    initial_pose.position.x = right_eef_initial_pos[0];
    initial_pose.position.y = right_eef_initial_pos[1];
    initial_pose.position.z = right_eef_initial_pos[2];
    waypoints.push_back(initial_pose);
    ROS_ERROR_STREAM("Right eef initial position : " << initial_pose.position.x << " " << initial_pose.position.y << " " << initial_pose.position.z);
    bool right_res =
            plan_and_execute_waypoint_traj("right",
                                           waypoints,
                                           robot_state,
                                           ac_right,
                                           "",
                                           dummy,
                                           dummy,
                                           dummy,
                                           dummy,
                                           eef_values,
                                           nh);

    if (!right_res){
        ROS_ERROR_STREAM("restart_robot_initial_position : Failed executing right arm motion");
        return false;
    }


    return true;
}

/**
 * @brief execute the descartes motion to a desired final position
 * @param desired position and ros handle
 * @param argv
 * @return
 */
bool plan_path_to_desired_position(robot_state::RobotState robot_state,
                                   Eigen::Vector3d goal,
                                   std::string my_side,
                                   std::vector<Eigen::Vector3d> mid_point_position_vector,
                                   std::vector<Eigen::Vector3d>& path,
                                   std::vector<Eigen::Vector3d>& final_path,
                                   Kinematic_values& eef_values){
    Eigen::Vector3d my_inter_points;

    std::string gripper;
    if(strcmp(eef_values.get_baxter_arm().c_str(), "left_arm") == 0)
        gripper = "left_gripper";
    else if(strcmp(eef_values.get_baxter_arm().c_str(), "right_arm") == 0)
        gripper = "right_gripper";
    //Eigen::Vector3d current_position;
    //geometry_msgs::Pose eef_pose = eef_values.get_eef_pose();
    //current_position << eef_pose.position.x,
    //            eef_pose.position.y,
    //            eef_pose.position.z;
    //current_position = eef_values.get_eef_position();
    //ROS_ERROR_STREAM("eef position is: " << eef_values.get_eef_position()(0) << " " << eef_values.get_eef_position()(1) << " " << eef_values.get_eef_position()(2));

    //create valid path
    // 2 segments are created in 'path'
    // each segments is divided into N points (6 for the first, 7 for the second)
    if (strcmp(my_side.c_str(), "no_side") != 0){
        path.clear();
        final_path.clear();
        path.push_back(eef_values.get_eef_position(gripper));

        //find nearest intermediate point to goal
        int index_nearest_goal;
        index_nearest_goal = find_nearest(mid_point_position_vector, goal);

        // check if the nearest intermediate to the correspondant side is the fasthest from the initial position
        // in this case, a small motion is created to avoid touching the box while moving to that intermediate position
        int index_farthest_goal;
        index_farthest_goal = find_farthest(mid_point_position_vector, eef_values.get_eef_position(gripper));

        if (index_nearest_goal == index_farthest_goal){
            Eigen::Vector3d over_current_side_pos;
            over_current_side_pos << goal[0],
                    goal[1],
                    eef_values.get_eef_position(gripper)(2);
            path.push_back(over_current_side_pos);
        }

        path.push_back(mid_point_position_vector[index_nearest_goal]);

        //add the final point which is the goal
        my_inter_points << goal(0), goal(1), goal(2);
        path.push_back(my_inter_points);

        //        std::vector<Eigen::Vector3d>::iterator iter;
        //        for (iter = path.begin(); iter != path.end(); iter++)
        //            ROS_ERROR_STREAM("the path points: " << (*iter));

        //start constructing the fine trajectory
        final_path.push_back({path[0]});
        for (unsigned int k = 0; k < path.size() - 1; k++){
            for (unsigned int j = 0; j < 6; j++){
                Eigen::Vector3d apoint;
                apoint << path[k](0) + 0.2*j*(path[k + 1](0) - path[k](0)),
                          path[k](1) + 0.2*j*(path[k + 1](1) - path[k](1)),
                          path[k](2) + 0.2*j*(path[k + 1](2) - path[k](2));

                /*apoint << path[k](0) + 0.3*j*(path[k + 1](0) - path[k](0)),
                              path[k](1) + 0.3*j*(path[k + 1](1) - path[k](1)),
                              path[k](2) + 0.2*j*(path[k + 1](2) - path[k](2));*/
                final_path.push_back(apoint);

                double d_x = 0.2*(path[k + 1](0) - path[k](0));
                double d_y = 0.2*(path[k + 1](1) - path[k](1));
                double d_z = 0.2*(path[k + 1](2) - path[k](2));

                if (k == path.size() - 2 && j == 5){
                    apoint(0) = apoint(0) + d_x*1;
                    apoint(1) = apoint(1) + d_y*1;
                    apoint(2) = apoint(2) + d_z*1;
                    final_path.push_back(apoint);
                }
            }
        }
        //        for (iter = final_path.begin(); iter != final_path.end(); iter++)
        //            ROS_ERROR_STREAM((*iter));
    }
    //the arm is going to home position so just go in reverse
    else {
        path.clear();
        if ((eef_values.get_eef_position(gripper) - goal).norm() < 0.01)
            return true;
        path.push_back(eef_values.get_eef_position(gripper));
        Eigen::Vector3d over_eef_position = eef_values.get_eef_position(gripper);
        // if(!eef_values.get_keep_height() && eef_values.get_eef_position(gripper)(2) < 0.0)
        //     over_eef_position[2] += 0.15;
        path.push_back(over_eef_position);
        path.push_back(goal);
    }
    //if it reaches here it means everything went well
    return true;
}

/**
 * @brief a
 * @param b
**/
//trajectory_msgs::JointTrajectory
void path_to_safe_pos(trajectory_msgs::JointTrajectory &trajectory,
                      ros::ServiceClient baxter_right_ik_solver,
                      Eigen::Vector3d safe_pose,
                      Eigen::Vector3d rotation,
                      std::vector<double> right_arm_joint_values){
    tf::Quaternion my_orientation;
    my_orientation.setRPY(rotation(0),
                          rotation(1),
                          rotation(2));
    geometry_msgs::PoseStamped my_desired_pose;
    my_desired_pose.header.frame_id = "/base";

    my_desired_pose.pose.position.x = safe_pose(0);
    my_desired_pose.pose.position.y = safe_pose(1);
    my_desired_pose.pose.position.z = safe_pose(2);

    my_desired_pose.pose.orientation.w = my_orientation.getW();
    my_desired_pose.pose.orientation.x = my_orientation.getX();
    my_desired_pose.pose.orientation.y = my_orientation.getY();
    my_desired_pose.pose.orientation.z = my_orientation.getZ();

    //solve for desired position
    baxter_core_msgs::SolvePositionIK::Request req;
    baxter_core_msgs::SolvePositionIK::Response res;
    req.pose_stamp.push_back(my_desired_pose);
    baxter_right_ik_solver.call(req, res);
    if(!res.isValid[0])
        ROS_ERROR("no solution found");

    //create the sequence of joint trajectory points
    trajectory.joint_names = res.joints[0].name;

    trajectory_msgs::JointTrajectoryPoint first_pt;
    first_pt.positions = right_arm_joint_values;
    first_pt.velocities.resize(7, 0.0);
    first_pt.accelerations.resize(7, 0.0);
    first_pt.effort.resize(7, 0.0);
    //first_pt.time_from_start = ros::Duration(0.5);
    trajectory.points.push_back(first_pt);

    trajectory_msgs::JointTrajectoryPoint second_pt;
    second_pt.positions = res.joints[0].position;
    second_pt.velocities.resize(7, 0.0);
    second_pt.accelerations.resize(7, 0.0);
    second_pt.effort.resize(7, 0.0);
    //second_pt.time_from_start = ros::Duration(5);
    trajectory.points.push_back(second_pt);
}


/**
 * @brief largest_difference
 * @param first
 * @param second
 * @return
 */
double largest_difference(std::vector<double> &first, std::vector<double> &second){
    Eigen::VectorXd difference(first.size());
    double my_max = 0;
    for(size_t j = 0; j < first.size(); ++j)
        difference(j) = fabs(first[j] - second[j]);
    for(size_t j = 0; j < first.size(); ++j){
        if(difference(j) > my_max)
            my_max = difference(j);
    }
    return my_max;
}

/**
 * @brief Given a directed trajectory, compute the related waypoints
 * @param b
**/
std::vector<geometry_msgs::Pose> compute_directed_waypoints(bool initialize_setup,
                                                            robot_state::RobotState robot_state,
                                                            Eigen::Vector3d goal_position,
                                                            std::string cube_side,
                                                            std::vector<Eigen::Vector3d> predefined_wp_trajectory,
                                                            Kinematic_values eef_values){
    //the move group to move the left arm to interact with the object as well as to derive it back home
    //waypoint of cartesian poses that will be used with the move_group to calculate the cartesian motion
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose_holder;
    std::string gripper;
    if(strcmp(eef_values.get_baxter_arm().c_str(), "left_arm") == 0)
        gripper = "left_gripper";
    else if(strcmp(eef_values.get_baxter_arm().c_str(), "right_arm") == 0)
        gripper = "right_gripper";
    //save the first pose which is the curren pose of "left_gripper"
    pose_holder = eef_values.get_eef_pose(gripper);
    waypoints.push_back(eef_values.get_eef_pose(gripper));

    std::vector<Eigen::Vector3d> path, final_path, current_path;
    std::vector<double> test_first(3), test_second(3);
    //get the cartesian path
    while(!plan_path_to_desired_position(robot_state,
                                         goal_position,
                                         cube_side,
                                         predefined_wp_trajectory,
                                         path,
                                         final_path,
                                         eef_values));
    if (initialize_setup)
        current_path = path; // intial_point and goal_point (Cartesian)
    else
        current_path = final_path; // set of Cartesian waypoints from initial to cube_side

    //in the current path there are some points that are doupled so fill the waypoint vector with only unique values
    for(size_t i = 0; i < current_path.size(); ++i){
        bool same_point = false;
        pose_holder.position.x = current_path[i](0);
        pose_holder.position.y = current_path[i](1);
        pose_holder.position.z = current_path[i](2);
        test_first[0] = current_path[i](0);
        test_first[1] = current_path[i](1);
        test_first[2] = current_path[i](2);
        //make sure this point is unique
        for(size_t j = 0; j < waypoints.size(); ++j){
            test_second[0] = waypoints[j].position.x;
            test_second[1] = waypoints[j].position.y;
            test_second[2] = waypoints[j].position.z;
            if(largest_difference(test_first, test_second) < 0.0001)
                same_point = true;
        }
        if(!same_point)
            waypoints.push_back(pose_holder);
    }



    return waypoints;
}

//get rid of repeqted waypoints in a trajectory
bool optimize_trajectory(std::vector<geometry_msgs::Pose>& vector_to_optimize,
                         double min_wp_dist){

    std::vector<geometry_msgs::Pose> updated_vector;

    std::size_t pos = 0;
    while (pos < vector_to_optimize.size()-1){
        geometry_msgs::Pose curr_pose = vector_to_optimize[pos];
        updated_vector.push_back(curr_pose);
        std::vector<double> current_wp = {curr_pose.position.x,
                                          curr_pose.position.y,
                                          curr_pose.position.z};
        bool far_found = false;
        std::size_t tmp_pos = pos + 1;
        //        ROS_ERROR_STREAM(pos << " " << tmp_pos);
        while (!far_found && (tmp_pos < vector_to_optimize.size())){
            geometry_msgs::Pose next_pose = vector_to_optimize[tmp_pos];
            std::vector<double> next_wp = {next_pose.position.x,
                                           next_pose.position.y,
                                           next_pose.position.z};
            if(largest_difference(current_wp, next_wp) >= min_wp_dist) {
                far_found = true;
            } else {
                tmp_pos++;
            }
            pos = tmp_pos;
        }
    }
    updated_vector.push_back(vector_to_optimize.back());
    vector_to_optimize = updated_vector;

    if (vector_to_optimize.size() == 1)
        return false;
    else
        return true;
}

/**
 * @brief Plan and execute waypoint trajectory
 * @param b
**/
//trajectory_msgs::JointTrajectory plan_trajectory(
int plan_and_execute_waypoint_traj(std::string selected_eef,
                                   std::vector<geometry_msgs::Pose> waypoints,
                                   robot_state::RobotState robot_state,
                                   actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                                   std::string object_name,
                                   std::vector<Eigen::Vector3d>& eef_position_vector,
                                   std::vector<Eigen::Vector3d>& eef_orientation_vector,
                                   std::vector<Eigen::Vector3d>& object_position_vector,
                                   std::vector<Eigen::Vector3d>& object_orientation_vector,
                                   Kinematic_values& eef_values,
                                   ros::NodeHandle nh,
                                   bool feedback_data,
                                   bool publish_topic,
                                   int feedback_frequency,
                                   ros::Publisher traj_res_pub){

    if (robot_state.getGlobalLinkTransform("left_gripper").translation()[0] < 0){
        ROS_ERROR_STREAM("Robot state failed !!!!");
        return 0;
    }

    // remove almost similar wps
    if (waypoints.size() > 1){
        ROS_ERROR_STREAM("nb waypoints before optimization is " << waypoints.size());
        double min_wp_dist;
        nh.getParam("min_wp_distance", min_wp_dist);
        if (!optimize_trajectory(waypoints, min_wp_dist)){
            ROS_ERROR_STREAM("waypoints too close! No execution.");
            return true;
        }
        ROS_ERROR_STREAM("nb waypoints after optimization is " << waypoints.size());
    }

    //the move group to move the selected arm to interact with the object as well as to derive it back home
    std::string arm_selected, eef_selected;
    if (strcmp(selected_eef.c_str(), "right") == 0){
        arm_selected = "right_arm";
        eef_selected = "right_gripper";
    }
    else{
        arm_selected = "left_arm";
        eef_selected = "left_gripper";
    }

    moveit::planning_interface::MoveGroup group(arm_selected);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setStartState(robot_state);

    //define a quaterion that will be used to look for valid orientation to return a complete path
    tf::Quaternion orientation;
    double roll, pitch, yaw, max_ang = M_PI, min_ang = -M_PI;
    roll = eef_values.get_eef_rpy_orientation(eef_selected)(0);
    pitch = eef_values.get_eef_rpy_orientation(eef_selected)(1);
    yaw = eef_values.get_eef_rpy_orientation(eef_selected)(2);

    geometry_msgs::Pose correct_orientation = eef_values.get_eef_pose(eef_selected);
    correct_orientation.orientation.w = 0.0;
    correct_orientation.orientation.x = 0.0;
    correct_orientation.orientation.y = 1;
    correct_orientation.orientation.z = 0.0;

    std::vector<geometry_msgs::Pose>::iterator wp_itr;
    for(wp_itr = waypoints.begin(); wp_itr != waypoints.end(); wp_itr++)
        wp_itr->orientation = correct_orientation.orientation;

    std::vector<geometry_msgs::Pose>::iterator waypoint_iter;
    for(waypoint_iter = waypoints.begin(); waypoint_iter != waypoints.end(); waypoint_iter++){
        ROS_ERROR_STREAM("desired position: " << (*waypoint_iter).position.x << ", " << (*waypoint_iter).position.y << ", " << (*waypoint_iter).position.z);
        ROS_ERROR_STREAM("desired orientation: " << (*waypoint_iter).orientation.w << ", " << (*waypoint_iter).orientation.x << ", " << (*waypoint_iter).orientation.y << ", " << (*waypoint_iter).orientation.z);
    }

    moveit_msgs::RobotTrajectory robot_trajectory;
    double fraction = group.computeCartesianPath(waypoints, 0.025, 0.0, robot_trajectory);
    ROS_WARN_STREAM("fraction solved of desired path in this trial is: " <<
                    fraction);  //eef_jump_step size determine the speed of resulted motion
    //    if(fraction < 1.0 && waypoints.size() <= 3){
    //        moveit::planning_interface::MoveGroup::Plan my_plan;
    //        geometry_msgs::PoseStamped init_pose;
    //        init_pose.header.frame_id = "/base";
    //        init_pose.pose = waypoints.back();
    //        group.setPoseTarget(init_pose);
    //        bool plan_success = group.plan(my_plan);
    //        if(plan_success)
    //            group.execute(my_plan);
    //        else
    //            ROS_ERROR_STREAM("Trying to move to initial position for left arm but group planning result is: " << plan_success);
    //        // if wrong orientation move to neutral position and come back
    //    }
    //this part is to look for an orientation that will produce 100% path, we change only pitch angle because in this setup it is the main angle that will give the required behavior
    int pitch_counter = 0, sign = 1;
    double step = 0.1;
    int trials = 0;
    //    while(fraction < 1.0 && waypoints.size() > 3 && trials < 200){
    while(fraction < 1.0 && trials < 50){
        ROS_WARN_STREAM("fraction is: " << fraction << " looking for orientations that will return complete path");
        pitch = pitch + step * pitch_counter;
        if (pitch > max_ang && sign > 0){
            pitch_counter = 0;
            sign = -1;
        }
        if (pitch < min_ang && sign < 0)
            break;
        //yaw = 0.0;
        orientation.setRPY(roll, pitch, yaw);
        std::vector<geometry_msgs::Pose>::iterator wp_itr;
        //        for(wp_itr = waypoints.begin(); wp_itr != waypoints.end() - 3; wp_itr++){
        for(wp_itr = waypoints.begin(); wp_itr != waypoints.end(); wp_itr++){
            wp_itr->orientation.w = orientation.w();
            wp_itr->orientation.x = orientation.x();
            wp_itr->orientation.y = orientation.y();
            wp_itr->orientation.z = orientation.z();
        }
        fraction = group.computeCartesianPath(waypoints, 0.025, 0.0, robot_trajectory);
        pitch_counter += 1;
        trials += 1;
    }

    if (trials == 50){
        ROS_ERROR_STREAM("NOT solution found for trajectory");
        return 2;
    }

    if(trials > 0 && fraction == 1){
        ROS_ERROR_STREAM("z coordinate is: " << (waypoints[waypoints.size() - 1]).position.z);
        ROS_ERROR_STREAM("robot state pose: " << robot_state.getGlobalLinkTransform("left_gripper").translation());
    }

    if (!ac.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR("Could not connect to action server");
        return 0;
    }
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = robot_trajectory.joint_trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);
    ac.sendGoal(goal);
    Eigen::Vector3d eef_pose;
    std_msgs::Float64MultiArray real_traj_to_publish;
    real_traj_to_publish.data.clear();
    int added_waypoint = 0;
    if(!feedback_data)
        ROS_ERROR_STREAM("waypoints size is: " << waypoints.size());
    ROS_ERROR_STREAM("waypoints size is: " << waypoints.size());

    //    ros::ServiceClient client_get_object_pose = nh.serviceClient<environment_functionalities::GetObjectState>("/env/get_object_state");
    //    environment_functionalities::GetObjectState getObjectStateSrv;
    int count = 0;
    while(!ac.getState().isDone()){
        if(feedback_data && fraction == 1){
            //always get current eef pose
            eef_pose = eef_values.get_eef_position(eef_selected);
            std::vector<double> curr_eff_position(3), expected_traj_position(3);
            curr_eff_position[0] = eef_pose(0);
            curr_eff_position[1] = eef_pose(1);
            curr_eff_position[2] = eef_pose(2);
            nh.setParam("/stop_traj", false);
            for(size_t i = 0; i < waypoints.size(); ++i){
                expected_traj_position[0] = waypoints[i].position.x;
                expected_traj_position[1] = waypoints[i].position.y;
                expected_traj_position[2] = waypoints[i].position.z;
                if(largest_difference(curr_eff_position, expected_traj_position) < 0.01){
                    //                    ROS_ERROR_STREAM("STORE/PUBLISH FEEDBACK");
                    waypoints.erase(waypoints.begin() + i);
                    count+=1;

                    //                    //save eef values
                    //                    eef_pose = eef_values.get_eef_position(eef_selected);
                    //                    eef_position_vector.push_back(eef_pose);
                    //                    eef_orientation_vector.push_back(eef_values.get_eef_rpy_orientation(eef_selected));

                    //                    //save object values
                    //                    getObjectStateSrv.request.object_name = object_name;
                    //                    client_get_object_pose.call(getObjectStateSrv);
                    //                    std::vector<double> object_state_vector = getObjectStateSrv.response.object_state;

                    //                    Eigen::Vector3d current_object_position;
                    //                    current_object_position <<  object_state_vector[0],
                    //                                                object_state_vector[1],
                    //                                                object_state_vector[2];
                    //                    object_position_vector.push_back(current_object_position);

                    //                    Eigen::Vector3d current_object_orientation;
                    //                    current_object_orientation << object_state_vector[3],
                    //                                                  object_state_vector[4],
                    //                                                  object_state_vector[5];
                    //                    object_orientation_vector.push_back(current_object_orientation);

                    //store to publish afterwards
                    real_traj_to_publish.data.push_back(eef_pose(0));
                    real_traj_to_publish.data.push_back(eef_pose(1));
                    real_traj_to_publish.data.push_back(eef_pose(2));
                    //                    real_traj_to_publish.data.push_back(object_state_vector[0]);
                    //                    real_traj_to_publish.data.push_back(object_state_vector[1]);
                    //                    real_traj_to_publish.data.push_back(object_state_vector[2]);

                    added_waypoint++;

                    if (publish_topic && (added_waypoint == feedback_frequency)){
                        added_waypoint = 0;
                        ROS_ERROR_STREAM("Printing in topic");
                        traj_res_pub.publish(real_traj_to_publish);
                        real_traj_to_publish.data.clear();
                    }
                }
            }
        }
    }

    if(feedback_data){

        //    // publish the remaining wp of the trajectory
        //    if (publish_topic && (real_traj_to_publish.data.size() > 0)) {
        //        ROS_ERROR_STREAM("Final printing in topic");
        //        traj_res_pub.publish(real_traj_to_publish);
        //    }

        // add last eef values
        eef_pose = eef_values.get_eef_position(eef_selected);
        eef_position_vector.push_back(eef_pose);
        eef_orientation_vector.push_back(eef_values.get_eef_rpy_orientation(eef_selected));

        //    //add last object values
        //    getObjectStateSrv.request.object_name = object_name;
        //    client_get_object_pose.call(getObjectStateSrv);
        //    std::vector<double> object_state_vector = getObjectStateSrv.response.object_state;

        //    Eigen::Vector3d current_object_position;
        //    current_object_position <<  object_state_vector[0],
        //                                object_state_vector[1],
        //                                object_state_vector[2];
        //    object_position_vector.push_back(current_object_position);

        //    Eigen::Vector3d current_object_orientation;
        //    current_object_orientation << object_state_vector[3],
        //                                  object_state_vector[4],
        //                                  object_state_vector[5];
        //    object_orientation_vector.push_back(current_object_orientation);

        //store to publish
        real_traj_to_publish.data.push_back(eef_pose(0));
        real_traj_to_publish.data.push_back(eef_pose(1));
        real_traj_to_publish.data.push_back(eef_pose(2));
        //    real_traj_to_publish.data.push_back(object_state_vector[0]);
        //    real_traj_to_publish.data.push_back(object_state_vector[1]);
        //    real_traj_to_publish.data.push_back(object_state_vector[2]);

        //    // publish full trajectory
        //    if (publish_topic && (real_traj_to_publish.data.size() > 0)) {
        //        ROS_ERROR_STREAM("Printing full traj in topic");
        //        traj_res_pub.publish(real_traj_to_publish);
        //    }
        // publish the remaining wp of the trajectory
        if (real_traj_to_publish.data.size() > 0)
            ROS_ERROR_STREAM("Final printing in topic");

        ROS_ERROR_STREAM("Number of saved coordinates is: " << count);
    }
    return 1;
}

bool move_object_close_to_robot(std::string selected_eef,
                                std::vector<geometry_msgs::Pose> waypoints,
                                moveit::core::RobotState robot_state,
                                Eigen::Vector3d goal_position,
                                std::string cube_side,
                                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                                Kinematic_values &eef_values){

    ROS_WARN_STREAM("trying to move the cube closer, the goal position is: " << goal_position(0) << ", "
                    << goal_position(1) << ", "
                    << goal_position(2) << " and cube side is: " << cube_side);
    std::string arm_selected, eef_selected;
    if (strcmp(selected_eef.c_str(), "right") == 0){
        arm_selected = "right_arm";
        eef_selected = "right_gripper";
    }
    else{
        arm_selected = "left_arm";
        eef_selected = "left_gripper";
    }

    tf::Quaternion orientation;
    double roll, pitch, yaw, max_ang = M_PI, min_ang = -M_PI;
    roll = eef_values.get_eef_rpy_orientation(eef_selected)(0);
    pitch = eef_values.get_eef_rpy_orientation(eef_selected)(1);
    yaw = eef_values.get_eef_rpy_orientation(eef_selected)(2);

    moveit::planning_interface::MoveGroup group(arm_selected);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setStartState(robot_state);
    std::vector<geometry_msgs::Pose> reduced_waypoint;

    //first point is the current pose of eef which the first point of the original failed waypoint
    reduced_waypoint.push_back(eef_values.get_eef_pose(eef_selected));
    reduced_waypoint.push_back(eef_values.get_eef_pose(eef_selected));
    reduced_waypoint[1].position.z = 0.4;

    //second point is a point near the current side of the box
    double dX = 0.07;
    geometry_msgs::Pose expected_traj_position;
    expected_traj_position.position.x = goal_position(0);
    expected_traj_position.position.y = goal_position(1);
    expected_traj_position.position.z = goal_position(2);
    expected_traj_position.orientation = eef_values.get_eef_pose(eef_selected).orientation;
    reduced_waypoint.push_back(expected_traj_position);
    if(strcmp(cube_side.c_str(), "down") == 0)
        reduced_waypoint[2].position.x = goal_position(0) + dX;
    else if(strcmp(cube_side.c_str(), "up") == 0)
        reduced_waypoint[2].position.x = goal_position(0) - dX;
    else if(strcmp(cube_side.c_str(), "left") == 0)
        reduced_waypoint[2].position.y = goal_position(1) + dX;
    else if(strcmp(cube_side.c_str(), "right") == 0)
        reduced_waypoint[2].position.y = goal_position(1) - dX;

    //third point is a point where x and y are changed as appropriate and z is the same
    reduced_waypoint.push_back(waypoints[waypoints.size() - 1]);
    if(strcmp(cube_side.c_str(), "up") == 0 || strcmp(cube_side.c_str(), "down") == 0)
        reduced_waypoint[3].position.x = 0.7;
    else if(strcmp(cube_side.c_str(), "left") == 0 || strcmp(cube_side.c_str(), "right") == 0)
        reduced_waypoint[3].position.y = 0.1;

    reduced_waypoint.push_back(reduced_waypoint[1]);

    std::vector<geometry_msgs::Pose>::iterator waypoint_iter;
    for(waypoint_iter = reduced_waypoint.begin(); waypoint_iter != reduced_waypoint.end(); waypoint_iter++){
        ROS_ERROR_STREAM("desired position: " << (*waypoint_iter).position.x << ", " << (*waypoint_iter).position.y << ", " << (*waypoint_iter).position.z);
        ROS_ERROR_STREAM("desired orientation: " << (*waypoint_iter).orientation.w << ", " << (*waypoint_iter).orientation.x << ", " << (*waypoint_iter).orientation.y << ", " << (*waypoint_iter).orientation.z);
    }

    moveit_msgs::RobotTrajectory robot_trajectory;
    double fraction = group.computeCartesianPath(reduced_waypoint, 0.025, 0.0, robot_trajectory);
    ROS_WARN_STREAM("fraction solved of desired path in this trial is: " <<
                    fraction);  //eef_jump_step size determine the speed of resulted motion
    //this part is to look for an orientation that will produce 100% path, we change only pitch angle because in this setup it is the main angle that will give the required behavior
    int pitch_counter = 0, sign = 1;
    double step = 0.1;
    int trials = 0;
    while(fraction < 1.0 && trials < 100){
        ROS_WARN_STREAM("fraction is: " << fraction << " looking for orientations that will return complete path");
        pitch = pitch + step * pitch_counter;
        if (pitch > max_ang && sign > 0){
            pitch_counter = 0;
            sign = -1;
        }
        if (pitch < min_ang && sign < 0)
            break;
        //yaw = 0.0;
        orientation.setRPY(roll, pitch, yaw);
        std::vector<geometry_msgs::Pose>::iterator wp_itr;
        for(wp_itr = reduced_waypoint.begin(); wp_itr != reduced_waypoint.end(); wp_itr++){
            wp_itr->orientation.w = orientation.w();
            wp_itr->orientation.x = orientation.x();
            wp_itr->orientation.y = orientation.y();
            wp_itr->orientation.z = orientation.z();
        }
        fraction = group.computeCartesianPath(reduced_waypoint, 0.025, 0.0, robot_trajectory);
        pitch_counter += 1;
        trials += 1;
    }



    if(fraction == 1){
        if (!ac.waitForServer(ros::Duration(2.0)))
        {
            ROS_ERROR("Could not connect to action server");
            return 0;
        }
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = robot_trajectory.joint_trajectory;
        goal.goal_time_tolerance = ros::Duration(1.0);
        ac.sendGoal(goal);
        if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(10)))
        {
            ROS_INFO("Action server reported successful execution");
            geometry_msgs::PoseStamped correct_orientation;
            correct_orientation.header.frame_id = "/base";
            correct_orientation.pose = eef_values.get_eef_pose(eef_selected);
            correct_orientation.pose.orientation.w = 0.02547;
            correct_orientation.pose.orientation.x = 0.140766;
            correct_orientation.pose.orientation.y = 0.9896;
            correct_orientation.pose.orientation.z = 0.0116586;
            moveit::planning_interface::MoveGroup::Plan my_plan;
            group.setPoseTarget(correct_orientation);
            if(group.plan(my_plan))
                group.execute(my_plan);
            return true;
        } else {
            ROS_WARN("Action server could not execute trajectory");
            return false;
        }

    }
    else
        return false;
}

/**
 * @brief Given a goal position, plan a trajectory and compute the related joint configuration
 * @param b
**/
//trajectory_msgs::JointTrajectory plan_trajectory(
bool plan_traj_to_goal_joint_config(Kinematic_values eef_values,
                                    bool initialize_setup,
                                    ros::ServiceClient &ik_solver,
                                    robot_state::RobotState robot_state,
                                    trajectory_msgs::JointTrajectory &trajectory,
                                    Eigen::Vector3d position,
                                    std::string cube_side,
                                    std::vector<Eigen::Vector3d> mid_point_position_vector,
                                    ros::NodeHandle nh,
                                    bool try_different_seed
                                    ){

    std::vector<Eigen::Vector3d> path, final_path, current_path;
    bool res_move = true;
    while(!plan_path_to_desired_position(robot_state,
                                         position,
                                         cube_side,
                                         mid_point_position_vector,
                                         path,
                                         final_path,
                                         eef_values));
    if (initialize_setup)
        current_path = path;
    else
        current_path = final_path;

    trajectory.joint_names.clear();
    trajectory.points.clear();
    if(ik_solver.exists()){
        geometry_msgs::PoseStamped my_desired_pose;
        tf::Quaternion my_orientation;
        // my_orientation.setRPY(eef_values.get_left_eef_initial_rot_x(),
        //                       eef_values.get_left_eef_initial_rot_y(),
        //                       eef_values.get_left_eef_initial_rot_z());

        double rot_x, rot_y, rot_z;
        nh.getParam("eef_rotation/x", rot_x);
        nh.getParam("eef_rotation/y", rot_y);
        nh.getParam("eef_rotation/z", rot_z);
        my_orientation.setRPY(rot_x, rot_y, rot_z);

        my_desired_pose.header.frame_id = "/base";
        my_desired_pose.pose.orientation.w = my_orientation.getW();
        my_desired_pose.pose.orientation.x = my_orientation.getX();
        my_desired_pose.pose.orientation.y = my_orientation.getY();
        my_desired_pose.pose.orientation.z = my_orientation.getZ();
        for(int i = 0; (unsigned)i < current_path.size(); ++i){
            my_desired_pose.pose.position.x = current_path[i](0);
            my_desired_pose.pose.position.y = current_path[i](1);
            my_desired_pose.pose.position.z = current_path[i](2);

            //solve for desired position
            baxter_core_msgs::SolvePositionIK::Request req;
            baxter_core_msgs::SolvePositionIK::Response res;
            req.pose_stamp.push_back(my_desired_pose);
            ik_solver.call(req, res);
            if(!res.isValid.empty()){
                trajectory_msgs::JointTrajectoryPoint pt;

                pt.positions = res.joints[0].position;
                pt.velocities.resize(7, 0.0);
                pt.accelerations.resize(7, 0.0);
                pt.effort.resize(7, 0.0);
                pt.time_from_start = ros::Duration(i*1);
                if(!pt.positions.empty())
                    trajectory.points.push_back(pt);
                else{
                    ROS_WARN_STREAM("point with no solution : " <<
                                    current_path[i](0) << " " <<
                                    current_path[i](1) << " " <<
                                    current_path[i](2));
                            //if(try_different_seed){
                            ROS_INFO("trying to solve for it ...");
                    req.pose_stamp.clear();
                    req.pose_stamp.push_back(my_desired_pose);
                    sensor_msgs::JointState new_state;
                    robot_state.setToRandomPositions(robot_state.getJointModelGroup("left_arm"));
                    new_state.name = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};

                    //new_state.position = {-0.08, -1.0, -1.19, 1.94, 0.67, 1.03, -0.5};
                    robot_state.copyJointGroupPositions("left_arm", new_state.position);
                    req.seed_angles.push_back(new_state);
                    ik_solver.call(req, res);

                    pt.positions = res.joints[0].position;
                    pt.velocities.resize(7, 0.0);
                    pt.accelerations.resize(7, 0.0);
                    pt.effort.resize(7, 0.0);
                    pt.time_from_start = ros::Duration(i*1);
                    if(!pt.positions.empty()){
                        ROS_INFO_STREAM("solution is found :)");
                        trajectory.points.push_back(pt);
                    }
                    //}
                }
            }
            else{
                ROS_ERROR_STREAM("point with no solution!!!!!!!!: " <<
                                 current_path[i](0) << "," <<
                                 current_path[i](1) << "," <<
                                 current_path[i](2));
                        res_move = false;
            }
        }

        //at the end push joints names
        trajectory.joint_names = {"left_s0", "left_s1", "left_e0",
                                  "left_e1", "left_w0", "left_w1",
                                  "left_w2"};
    }
    else { // no ik solver
        ROS_ERROR_STREAM("No IK solver available");
        exit(-1);
    }

    return res_move;
}

/**
 * @brief execute a trajectory composed by joint config
 * @param b
**/
void execute_joint_config_traj(ros::Publisher &pub_msg,
                               trajectory_msgs::JointTrajectory &trajectory){

    //execute the trajectory
    if(!trajectory.points.empty()){
        for (int j = 0; (unsigned)j < trajectory.points.size(); ++j){
            boost::timer time_elapsed;
            time_elapsed.restart();
            baxter_core_msgs::JointCommand command_msg;
            command_msg.mode = command_msg.POSITION_MODE;
            command_msg.names = trajectory.joint_names;
            command_msg.command = trajectory.points[j].positions;
            while(time_elapsed.elapsed() < 2.0)
                pub_msg.publish(command_msg);
        }
    }
}
