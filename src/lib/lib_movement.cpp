#include "../../include/baxter_kinematics/lib_movement.hpp"
#include <sstream>

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

/** restart arms to initial safe position (before any movement)
 * @brief a
 * @param b
**/
bool restart_robot_initial_position(Kinematic_values& eef_values,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left,
                                    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_right,
                                    ros::NodeHandle nh){
    double tmp_x, tmp_y, tmp_z;
    std::vector<geometry_msgs::Pose> waypoints;
    //////////////////////////////////////////////////////////////////////////
    // Left arm
    ROS_INFO("\nLeft arm to initial pose");

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
                                           ac_left,
                                           eef_values,
                                           nh,
                                           true); // force_orien
    if (!left_res){
        ROS_ERROR_STREAM("restart_robot_initial_position : Failed executing left arm motion");
        return false;
    }

    ////////////////////////////////////////////////////////////////////////
    // Right arm
    ROS_INFO("\nRight arm to initial position");

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
    waypoints.clear();
    waypoints.push_back(initial_pose);
    ROS_ERROR_STREAM("Right eef initial position : " << initial_pose.position.x << " " << initial_pose.position.y << " " << initial_pose.position.z);
    bool right_res =
            plan_and_execute_waypoint_traj("right",
                                           waypoints,
                                           ac_right,
                                           eef_values,
                                           nh,
                                           true); // force_orien

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
bool plan_path_to_desired_position(Eigen::Vector3d goal,
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
        //Eigen::Vector3d over_eef_position = eef_values.get_eef_position(gripper);
        // if(!eef_values.get_keep_height() && eef_values.get_eef_position(gripper)(2) < 0.0)
        //     over_eef_position[2] += 0.15;
        //path.push_back(over_eef_position);
        path.push_back(goal);
    }
    //if it reaches here it means everything went well
    return true;
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
    while(!plan_path_to_desired_position(goal_position,
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
                                   actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
                                   Kinematic_values& eef_values,
                                   ros::NodeHandle nh,
                                   bool force_orien,
                                   bool feedback_data,
                                   ros::Publisher traj_res_pub,
                                   std::string object_name,
                                   std::vector<std::string> gripper_values,
                                   ros::ServiceClient gripper_client){

    // initially always moves
    nh.setParam("env_changed", false);

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

    // initially use same orientation fot the whole traj
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
    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);
    ROS_WARN_STREAM("fraction solved of desired path in this trial is: " <<
                    fraction);  //eef_jump_step size determine the speed of resulted motion

    //this part is to look for an orientation that will produce 100% path, we change only pitch angle because in this setup it is the main angle that will give the required behavior
    int pitch_counter = 0, sign = 1;
    double step = 0.1;
    int trials = 0;

    // if it not possible to find a trajectory with the corrent orientation, then
    // use a quaterion that will be used to look for valid orientation to return a complete path
    tf::Quaternion orientation;
    double roll, pitch, yaw, max_ang = M_PI, min_ang = -M_PI;
    roll = eef_values.get_eef_rpy_orientation(eef_selected)(0);
    pitch = eef_values.get_eef_rpy_orientation(eef_selected)(1);
    yaw = eef_values.get_eef_rpy_orientation(eef_selected)(2);

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
        // force vertical orientation of final WP
        if (force_orien){
            wp_itr = waypoints.end();
            wp_itr->orientation = correct_orientation.orientation;
        }

        fraction = group.computeCartesianPath(waypoints, 0.025, 0.0, robot_trajectory);
        pitch_counter += 1;
        trials += 1;
    }

    if (trials == 50){
        ROS_ERROR_STREAM("NOT solution found for trajectory");
        return 2;
    }

//    if(trials > 0 && fraction == 1){
//        ROS_ERROR_STREAM("z coordinate is: " << (waypoints[waypoints.size() - 1]).position.z);
//    }

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
//    int added_waypoint = 0;
    ROS_ERROR_STREAM("waypoints size is: " << waypoints.size());

    bool real_robot;
    nh.getParam("real_robot", real_robot);
    ros::ServiceClient client_get_object_pose;
    visual_functionalities::GetObjectStateBlob getObjectStateSrvBlob;
    environment_functionalities::GetObjectState getObjectStateSrv;
    if (real_robot)
        client_get_object_pose = nh.serviceClient<visual_functionalities::GetObjectStateBlob>("/visual/get_object_state_blob");
        client_get_object_pose = nh.serviceClient<environment_functionalities::GetObjectState>("/env/get_object_state");
    std_msgs::Empty empty;
    std::vector<Eigen::Vector3d> eef_position_vector;
    std::vector<Eigen::Vector3d> eef_orientation_vector;
    std::vector<Eigen::Vector3d> object_position_vector;
    std::vector<Eigen::Vector3d> object_orientation_vector;
    int nb_wp_reached = 0;
    if(fraction == 1){

        std::vector<double> curr_eff_position(3), expected_traj_position(3);
        double curr_distance;
        bool next_wp_reached;
        bool timeOut = false;        
        std::string tmp_env_changed;
        nh.getParam("env_changed", tmp_env_changed);
        bool env_changed;
        std::istringstream(tmp_env_changed) >> std::boolalpha >> env_changed;
        size_t nb_wp_to_reach = 0;
        while (!env_changed && (nb_wp_to_reach < waypoints.size())){

            expected_traj_position[0] = waypoints[nb_wp_to_reach].position.x;
            expected_traj_position[1] = waypoints[nb_wp_to_reach].position.y;
            expected_traj_position[2] = waypoints[nb_wp_to_reach].position.z;

            next_wp_reached = false;
            size_t tmp_counter = 0;            
            while (!env_changed && !timeOut && !next_wp_reached){
                eef_pose = eef_values.get_eef_position(eef_selected);
                curr_eff_position[0] = eef_pose(0);
                curr_eff_position[1] = eef_pose(1);
                curr_eff_position[2] = eef_pose(2);

                curr_distance = largest_difference(curr_eff_position, expected_traj_position);
                //ROS_ERROR_STREAM("Distance to WP: " << nb_wp_to_reach << " is " << curr_distance << " iteration " << tmp_counter);

                if (curr_distance < 0.025){
                    ROS_ERROR_STREAM("WP " << nb_wp_to_reach << " REACHED for feedback");
                    // open/close gripper
                    if (gripper_values.size() > 0){
                        std::string curr_gripper_value = gripper_values[nb_wp_to_reach];
                        baxter_kinematics::GripperAction srv;
                        srv.request.eef_name = selected_eef;
                        srv.request.action = curr_gripper_value; //curr_gripper_value_bool;
                        if (gripper_client.call(srv))
                            ROS_ERROR_STREAM("Set gripper value " << curr_gripper_value << " for wp " << nb_wp_to_reach);
                        else
                            ROS_ERROR("plan_and_execute_waypoint_traj - Failed to execute gripper action");
                    }

                    if(feedback_data){
//                        ROS_ERROR_STREAM("STORE/PUBLISH FEEDBACK");

                        real_traj_to_publish.data.clear();

                        //save eef values
                        eef_pose = eef_values.get_eef_position(eef_selected);
                        eef_position_vector.push_back(eef_pose);
                        eef_orientation_vector.push_back(eef_values.get_eef_rpy_orientation(eef_selected));

                        //save object values
                        std::vector<double> object_state_vector;
                        if (real_robot) {
                            getObjectStateSrvBlob.request.request_position = empty;
                            client_get_object_pose.call(getObjectStateSrvBlob);
                            object_state_vector = getObjectStateSrvBlob.response.model_state;
                        } else {
                            getObjectStateSrv.request.object_name = object_name;
                            client_get_object_pose.call(getObjectStateSrv);
                            object_state_vector = getObjectStateSrv.response.object_state;
                        }
                        object_state_vector.push_back(0); // fake orientation
                        object_state_vector.push_back(0);
                        object_state_vector.push_back(0);

                        Eigen::Vector3d current_object_position;
                        current_object_position <<  object_state_vector[0],
                                                    object_state_vector[1],
                                                    object_state_vector[2];
                        object_position_vector.push_back(current_object_position);

                        Eigen::Vector3d current_object_orientation;
                        current_object_orientation << object_state_vector[3],
                                                      object_state_vector[4],
                                                      object_state_vector[5];
                        object_orientation_vector.push_back(current_object_orientation);

                        //store to publish afterwards
                        real_traj_to_publish.data.push_back(eef_pose(0));
                        real_traj_to_publish.data.push_back(eef_pose(1));
                        real_traj_to_publish.data.push_back(eef_pose(2));
                        real_traj_to_publish.data.push_back(object_state_vector[0]);
                        real_traj_to_publish.data.push_back(object_state_vector[1]);
                        real_traj_to_publish.data.push_back(object_state_vector[2]);

                        // publish current trajectory
                        if (real_traj_to_publish.data.size() > 0) {
                            ROS_ERROR_STREAM("Traj feedback until WP " << nb_wp_reached);
                            traj_res_pub.publish(real_traj_to_publish);
                        }
                    }
                    next_wp_reached = true;
                    nb_wp_reached += 1;
                } // if wp reached

                nh.getParam("env_changed", env_changed);
                tmp_counter++;
                if (tmp_counter > 50000)
                    timeOut = true;                                
            } // while

            if (timeOut)
                ROS_ERROR_STREAM(" WP " << nb_wp_to_reach << "NOT REACHED !!");
            nb_wp_to_reach++;
        } // external while

//        if(env_changed)
//            ac.cancelGoal();
//        else{
//            if (feedback_data){
//                sleep(1);
//                // add last eef values
//                eef_pose = eef_values.get_eef_position(eef_selected);
//                eef_position_vector.push_back(eef_pose);
//                eef_orientation_vector.push_back(eef_values.get_eef_rpy_orientation(eef_selected));

//                //add last object values
//                getObjectStateSrv.request.object_name = object_name;
//                client_get_object_pose.call(getObjectStateSrv);
//                std::vector<double> object_state_vector = getObjectStateSrv.response.object_state;

//                Eigen::Vector3d current_object_position;
//                current_object_position <<  object_state_vector[0],
//                                            object_state_vector[1],
//                                            object_state_vector[2];
//                object_position_vector.push_back(current_object_position);

//                Eigen::Vector3d current_object_orientation;
//                current_object_orientation << object_state_vector[3],
//                                              object_state_vector[4],
//                                              object_state_vector[5];
//                object_orientation_vector.push_back(current_object_orientation);

//                //store to publish
//                real_traj_to_publish.data.push_back(eef_pose(0));
//                real_traj_to_publish.data.push_back(eef_pose(1));
//                real_traj_to_publish.data.push_back(eef_pose(2));
//                real_traj_to_publish.data.push_back(object_state_vector[0]);
//                real_traj_to_publish.data.push_back(object_state_vector[1]);
//                real_traj_to_publish.data.push_back(object_state_vector[2]);

//                // publish full trajectory
//                if (real_traj_to_publish.data.size() > 0) {
//                    ROS_ERROR_STREAM("Printing full traj in topic");
//                    traj_res_pub.publish(real_traj_to_publish);
//                } else
//                    ROS_ERROR_STREAM("NOTHING TO PUBLISH !!! ");
//            } // if feedback
//        } // else
    } // if fraction

    ROS_ERROR_STREAM("Number of reached wp is: " << nb_wp_reached << "/" << waypoints.size());

//    // always ready to move
//    nh.setParam("env_changed", false);

    return 1;
}
