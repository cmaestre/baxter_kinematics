# Introduction

Services for the Baxter robot (both the physical one and in simulation)

# How to

## Dependencies (using Ubuntu 14.04 LTS)

Ros Indigo : http://wiki.ros.org/indigo/Installation/Ubuntu

MoveIt! : http://moveit.ros.org/install/

Baxter SDK : http://sdk.rethinkrobotics.com/wiki/Workstation_Setup

(To run in Gazebo) Baxter model in Gazebo : http://sdk.rethinkrobotics.com/wiki/Simulator_Installation

## Install :
git clone https://github.com/cmaestre/baxter_kinematics.git 

## Run (with the real robot switched on or a simulated environment loaded, as with roslaunch baxter_gazebo baxter_world.launch):

roslaunch baxter_kinematics run_services.launch

(In simulation) roslaunch baxter_kinematics run_services.launch real_baxter:=false

## Check :

TIP!! When calling a service from a terminal, write the name (i.e. rosservice call /baxter_kinematics/move_to_position), and then double-click the TAB key to get the list of available parameters to set

rosservice list | grep baxter_kinematics

/baxter_kinematics/execute_trajectory
/baxter_kinematics/get_gripper_openness
/baxter_kinematics/gripper_action
/baxter_kinematics/move_to_position
/baxter_kinematics/restart_robot

## Services (some examples) :

### Restart robot to initial position:
rosservice call /baxter_kinematics/restart_robot

### Remove right eef from table (to avoid collisions):
rosservice call /baxter_kinematics/move_to_position "{eef_name: 'right', x: 0.3, y: -0.6, z: 0.0, force_orien: false}"

### Example of moving the left eef without feedback:
rosservice call /baxter_kinematics/execute_trajectory "eef_name: 'left'
trajectory: [0.65, 0, 0, 0.45, 0.3, 0.2, 0.75, -0.3, -0.05]
gripper_values: []
feedback: false"

### Example of moving the left eef with feedback to be improved:
Listening to eef feedback:
rostopic echo /baxter_kinematics/traj_exec_info

Trajectory with feedback (only indicating the last waypiint reached):
rosservice call /baxter_kinematics/execute_trajectory "eef_name: 'left'
trajectory: [0.65, 0.1, 0, 0.55, -0.3, -0.05, 0.75, 0, 0.3]
gripper_values: []
feedback: true"

Expected feedback (do not take -999.0 into account):
---
layout: 
  dim: []
  data_offset: 0
data: [0.7499918674594362, 0.00021892203344442827, 0.29978400359396273, -999.0, -999.0, -999.0]

### Grasping:

First, calibrate the Baxter grippers
rosrun baxter_examples gripper_keyboard.py

Then push C and c

Let's get the current state (openness is 0, therefore its closed):
rosservice call /baxter_kinematics/get_gripper_openness "eef_name: 'left'"
openness: -3.73201942239e-07

Let's open it a little bit:
rosservice call /baxter_kinematics/gripper_action "eef_name: 'left'
action: 'open_slow'"

Now fully open (openness is 1):
rosservice call /baxter_kinematics/gripper_action "eef_name: 'left'
action: 'open'"

rosservice call /baxter_kinematics/get_gripper_openness "eef_name: 'left'"
openness: 99.9999008179
