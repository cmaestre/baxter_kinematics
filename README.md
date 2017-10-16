# Introduction

Services for the Baxter robot (both the physical one and in simulation)

# How to

## Install :
git clone https://github.com/cmaestre/baxter_kinematics.git

## Run :

roslaunch baxter_kinematics run_services.launch

(In simulation) roslaunch baxter_kinematics run_services.launch real_baxter:=false

## Check :

TIP!! When calling a service from a terminal, write the name (i.e. rosservice call /baxter_kinematics/move_to_position), and then double-click the TAB key to get the list of available parameters to set

rosservice list | grep baxter_kinematics

/baxter_kinematics/execute_trajectory
/baxter_kinematics/get_gripper_openness
/baxter_kinematics/get_sim_eef_pose
/baxter_kinematics/gripper_action
/baxter_kinematics/move_to_position
/baxter_kinematics/restart_robot

## Services (some examples) :

### Restart robot to initial position:
rosservice call /baxter_kinematics/restart_robot

### Remove right eef from table (to avoid collisions):
rosservice call /baxter_kinematics/move_to_position "{eef_name: 'right', x: 0.3, y: -0.6, z: 0.0, force_orien: false}"

### Example of moving the left eef (without feedback):
rosservice call /baxter_kinematics/execute_trajectory "eef_name: 'left'
trajectory: [0.65, 0, 0, 0.45, 0.3, 0.2, 0.75, -0.3, -0.05]
gripper_values: []
feedback: false"

### Listening to eef feedback:
rostopic echo /baxter_kinematics/traj_exec_info

Trajectory with feedback:
rosservice call /baxter_kinematics/execute_trajectory "eef_name: 'left'
trajectory: [0.65, 0.1, 0, 0.55, -0.3, -0.05, 0.75, 0, 0.3]
gripper_values: []
feedback: true"

Expected feedback (do not take -999.0 into account):
layout: 
layout: 
  dim: []
  data_offset: 0
data: [0.6515854514016458, 0.09877138311717786, 0.004028279026203119, -999.0, -999.0, -999.0]
---
layout: 
  dim: []
  data_offset: 0
data: [0.550353361504443, -0.29696188867351775, -0.04954377515473407, -999.0, -999.0, -999.0]
---
layout: 
  dim: []
  data_offset: 0
data: [0.7478432800149526, -0.0026350589180668196, 0.2962306155197778, -999.0, -999.0, -999.0]

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
