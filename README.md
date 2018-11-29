# auto_sim
A lab projcet of the surgical robots -- Raven II. Autonomous agent for picking up an object and transferring it to a container. A gazebo simulator. This pipeline works without the real robot device.

Todo: description

## autonomous agent


## Gazebo Simulator
raven_2arm folder is a ROS package for the Raven II robots using Gazebo.

raven_2arm.launch is the launch file for running. The Gazebo subscribed to the joint_states topic published by r2_control which is the node of controlling the robot. For r2_control, some codes are commented to ensure it can run without connected to the real robots.

## add label to /ravenstate topic
so that we can have segment label in the csv file.

The code file is auto_sim/auto_agent/auto_circle_generator/src/dsn/add_label_ravenstate.cpp
auto_circle_generator is a ROS package for performing autonomous task.
The add_label_ravenstate will call a service I added in r2_control node which is in raven2 package to change the segment value in auto_sim/raven2/src/raven/local_io.cpp
So raven2 folder is not the same as the UW original version.


## 
