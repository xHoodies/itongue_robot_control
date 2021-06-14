# itongue_robot_control
Implementation of an assistive robotic solution for tetraplegic patients.  The main purposeof  this  project  is  to  create  a  system  that helps  tetraplegics,  using  the  given  equipment: JACO2robotic  arm  developed  by Kinova Roboticsand an iTongue control interface for manual movement.


The jaco package include the code needed for controlling a jaco Kinova robot arm with an iTongue control unit.

The depth_pkg package include the classification code for finding specific shapes and colours with a RGB-D camera.

The two packages together make it possible for a user with the iTongue control unit to select a bottle in close proximity, and have the robot arm automatically pick it up, guiding the bottle to the users mouth safely.


INITIALIZATION:

Important files:

debth_pkg/src: 
debthcamera.cpp

jaco/pubsub/src: 
talker.cpp (controller)
keystate.cpp

jaco/kinova_demo/nodes/kinova_demo:
fingers_action_client_copy.py
pose_acion_client_copy.py


RUN

Terminal
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300 
		To make connection and it is a roscore. 

New terminal
cd ~/debthcamera_ws
source devel/setup.bash
rosrun debth_pkg debthcammera

New terminal
cd  ~/jaco
source devel/setup.bash
rosrun pubsub talker

New terminal
cd  ~/jaco
source devel/setup.bash
rosrun pubsub keystate

New terminal
cd  ~/jaco
source devel/setup.bash
rosrun kinova_demo fingers_action_client_copy.py

New terminal
cd  ~/jaco
source devel/setup.bash
rosrun kinova_demo pose_acion_client_copy.py
