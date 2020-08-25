#!/bin/sh

#Setting up the path
catkin_path="/home/workspace/catkin_ws"

#If you want to import the specific gazebo word, the export command can be used
#Such as export TURTLEBOT_GAZEBO_WORLD_FILE = "{PATH_OF_WORLD}"

#Launch the world file from turtlebot package
xterm -e "cd ${catkin_path}; source devel/setup.bash; 
roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

#Launch gmapping to perform SLAM and map of the environment with robot.
xterm -e "cd ${catkin_path}; source devel/setup.bash;
roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5

#Launch RVIZ for visualization of robot's suronding with sensors.
xterm -e "cd ${catkin_path}; source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

#Launch teleop_key for controlling the robot in turtlebot_world. 
xterm -e " cd ${catkin_path}; source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch " &
