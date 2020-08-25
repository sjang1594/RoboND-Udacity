#!/bin/sh

#Setting up the Path
catkin_path="/home/workspace/catkin_ws"

#Launch the world file from turtlebot package
xterm -e "cd ${catkin_path}; source devel/setup.bash; 
roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

#Launch amcl 
xterm -e "cd ${catkin_path}; source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5

#Launch RVIZ
xterm -e "cd ${catkin_path}; source devel/setup.bash; 
roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

#Launch add_markers NODE
xterm -e " cd ${catkin_path}; source devel/setup.bash;
rosrun add_markers add_markers" 

