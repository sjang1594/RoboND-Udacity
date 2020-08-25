# Robotics-UD-HomeServiceRobot

The goal of this final project is to implement a robot can autonomously map the environment within set-up simulation map, navigate to pick-up zone, and drop the virtual objects into drop-zone. 

For this project, this was the progress.
1. Design & create a simple gazebo world(environment) with building and model editor. 
2. Clone all ROS package such as gmapping, turtlebot_teleop, turtlebot_rviz_launcher, and turtlebot_gazebo
3. Create test_slam.sh to launch turtlebot_world, to launch slam_gmapping for SLAM, to observe the map in RVIZ, and to manually control the robot with keyboard commands.
4. Create pick_object with move_base_msgs, actionlib, and roscpp dependencies for picking up the object and deliver it to designated location.
5. Create & Test pick_object.sh (launch turtlebot, amcl, rviz, pick_object.sh)
5. Create add_markers package with roscpp and visualization_msgs for virtual object. 
6. Create & Test add_marker.sh (launch turtlebot, amcl, rviz, add_markers)
7. Create & Test homeservice.sh (launch turtlebot, amcl, rviz config file, pick_objects, and add_marker)

## Project Setup.

### With proper setup, use previous project environment : MapMyWorld.

Update System: 
`sudo apt-get update && sudo apt-get upgrade -y`

Create catkin workspace:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
```
Clone all the necessary packages into src folder
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/turtlebot/turtlebot.git
$ git clone https://github.com/turtlebot/turtlebot_interactions.git
$ git clone https://github.com/turtlebot/turtlebot_simulator.git
```
Install package dpendencies with `rosdep install [package_name]`
Before installing, `rosdep check [package_name]` can be done to checkk all the requirement is ready to go.

HomeServiceRobot - File Tree
catkin_ws/src
```
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├── World                          # world files
    │   ├── ...
    ├── ShellScripts                   # shell scripts files
    │   ├── ...
    ├──RvizConfig                      # rviz configuration files
    │   ├── ...
    ├──wall_follower                   # wall_follower C++ node
    │   ├── src/wall_follower.cpp
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```






  
