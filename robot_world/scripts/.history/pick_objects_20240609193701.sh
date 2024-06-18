#!/bin/sh

# Launch home world and debloy my_robot in it
xterm  -e  " source devel/setup.bash; roslaunch robot_world smallworld.launch " &
sleep 10

# Launch amcl node
xterm  -e  " source devel/setup.bash; roslaunch robot_world amcl.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e " source devel/setup.bash; roslaunch robot_world view_navigation.launch" &
sleep 20

# Launch pick objects node
xterm  -e  " source devel/setup.bash; rosrun pick_objects pick_objects "