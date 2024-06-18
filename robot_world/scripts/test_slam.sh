#!/bin/sh

# Launch world and deploy robot in it
xterm  -e  " source devel/setup.bash; roslaunch robot_world smallworld.launch " &
sleep 5

# Launch gmapping with tuned parameters to create a 2D occupancy grid map
xterm  -e  " source devel/setup.bash; roslaunch robot_world gmapping_demo.launch " &
sleep 5

# Launch rviz with saved configurations 
xterm  -e " source devel/setup.bash; roslaunch robot_world view_navigation.launch" &
sleep 5

# Launch teleop to teleoperate the robot and create a map 
xterm  -e  " source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py "
