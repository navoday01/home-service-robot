# Home Service Robot

This project aims to perform mapping using RGB-D based RTAB-Map (Real-Time Appearance-Based Mapping) Algorithm.

![Alt text](assets/robotworld.gif)
:--:
*Home Service Robot*

## Directory Tree 

```
.home-service-robot                                 # Home Service Robot Project
├── add_markers                                     # add markers package
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── add_markers.cpp
├── assets                                          # simulation media
│   ├── robotworld.gif
│   └── robotworld.mp4
├── CMakeLists.txt
├── pick_objects                                    # pick objects package
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── pick_objects.cpp
├── robot_world
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── __MACOSX
│   ├── include
│   │   └── robot_world
│   ├── launch                                       # launch files for all packages
│   │   ├── amcl.launch
│   │   ├── gmapping_demo.launch
│   │   ├── robot_description.launch
│   │   ├── smallworld.launch
│   │   └── view_navigation.launch
│   ├── maps
│   │   ├── world.pgm
│   │   └── world.yaml
│   ├── meshes                                      # meshes folder for sensors
│   │   ├── bases
│   │   │   └── burger_base.stl
│   │   ├── hokuyo.dae
│   │   ├── RpiCamera.stl
│   │   ├── sensors
│   │   │   ├── astra.dae
│   │   │   ├── astra.jpg
│   │   │   ├── lds.stl
│   │   │   ├── r200.dae
│   │   │   └── r200.jpg
│   │   └── wheels
│   │       ├── left_tire.stl
│   │       └── right_tire.stl
│   ├── package.xml
│   ├── rviz
│   │   └── amcl.rviz
│   ├── scripts                                     # shell scripts 
│   │   ├── add_markers.sh
│   │   ├── home_service.sh
│   │   ├── pick_objects.sh
│   │   ├── test_navigation.sh
│   │   └── test_slam.sh
│   ├── urdf                                        # urdf folder for xarco files
│   │   ├── common_properties.xacro
│   │   ├── my_robot.gazebo
│   │   └── turtlebot3_burger.urdf.xacro
│   └── worlds                                      # world files
│       └── smallworld.world
└── teleop_twist_keyboard                           # teleop twist keyboard package

21 directories, 53 files
```
## Turtlebot3 Package

This project is developed in ROS Noetic which supports Turtlebot3. Install all turtlebot-related packages using the following command so that you don't have to do `rosdep install` for every package.
```
sudo apt install ros-noetic-turtlebot3*
```

## Launch the Project

To launch the project, clone the github repository under src folder is your ros workspace by
```
git clone https://github.com/navoday01/home-service-robot.git
```
Build and source the workspace and then go to scripts directory
```
cd src/home-service-robot/robot_world/scripts
```
make shell scripts executable by
```
sudo chmod +x *.sh
```
## SLAM

To test the SLAM algorithm and create a new map of the environment, run:
```
./test_slam.sh
```

Run telop package to navigate through the environment by:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

Once the house is fully explored, save the map with:
```
rosrun map_server map_saver -f <desired directory to save map>/<map name>
```
## Navigation

To test the navigation algorithm, run:
```
./test_navigation.sh
```
We will be using the already generated map and localize with AMCL. Press the `2D Nav Goal` button in Rviz and click somewhere on the map to command the robot to navigate there.

## Home Service

To simulate a full home service robot capable of navigating to pick up and deliver virtual objects, run:
```
./home_service.sh
```

