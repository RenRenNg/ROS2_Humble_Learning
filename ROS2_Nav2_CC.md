### **What is Nav2 (Navigation2 Stack)**  
#### **Benefits of ROS2**  
1. Create the **base layer** super fast
2. Provide a **standard** for robotics applications
3. Use on **any robot**
4. Allows you to **avoid reinventing the wheel**
5. Strong open source **community**
6. **Plug and play** packages

#### **What is a ROS stack**  
Collection of ROS packages created to achieve a specific goal  

#### **Navigation2 stack**   
Move a robot from point a to b in a safe way
1. Create a map with SLAM
2. Make the robot navigate from point a to b
3. Easily integrate nav2 stack into ROS2 application

### **Install Nav2 for ROS2 Humble**
cd ~  
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humbe-turtlebot3*  

### **Make your robot move in the environment**  
cd ~  
gedit ~/.bashrc
Add > export TURTLEBOT3_MODEL=waffle > above source /opt/ros/humble/setup.bash
printenv | grep TURTLE > should show TURTLEBOT3_MODEL=waffle  
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
ros2 run turtlebot3_teleop teleop_keyboard  

### **Generate a Map with SLAM**
Kill everything  
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
ros2 launch turtlebot3_cartographer cartographer.launch.py use_stim_time:=True  
ros2 run turtlebot3_teleop teleop_keyboard  

Rviz is just a tool for navigation or other applications  
Gazebo is purely simulation (physic, inertia and etc)

#### **Tips & debugging**  
Dont turn the robot too fast else the rviz cannot read correctly
Dont collide with anything  
If fail > kill everything > relaunch the application

### **Saving map**  
cd ~  
ls  
mkdir maps  
ros2 run nav2_map_server map_saver_cli -f maps/[map_name]  

#### **Inside pgm file**  
White = Free space  
Black = Obstacle  
Grey = Unknown   

#### **Inside yaml file**
```yaml
image: map_map.pgm
mode: trinary
resolution: 0.05 # metres/pixel
origin: [-1.27, -2.47, 0] # starting point with respect to coordinates from the lowest left point
negate: 0
occupied_thresh: 0.65 # if pixel > 65% occupied, consider as an obstacle
free_thresh: 0.25 # if pixel < 25% not occupied, not an obstacle
```
#### **Calculate distance of the map from pixel**  
cd ~/maps/[map_name]  
nano [map_name.pgm]  
```pgm  
P5
125 118 # x and y coordinates
255
```
x coord * resolution = 125 * 0.05 = 6.25m long on the x axis  
y coord * resolution = 116 * 0.05 = 5.8m long on the x axis

### **Quick Fix DDS Issue with Nav2**  
Change quick DDS to cyclone DDS 
sudo apt install ros-humble-rmw-cyclonedds-cpp  
gedit ~/.bashrc
Add > export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp > between source /opt/ros/humble/setup.bash & export TURTLEBOT3_MODEL=waffle  

#### **Optional**  
cd /opt/ros/jumble/share/turtlebot3_navigation2/param  
sudo gedit waffle.yaml  
ctrl + f > robot_model_type > "nav2_amcl::DifferentialMotionModel" > save > reboot  

### **Make the robot navigate using the map**  
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/map_map.yaml  

#### **Localisation**  
In Rviz > 2D Pose estimate > click, hold and drag to where the robot is and facing.  

#### **Goal**    
Click Nav2 goal > choose a spot and orientation 
controller = local planner  

### **Waypoint Follower**  
Creating a route/patrol
In Rviz > waypoint mode > Nav2 goal > choose section where you want the route to be > repeat from Nav2 goal > cotinue till desired route is finished > Start waypoint following
Start Nav through poses (Goes through waypoint endlessly) less stable than Start waypoint following  

#### **Create Workspace (ros2_ws)**  
