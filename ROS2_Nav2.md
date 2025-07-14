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
printenv | grep TURTLE > should shpw TURTLEBOT3_MODEL=waffle  
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  

#### **Create Workspace (ros2_ws)**  
