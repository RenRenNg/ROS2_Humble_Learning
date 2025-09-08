# **ROS2 - TF|URDF|RViz|Gazebo**  
https://wiki.ros.org/urdf/XML #Useful doc on urdf properties  
## **Rviz2 Notes**  
### **Axes**  
x axis: red (forward)  
y axis: green (left)  
z axis: blue (upward)  
Right hand rule  
### **Setting the Joints/Links for TFs**  
- Fix the origin of the joint
- Fix the axis of the joint
- Fix origin of the link if needed  
### **TF Tools**  
sudo aot install ros-humble-tf2-tools #Required only If package not install  
ros2 run tf2_tools view_frames #Produce a pdf of the list of transform between links. File found on where the command is used. (Good for troublshooting joints)  
### **URDF Tools**  
ros2 launch urdf_tutorial display.launch.py model:=/PATH/ROBOT_FILENAME.urdf  
### **Template for URDF**  
```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
        <visual>
            <geometry>
            </geometry>
        </visual>
    </link>
</robot> 
```  

## **Intro to TF (TransForm)**  
### **Intro**  
cd ~  
sudo apt install ros-humble-urdf-tutorial  
source /opt/ros/humble/setup.bash or source ~/.bashrc (if you have place the source command inside bashrc)  
cd /opt/ros/humble #This where all the packages are installed  
cd /opt/ros/humble/share/urdf_tutorial/urdf  
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro  
What is Rviz:  
A 3D visualisation tool  
There are displays on the left; gird, RobotModel and TF  
Checking and unchecking TF will show/hide the axis  
RobotModel > Links > Show all links of the robot  
Back to the TF:  
**Are the links between the axes specifically with respect to which link**  
**And how they move relative to each other**  
### **Relationship between TFs, TF tree**  
While the Rviz is running  
ros2 topic list > /tf   
ros2 topic echo /tf #Show all the TFs
### **What problem are we tryng to solve with TF?**  
What we try to achieve with TFs:  
- Keep a structured tree for all joints/frames over time  
We want to know:  
- How frames are placed relative to one another  
- How thry move relative to each other  

How to compute TransForms?  
- We need to keep track of each frame relative to other frames  
- Solutions: ROS TF functionality  

How to create TF with ROS?
- Understanding TF is the most important
- Then you dont need to implement TF directly. Instead, you'll create a URDF file and use existing ROS packages  
## **Create a URDF for a Robot**  
### **Intro - What is URDF?**  
Unified Robot Description Format (URDF)  
- Description of all the elements in the robot
- Used to generate TFs  
- XML format

Most important thing to get:
- How to assemble 2 parts (links) together of the robot with a joint

### **Your first URDF file: Create and Visualise a Link**  
cd ~  
touch my_robot.urdf  
code my_robot.urdf  
#### **Inside my_robot.urdf**  
```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <!-- A box shape with dimensions L x W x H in metres -->
                <box size="0.6 0.4 0.2" /> 
            </geometry>
            <!-- Coordinates in xyz and roll, pitch, yaw -->
            <origin xyz="0 0 0.1" rpy="0 0 0" />
        </visual>
    </link>
</robot>
```
Save  
ros2 launch urdf_tutorial display.launch.py model:=/home/aaron/my_robot.urdf  
### **Material - Add some colours**  
Defining and assigning colours    
```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <!-- Defining colours -->
    <material name="grey">
        <color rgba="0.7 0.7 0.7 1"/>
    </material>

    <material name="green">
        <color rgba="0 0.6 0 1"/>
    </material>

    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <!-- A box shape with dimensions L x W x H in metres -->
                <box size="0.6 0.4 0.2" /> 
            </geometry>
            <!-- Coordinates in xyz andd roll, pitch, yaw -->
            <origin xyz="0.0 0 0.1" rpy="0 0 0" />
            <!-- Assign colour to object -->
            <material name="green"/>
        </visual>
    </link>
</robot>
```
Save  
ros2 launch urdf_tutorial display.launch.py model:=/home/aaron/my_robot.urdf    
### **Combine 2 Links with a Joint**    
When naming the joint have the convention to use the name of the 2 elements and combine them together. (e.g. base_lidar_joint)   
**Useful tip:**  
- Fix the origin of the joint
- Fix the axis of the joint
- Fix origin of the link if needed
```xml
    <link name="lidar_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="0 0 0" />
            <material name="white"/>
        </visual>
    </link>

    <!-- Add relationship between the links -->
    <joint name="base_lidar_joint" type="fixed">
        <parent link="base_link"/>
        <child link="lidar_link"/>
        <!-- Change this to fix where it sit on the robot. If not enough change the link origins. -->
        <origin xyz="0.0 0 0.225" rpy="0 0 0" />
    </joint>
```  
## **Broadcast TFs with the Robot State Publisher**  
### **Intro**  
Create and package a ROS2 package application, around URDF  
This process can be applied to any robot  

### **How the Robot State Publisher and URDF Work Together**  
ros2 launch urdf_tutorial display.launch.py model:=/home/aaron/my_robot.urdf  
rqt_graph 
<img width="1324" height="134" alt="image" src="https://github.com/user-attachments/assets/f47bd607-2ce1-4629-914c-44f201194be0" />  
ros2 topic list > see /tf  
ros2 node list > see /robot_state_publisher  
ros2 param list /robot_state_publisher > see robot description  
ros2 param get /robot_state_publisher robot_description > URDF of the file  
ros2 topic echo /joint_states > Shows the joint state of the of robot    
<img width="1131" height="374" alt="image" src="https://github.com/user-attachments/assets/f89b9810-bf58-4094-a37c-3231498cdd27" />   
Pass the urdf to the robot_state_publisher by using parameter.  
State of each joint published on a joint_state topic.  
Using urdf and joint state to compute TF on tf topic.  
Then TF will be used another node like navigation stack (e.g. moving).  
### **Run the Robot State Publisher with URDF in the Terminal**  
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf)"  
sudo apt install ros-humble-xacro (if missing package)  
rqt_graph  
<img width="856" height="205" alt="image" src="https://github.com/user-attachments/assets/83668720-256c-4166-a215-05d25d2de080" />  
Nothing is publishing to joint_state topic.  
ros2 run joint_state_publisher_gui joint_state_publisher_gui   
sudo apt install ros-humble-joint-state-publisher-gui (if missing package)  
<img width="1033" height="203" alt="image" src="https://github.com/user-attachments/assets/97ac1d74-2f00-4a8a-aa01-c64fa883ee68" />  
ros2 run rviz2 rviz2 > add robotmodel & TF > Global options > Fixed frame > base_footprint  
robotmodel > Description Topic > select /robot_description   





