# **ROS2 - TF|URDF|RViz|Gazebo**  
https://wiki.ros.org/urdf/XML #Useful doc on urdf properties  
## **Rviz2 Notes**  
https://wiki.ros.org/urdf/XML/link #URDF Link Doc  
https://wiki.ros.org/urdf/XML/joint #URDF Joint Doc  
https://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model #URDF Inertia Doc  
https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors #List of 3D Inertia  
https://classic.gazebosim.org/tutorials?tut=ros_gzplugins #Gazebo Plugins Doc  
https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins #Gazebo Github

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
### **Create a Robot Description Package to Install the URDF**    
cd ~/ros2_ws/src  
ros2 pkg create my_robot_description 
rm -rf include/ src/  
mkdir urdf  
cd ~  
mv my_robot.urdf ros2_ws/src/my_robot_description/urdf/  
cd ~/ros2_ws  
colcon build  
source install/setup.bash  
cd ~/ros2_ws/src  
code . 
#### **Inside CMakeList**  
add  
```
install(
  DIRECTORY urdf
  DESTINATION share/${project_name}/
)
```  
cd ~/ros2_ws  
colcon build  
source install/setup.bash 
~/ros2_ws/install/my_robot_description/share  
ls > urdf package  
### **Write a Launch file to start the Robot State Publisher with URDF(XML)**   
cd ~/ros2_ws/src/my_robot_description  
mkdir launch
cd launch  
touch display.launch.xml  
#### **Inside display.launch.xml**  
```xml
<launch>
  <arg name="model" default="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>

  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(var model)')"/>
  </node>

  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>

  <node pkg="rviz2" exec="rviz2" output="screen" name="rviz2"/>
</launch>
```
Add TF  
Add robotmodel > description topic > robot_description  
Global options > Fixed Frame > base_footprint  
##### **Inside display.launch.xml with rviz config set**  
```xml
<launch>
  <arg name="model" default="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
  <arg name="rviz_config" default="$(find-pkg-share my_robot_description)/rviz/my_rviz_config.rviz"/>

  <!-- Publish TF from the URDF -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(var model)')"/>
  </node>

  <!-- GUI for publishing joint states -->
  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>

  <!-- Launch RViz with config -->
  <node pkg="rviz2" exec="rviz2" output="screen" name="rviz2" args="-d $(var rviz_config)"/>
</launch>
```  
## **Improve URDF with Xacro**  
- Properties (variables)  
- Macros (functions)  
- Include URDF file inside another URDF  
Make the URDF cleaner, more dynamic, modular, scalable
### **Make the URDF compatible with Xacro**  
Rename my_robot.urdf file extension to my_robot.urdf.xacro  
Add xmlns:xacro="http://www.ros.org/wiki/xacro" into  
```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```
build > source > launch file  
### **Create Variables with Xacro Properties**  
Declare variables  
```xacro
    <xacro:property name="base_length" value="0.6" />
```
Use variable
```xacro
    ${base_length/3.0}
```  
##### **my_robot.urdf.xacro with xacro variables**  
```xacro
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />

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

    <link name="base_footprint"/>

    <link name="base_link">
        <visual>
            <geometry>
                <!-- A box shape with dimensions L x W x H in metres -->
                <box size="${base_length} ${base_width} ${base_height}" /> 
            </geometry>
            <!-- Coordinates in xyz and roll, pitch, yaw -->
            <origin xyz="0.0 0 ${base_height/2.0}" rpy="0 0 0" />
            <!-- Assign colour to object -->
            <material name="green"/>
        </visual>
    </link>

    <link name="lidar">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="0 0 0" />
            <material name="white"/>
        </visual>
    </link>

    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="${pi/2.0} 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="${pi/2.0} 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <link name="caster_wheel">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius/2.0}"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="0 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <!-- Add relationship between the links -->
    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_lidar_joint" type="fixed">
        <parent link="base_link"/>
        <child link="lidar"/>
        <!-- Change this to fix where it sit on the robot. If not enough change the link origins. -->
        <origin xyz="0.0 0 0.225" rpy="0 0 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="${-base_length/4.0} ${base_width/2.0 + wheel_length/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="${-base_length/4.0} ${-base_width/2.0 - wheel_length/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel"/>
        <origin xyz="${base_length/3.0} 0 ${-wheel_radius/2.0}" rpy="0 0 0" />
    </joint>

</robot>
```
### **Create Functions with Xacro Properties**  
##### **my_robot.urdf.xacro with xacro functions**  
**FYI Dont optimisize too much. Unless the links is repeated 3 times or more.**  
```xacro
    <!--Create function for wheels-->
    <xacro:macro name="wheel" params="prefix">
        <link name="${prefix}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
                </geometry>
                <origin xyz="0.0 0 0" rpy="${pi/2.0} 0 0" />
                <material name="grey"/>
            </visual>
        </link>
    </xacro:macro>
    <!--Calling functions to create wheel links-->
    <xacro:wheel prefix="right"/>
    <xacro:wheel prefix="left"/>
```
### **Include Xacro File into Another Xacro File**  
**May not do it as much.**    
Create common_properties.xacro and put the materials inside.  
##### **common_properties.xacro**  
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
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
</robot>
```  
Create mobile_base.xacro and put the rest inside.  
##### **mobile_base.xacro**   
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
 <!--Creating variables-->
    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />

    <link name="base_footprint"/>

    <link name="base_link">
        <visual>
            <geometry>
                <!-- A box shape with dimensions L x W x H in metres -->
                <box size="${base_length} ${base_width} ${base_height}" /> 
            </geometry>
            <!-- Coordinates in xyz and roll, pitch, yaw -->
            <origin xyz="0.0 0 ${base_height/2.0}" rpy="0 0 0" />
            <!-- Assign colour to object -->
            <material name="green"/>
        </visual>
    </link>

    <link name="lidar">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="0 0 0" />
            <material name="white"/>
        </visual>
    </link>

    <!--Create function for wheels-->
    <xacro:macro name="wheel" params="prefix">
        <link name="${prefix}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
                </geometry>
                <origin xyz="0.0 0 0" rpy="${pi/2.0} 0 0" />
                <material name="grey"/>
            </visual>
        </link>
    </xacro:macro>
    <!--Calling functions to create wheel links-->
    <xacro:wheel prefix="right"/>
    <xacro:wheel prefix="left"/>

    <link name="caster_wheel">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius/2.0}"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="0 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <!-- Add relationship between the links -->
    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0" />
    </joint>

    <joint name="base_lidar_joint" type="fixed">
        <parent link="base_link"/>
        <child link="lidar"/>
        <!-- Change this to fix where it sit on the robot. If not enough change the link origins. -->
        <origin xyz="0.0 0 0.225" rpy="0 0 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="${-base_length/4.0} ${base_width/2.0 + wheel_length/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="${-base_length/4.0} ${-base_width/2.0 - wheel_length/2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel"/>
        <origin xyz="${base_length/3.0} 0 ${-wheel_radius/2.0}" rpy="0 0 0" />
    </joint>
</robot>
```
##### **my_robot.urdf.xacro**   
```xacro
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <xacro:include filename="common_properties.xacro" /> <!--Must be first-->
    <xacro:include filename="mobile_base.xacro" /> <!--depend on common_properties-->
   
</robot>
```
### **Xacro Command to Generate the URDF**   
Xacro command will generate a plain urdf file and pass to the robot description as hard coded values.  
Launch xacro file  
ros2 param get /robot_state_publisher robot_description > can auto generated by Xacro and hard coded values  
### **Real Meshes - Quick Overview**   
Create meshes folder under my_robot_description or package_file_name  
Add stl file into the mashes folder  
Inside CMakeList.txt > Under install code block > add meshes
Inside any robot urdf file > add
```xacro
<mesh filename="package://turtlebot3_description/meshes/bases/waffle_base.stl" scale="0.001 0.001 0.001"/>
```  
## **Simulate the robot in Gazebo**  
Difference between Rviz and Gazebo:  
- 3D visualisation/Debug tool (Rviz) vs Simulation Tool (Gazebo)

What are you going to do:   
- Understand how Gazebo is integrated with ROS
- Adapt the URDF for Gazebo
- Add a Gazebo control plugin
- Create a simulated world for the robot
### **Run gazebo**   
gazebo  
Insert shapes > move axis > rotate > play with physics  
Pause time, pause physics  
### **How Gazebo work with ROS**   
Gazebo is standalone physic tool not ROS specific  
<img width="1147" height="429" alt="image" src="https://github.com/user-attachments/assets/017114c1-1acd-4fc9-95c5-6b7ef5fe71d0" />  
gazebo_ros is the bridge between Gazebo and ROS.   
Control plugins are the simulation of the hardware (e.g. camera, sensor plugins).   
### **Add Inertia Tags in URDF**  
##### **common_properties.xacro**   
Create functions for inertia  
```xacro
    <xacro:macro name="box_inertia" params="m l w h xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}"/>
            <mass value="${m}"/>
            <inertia
                ixx="${m/12.0 * (w*w + h*h)}"
                ixy="0"
                ixz="0"
                iyy="${m/12.0 * (l*l + h*h)}"
                iyz="0"
                izz="${m/12.0 * (l*l + w*w)}"/>
        </inertial>
    </xacro:macro>

    <xacro:macro name="wheel_inertia" params="m r h xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}"/>
            <mass value="${m}"/>
            <inertia
                ixx="${1.0/12.0*m * (3*r*r + h*h)}"
                ixy="0"
                ixz="0"
                iyy="${1.0/12.0*m * (3*r*r + h*h)}"
                iyz="0"
                izz="${1.0/2.0*m*r*r}"/>
        </inertial>
    </xacro:macro>

    <xacro:macro name="caster_inertia" params="m r xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}"/>
            <mass value="${m}"/>
            <inertia
                ixx="${2.0/5.0 * m * r * r}"
                ixy="0"
                ixz="0"
                iyy="${2.0/5.0 * m * r * r}"
                iyz="0"
                izz="${2.0/5.0 * m * r * r}"/>
        </inertial>
    </xacro:macro>
```
##### **mobile_base.xacro**   
```xacro
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
  
    <!-- Physical properties -->
    <xacro:box_inertia
      m="5.0"
      l="${base_length}"
      w="${base_width}"
      h="${base_height}"
      xyz="0 0 ${base_height/2.0}"
      rpy="0 0 0"/>
  </link>
```
### **Add Collision Tags in URDF**   
In between visual tag and inertia tag (xacro)  
```xacro
            <collision>
                <geometry><cylinder radius="${wheel_radius}" length="${wheel_length}"/></geometry>
                <origin xyz="0 0 0" rpy="${pi/2.0} 0 0"/>
            </collision>
```
### **Spawn the robot in Gazebo**  
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)" #robot state publisher   
ros2 launch gazebo_ros gazebo.launch.py #Launch gazebo  
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot #Spawn robot  
OR  
### **Launch File to Start the Robot in Gazebo**  
```xml
<launch>
  <arg name="model" default="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro"/>
  <arg name="rviz_config" default="$(find-pkg-share my_robot_bringup)/rviz/my_rviz_config.rviz"/>

  <!-- Publish TF from the URDF -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(var model)')"/>
  </node>
  
  <!--Run Gazebo-->
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"/>
  
  <!-- Spawn my_robot into the world -->
  <node pkg="gazebo_ros" exec="spawn_entity.py"
        args="-topic robot_description -entity my_robot"/>

  <!-- Launch RViz with config -->
  <node pkg="rviz2" exec="rviz2" output="screen" 
        args="-d $(var rviz_config)"/>
</launch>
```
### **Fixing the Inertia Values**  
Robot is drifting backwards  
Increase Inertia > Add a "2*" to the parameter l,w,h,r.  
### **Fixing the colours in Gazebo**  
For Gazebo, the material use a different tag, gazebo tag.  
Create a new file "mobile_base_gazebo.xacro"  
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <gazebo reference="base_link">
        <material>Gazebo/Blue</material>
    </gazebo>

</robot>
```
Add `<xacro:include filename="mobile_base_gazebo.xacro" />` in my_robot.urdf.xacro  
### **Add a Gazebo Plugin to Control the robot**   
Go to https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins/include/gazebo_plugins > navigate to gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.hpp > copy the usage example and paste inside "mobile_base_gazebo.xacro  
##### **mobile_base_gazebo.xacro**  
Add
```xacro
    <gazebo>
        <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">

            <!-- Update rate in Hz -->
            <update_rate>50</update_rate>

            <!-- wheels -->
            <!-- Add the wheel joints here -->
            <left_joint>base_left_wheel_joint</left_joint>
            <right_joint>base_right_wheel_joint</right_joint>

            <!-- kinematics -->
            <!--Dist between center of left to right wheel-->
            <wheel_separation>0.45</wheel_separation> 
            <!--Twice the wheel radius-->
            <wheel_diameter>0.2</wheel_diameter> 

            <!-- output -->
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>

            <odometry_topic>odom</odometry_topic>
            <odometry_frame>odom</odometry_frame>
            <!--replace chassis as base_footprint-->
            <robot_base_frame>base_footprint</robot_base_frame> 

        </plugin>
    </gazebo>
```
### **Create a World in Gazebo**   
In gazebo > Insert > Click model.gazebo > CLick object and place  
In gazebo > Building editor > Build walls > Save  
In gazebo > File > Save as > my_world.world  
### **Launch the robot in the world**   
##### **my_robot_gazebo.launch.xml**  
```xml
<launch>
  <arg name="model" default="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro"/>
  <arg name="rviz_config" default="$(find-pkg-share my_robot_bringup)/rviz/my_rviz_config.rviz"/>

  <!-- Publish TF from the URDF -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(var model)')"/>
  </node>
  
  <!--Run Gazebo-->
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(find-pkg-share my_robot_bringup)/worlds/my_world.world"/>
  </include>
  
  <!-- Spawn my_robot into the world -->
  <node pkg="gazebo_ros" exec="spawn_entity.py"
        args="-topic robot_description -entity my_robot"/>

  <!-- Launch RViz with config -->
  <node pkg="rviz2" exec="rviz2" output="screen" 
        args="-d $(var rviz_config)"/>
</launch>
```
<img width="2424" height="1086" alt="image" src="https://github.com/user-attachments/assets/29bd199a-e0ba-40f3-b602-3102efb72469" />

## **Add a Sensor in Gazebo**  
- Add camera to the URDF
- Configure a Gazebo Plugin

Quick Intro, focus on:
- The process
- Giving you the right resources

### **Add a Camera to the URDF**   
Create "camera.xacro" file under urdf  
##### **camera.xacro**
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Camera dimensions -->
    <xacro:property name="camera_length" value="0.01"/>
    <xacro:property name="camera_width"  value="0.1"/>
    <xacro:property name="camera_height" value="0.1"/>

    <link name="camera_link">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
        </collision>
        <xacro:box_inertia 
            m="0.1" 
            l="${camera_length}" 
            w="${camera_width}" 
            h="${camera_height}"
            xyz="0 0 0" 
            rpy="0 0 0"/>
    </link>
    
    <joint name="base_camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="${(base_length+camera_length)/2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>

</robot>
```
Add `<xacro:include filename="camera.xacro" />` inside my_robot.urdf.xacro file.  
### **Add a Gazebo Plugin for Camera**   
Add gazebo tag into "camera.xacro"
##### **camera.xacro**  
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Camera dimensions -->
    <xacro:property name="camera_length" value="0.01"/>
    <xacro:property name="camera_width"  value="0.1"/>
    <xacro:property name="camera_height" value="0.1"/>

    <link name="camera_link">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
        </collision>
        <xacro:box_inertia 
            m="0.1" 
            l="${camera_length}" 
            w="${camera_width}" 
            h="${camera_height}"
            xyz="0 0 0" 
            rpy="0 0 0"/>
    </link>
    
    <joint name="base_camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="${(base_length+camera_length)/2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>

    <gazebo reference="camera_link">
        <material>Gazebo/Red</material>
        <sensor name="camera_sensor" type="camera">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>10.0</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                
                <frame_name>camera_link</frame_name>

            </plugin>
        </sensor>
    </gazebo>
</robot>
```
### **Quick fix for camera work with ROS**   
Using gazebo camera with rviz + opencv frame inconsistentency  
Create a new joint called camera_optical_joint  
Add  
```xacro
    <link name="camera_link_optical">
    </link>

    <joint name="camera_optical_joint" type="fixed">
        <!-- these values have to be these values otherwise the gazebo camera
            image won't be aligned properly with the frame it is supposedly
            originating from -->
        <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
        <parent link="camera_link"/>
        <child link="camera_link_optical"/>
    </joint>

    <gazebo reference="camera_link">
        <material>Gazebo/Red</material>
        <sensor name="camera_sensor" type="camera">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>10.0</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                
                <frame_name>camera_link_optical</frame_name>

            </plugin>
        </sensor>
    </gazebo>
```   
## **Final Project: Add an arm onto the robot**  
##### **arm.xacro**  
```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="arm_base_length" value="0.1"/>
  <xacro:property name="arm_base_width"  value="0.1"/>
  <xacro:property name="arm_base_height" value="0.02"/>
  <xacro:property name="forearm_radius"  value="0.02"/>
  <xacro:property name="forearm_length"  value="0.3"/>
  <xacro:property name="hand_radius"  value="0.02"/>
  <xacro:property name="hand_length"  value="0.3"/>

  <link name="arm_base_link">
    <visual>
      <geometry>
        <box size="${arm_base_length} ${arm_base_width} ${arm_base_height}"/>
      </geometry>
      <origin xyz="0 0 ${arm_base_height/2.0}" rpy="0 0 0"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <box size="${arm_base_length} ${arm_base_width} ${arm_base_height}"/>
      </geometry>
      <origin xyz="0 0 ${arm_base_height/2.0}" rpy="0 0 0"/>
    </collision>
    <xacro:box_inertia
            m="0.5"
            l="${2*arm_base_length}"
            w="${2*arm_base_width}"
            h="${2*arm_base_height}"
            xyz="0 0 ${arm_base_height/2.0}"
            rpy="0 0 0"/>
  </link>

  <link name="forearm_link">
    <visual>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
      <origin xyz="0 0 ${forearm_length/2.0}" rpy="0 0 0"/>
      <material name="yellow"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
      <origin xyz="0 0 ${forearm_length/2.0}" rpy="0 0 0"/>
    </collision>
    <xacro:wheel_inertia
                m="0.3"
                r="${2*forearm_radius}"
                l="${2*forearm_length}"
                xyz="0 0 ${forearm_length/2.0}"
                rpy="${pi/2.0} 0 0"/>
  </link>

  <link name="hand_link">
    <visual>
      <geometry>
        <cylinder radius="${hand_radius}" length="${hand_length}"/>
      </geometry>
      <origin xyz="0 0 ${hand_length/2.0}" rpy="0 0 0"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${hand_radius}" length="${hand_length}"/>
      </geometry>
      <origin xyz="0 0 ${hand_length/2.0}" rpy="0 0 0"/>
    </collision>
    <xacro:wheel_inertia
                m="0.3"
                r="${2*hand_radius}"
                l="${2*hand_length}"
                xyz="0 0 ${hand_length/2.0}"
                rpy="${pi/2.0} 0 0"/>
  </link>

  <joint name="arm_base_forearm_joint" type="revolute">
    <parent link="arm_base_link"/>
    <child  link="forearm_link"/>
    <origin xyz="0 0 ${arm_base_height}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${pi/2}" effort="10" velocity="1.0"/>
    <dynamics friction="0.05" damping="0.1" />
  </joint>

  <joint name="forearm_hand_joint" type="revolute">
    <parent link="forearm_link"/>
    <child  link="hand_link"/>
    <origin xyz="0 0 ${forearm_length}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${pi/2}" effort="10" velocity="1.0"/>
    <dynamics friction="0.05" damping="0.1" />
  </joint>

</robot>
```
##### **arm_gazebo.xacro**  
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <gazebo reference="arm_base_link">
        <material>Gazebo/Orange</material>
    </gazebo>

    <gazebo reference="forearm_link">
        <material>Gazebo/Yellow</material>
    </gazebo>

    <gazebo reference="hand_link">
        <material>Gazebo/Orange</material>
    </gazebo>

    <gazebo>
        <plugin name="joint_state_publisher_controller" filename="libgazebo_ros_joint_state_publisher.so">
            <!-- Update rate in Hertz -->
            <update_rate>10</update_rate>
            <!-- Name of joints in the model whose states will be published. -->
            <joint_name>arm_base_forearm_joint</joint_name>
            <joint_name>forearm_hand_joint</joint_name>
        </plugin>
    </gazebo>

    <gazebo>
        <plugin name="joint_pose_trajectory_controller" filename="libgazebo_ros_joint_pose_trajectory.so">
            <!-- Update rate in Hz -->
            <update_rate>2</update_rate>
        </plugin>
    </gazebo>
</robot>
```
## **Conclusion**  
- TF introduction
- Create URDF with links and joints
- Robot description package with config and launch file
- Improve URDF with Xacro - properties, macros
- Add inertial and collision tags in URDF
- Gazebo Plugins
- Spawn the robot in Gazebo
- Robot bringup package with Gazebo launch file
- Create a world with objects and walls
- Add a sensor to the robot
- Project: add a robotic arm on top of the robot

