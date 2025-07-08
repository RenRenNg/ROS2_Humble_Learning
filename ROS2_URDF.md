### **Rviz2 Tools**  
https://wiki.ros.org/urdf/XML/link # Useful documentation about URDF  
ros2 run tf2_tools view_frames # produce a pdf of the list of transform between links

### **Rviz2 notes**
#### **Axes**  
Right hand rule, Right hand grup rule  
Red = x-axis, Roll  
Green = y-axis, Pitch  
Blue = z-axis, Yaw  

### **Setup URDF file**
touch my_robot.urdf  
code my_robot.urdf  # open code editor  
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
            <!-- Coordinates in xyz andd roll, pitch, yaw -->
            <origin xyz="0 0 0.1" rpy="0 0 0" />
        </visual>
    </link>
</robot>    
```
save file  
dpkg -l | grep ros-humble-desktop # Check if you have ros-humble-desktop  
sudo apt install ros-humble-urdf
source /opt/ros/humble/setup.bash  
ros2 launch urdf_tutorial display.launch.py model:=/home/aaron/my_robot.urdf

### **Defining & Assigning colours**
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
ros2 launch urdf_tutorial display.launch.py model:=/home/aaron/my_robot.urdf  

### **Adding a link and joint**  
Useful tip:  
1. Fix the origin of the point
2. Fix the axis
3. Fix origin of the link if needed
#### **Lidar** 
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
#### **Left Wheel**
```xml
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="1.57 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="-0.15 0.225 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>
```
#### **Right Wheel**
```xml
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="1.57 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="-0.15 -0.225 0" rpy="0 0 0" />
        <axis xyz="0 1 0"/>
    </joint>
```
#### **Caster Wheel**
```xml
    <link name="caster_wheel">
        <visual>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
            <origin xyz="0.0 0 0" rpy="0 0 0" />
            <material name="grey"/>
        </visual>
    </link>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link"/>
        <child link="caster_wheel"/>
        <origin xyz="0.2 0 -0.05" rpy="0 0 0" />
    </joint>
```
#### **Base Footprint**
```xml
    <link name="base_footprint"/>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0" />
    </joint>
```



