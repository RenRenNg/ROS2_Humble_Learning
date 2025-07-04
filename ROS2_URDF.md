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
```python
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
