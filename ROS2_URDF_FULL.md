# **ROS2 - TF|URDF|RViz|Gazebo**
## **Rviz2 Notes**
### **Axes**  
x axis: red (forward)  
y axis: green (left)  
z axis: blue (upward)  
Right hand rule  
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
On axis 
