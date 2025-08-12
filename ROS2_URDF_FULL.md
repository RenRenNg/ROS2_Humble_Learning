# **ROS2 - TF|URDF|RViz|Gazebo**  
## **Rviz2 Notes**  
### **Axes**  
x axis: red (forward)  
y axis: green (left)  
z axis: blue (upward)  
Right hand rule  
### **TF Tools**  
sudo aot install ros-humble-tf2-tools #Required only If package not install  
ros2 run tf2_tools view_frames #Produce a pdf of the list of transform between links. File found on where the command is used. (Good for troublshooting joints)  

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
