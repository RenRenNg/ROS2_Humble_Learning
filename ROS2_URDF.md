### **Setup URDF file**
touch my_robot.urdf  
code my_robot.urdf  # open code editor  
#### **Inside my_robot.urdf**  

dpkg -l | grep ros-humble-desktop # Check if you have ros-humble-desktop  
sudo apt install ros-humble-urdf
source /opt/ros/humble/setup.bash  
ros2 launch urdf_tutorial display.launch.py model:=/home/aaron/my_robot.urdf
