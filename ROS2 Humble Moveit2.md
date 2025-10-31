# **ROS2 Humble MoveIt2 Learning**  
## **Useful tools**  
## **Setup and Installation for ROS2 and MoveIt2**  
Go to: https://moveit.ai/install-moveit2/binary/  
FYI: Change of Middleware > sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp > gedit ~/.bashrc > copy and paste export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp > above the humble source command code.  
sudo apt install ros-humble-moveit  
## **Create a URDF for a 6-axis Robotic Arm**  
### **Create a Description package**  
(If needed) Create ros2_ws: cd > mkdir ros2_ws > cd ros2_ws/ > mkdir src > colcon build > gedit ~/.bashrc > source ~/ros2_ws/install/setup.bash > save   
(If needed) Create my_robot_description inside src folder of ros2_ws: cd ~/ros2_ws/src/ > ros2 pkg create my_robot_description > cd my_robot_description > rm -r include/ src/ > mkdir urdf launch rviz > cd .. > code . > CMakeList.txt file  
##### **Inside CMakeList.txt**  
remove BUILD_TESTING code block
add  
```
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}/
)
```
Save file > cd ~/ros2_ws/ > colcon build --packages-select my_robot_description  
